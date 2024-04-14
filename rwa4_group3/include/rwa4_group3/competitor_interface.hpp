/**
 * @file competitor_interface.hpp
 * @author Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu),
 *   Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
 * @brief Competitor Interface class handles the competition state, order
 * processing, AGV status and AGV movement, this file is used to define the
 * functions
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

// Importing the required libraries
#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <boost/functional/hash.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rwa4_group3/utils_interface.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Competitor Interface class
 *
 */
class CompetitorInterface : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Competitor Interface object
   *
   */
  CompetitorInterface(std::string node_name)
      : Node(node_name),
        time_interval_(500.0),
        received_parts_trays1_(false),
        received_parts_trays2_(false),
        received_parts_left_(false),
        received_parts_right_(false) {
    // Create callback group 1
    group1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    options1_.callback_group = group1_;
    // Create a subscriber to the competition state
    competition_state_subscriber_ =
        this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 10,
            std::bind(&CompetitorInterface::competitionStateCallback, this,
                      std::placeholders::_1),
            options1_);
    // Create a service client to start the competition based on the competition
    // state
    start_competition_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/ariac/start_competition", rmw_qos_profile_services_default, group1_);

    // Create callback group 2
    group2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    options2_.callback_group = group2_;
    order_subscriber_ = this->create_subscription<ariac_msgs::msg::Order>(
        "/ariac/orders", 10,
        std::bind(&CompetitorInterface::orderCallback, this,
                  std::placeholders::_1),
        options2_);
    // Create a wall timer to process the orders
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(time_interval_)),
        std::bind(&CompetitorInterface::processOrders, this), group2_);
    // Create callback group 3
    group3_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options3_.callback_group = group3_;
    // Create a subscribe to /ariac/agv1_status
    agv1_status_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AGVStatus>(
            "ariac/agv1_status", 10,
            std::bind(&CompetitorInterface::agv1StatusCallback, this,
                      std::placeholders::_1),
            options3_);
    // Create a subscribe to /ariac/agv2_status
    agv2_status_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AGVStatus>(
            "ariac/agv2_status", 10,
            std::bind(&CompetitorInterface::agv2StatusCallback, this,
                      std::placeholders::_1),
            options3_);
    // Create a subscribe to /ariac/agv3_status
    agv3_status_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AGVStatus>(
            "ariac/agv3_status", 10,
            std::bind(&CompetitorInterface::agv3StatusCallback, this,
                      std::placeholders::_1),
            options3_);
    // Create a subscribe to /ariac/agv4_status
    agv4_status_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AGVStatus>(
            "ariac/agv4_status", 10,
            std::bind(&CompetitorInterface::agv4StatusCallback, this,
                      std::placeholders::_1),
            options3_);

    // Create callback group 4
    group4_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options4_.callback_group = group4_;
    // Create a subscribe to /ariac/sensors/kts1_camera/image
    kts1_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitorInterface::kts1Callback, this,
                      std::placeholders::_1),
            options4_);
    // Create a subscribe to /ariac/sensors/kts2_camera/image
    kts2_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitorInterface::kts2Callback, this,
                      std::placeholders::_1),
            options4_);
    // Create a subscribe to /ariac/sensors/left_bins_camera/image
    left_bins_camera_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitorInterface::leftBinsCameraCallback, this,
                      std::placeholders::_1),
            options4_);
    // Create a subscribe to /ariac/sensors/right_bins_camera/image
    right_bins_camera_subscriber_ =
        this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&CompetitorInterface::rightBinsCameraCallback, this,
                      std::placeholders::_1),
            options4_);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer to process the orders
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr
      competition_state_subscriber_;  ///< Subscriber to the competition state
  rclcpp::CallbackGroup::SharedPtr group1_;  ///< Callback group 1
  rclcpp::CallbackGroup::SharedPtr group2_;  ///< Callback group 2
  rclcpp::CallbackGroup::SharedPtr group3_;  ///< Callback group 3
  rclcpp::CallbackGroup::SharedPtr group4_;  ///< Callback group 4
  rclcpp::SubscriptionOptions options1_;     ///< Subscription options 1
  rclcpp::SubscriptionOptions options2_;     ///< Subscription options 2
  rclcpp::SubscriptionOptions options3_;     ///< Subscription options 3
  rclcpp::SubscriptionOptions options4_;     ///< Subscription options 4
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      start_competition_client_;  ///< Client to start the competition
  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr
      order_subscriber_;  ///< Subscriber to the orders
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr
      agv1_status_subscriber_;  ///< Subscriber to AGV 1 status
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr
      agv2_status_subscriber_;  ///< Subscriber to AGV 2 status
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr
      agv3_status_subscriber_;  ///< Subscriber to AGV 3 status
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr
      agv4_status_subscriber_;  ///< Subscriber to AGV 4 status
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      kts1_subscriber_;  ///< Subscriber to the kit tray camera sensor 1
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      kts2_subscriber_;  ///< Subscriber to the kit tray camera sensor 2
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      left_bins_camera_subscriber_;  ///< Subscriber to the left bins camera
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr
      right_bins_camera_subscriber_;  ///< Subscriber to the right bins camera
  unsigned int agv1_location_;        ///< Location of AGV 1 class attribute
  unsigned int agv2_location_;        ///< Location of AGV 2 class attribute
  unsigned int agv3_location_;        ///< Location of AGV 3 class attribute
  unsigned int agv4_location_;        ///< Location of AGV 4 class attribute
  int competition_state_;             ///< Competition state
  std::vector<Order> high_priority_orders_;  ///< High priority orders list
  std::vector<Order> low_priority_orders_;   ///< Low priority orders list
  std::vector<Order> interrupted_orders_;    ///< Interrupted orders list
  std::vector<Order> active_order_;          ///< Active orders list
  std::vector<Order> completed_orders_;      ///< Completed orders list
  double time_interval_;  ///< Time interval for the timer callback function
  std::unordered_map<int, std::vector<Trays>> trays1_;  ///< Trays1 list
  std::unordered_map<int, std::vector<Trays>> trays2_;  ///< Trays2 list
  std::unordered_map<std::pair<int, int>, std::vector<Parts>, hash_pair>
      left_bins_;  ///< Left parts list
  std::unordered_map<std::pair<int, int>, std::vector<Parts>, hash_pair>
      right_bins_;              ///< Right parts list
  bool received_parts_trays1_;  ///< Flag to check if parts are received
  bool received_parts_trays2_;  ///< Flag to check if parts are received
  bool received_parts_left_;    ///< Flag to check if parts are received
  bool received_parts_right_;   ///< Flag to check if parts are received

  // Define constants
  const uint8_t RED = 0;
  const uint8_t GREEN = 1;
  const uint8_t BLUE = 2;
  const uint8_t ORANGE = 3;
  const uint8_t PURPLE = 4;

  const uint8_t BATTERY = 10;
  const uint8_t PUMP = 11;
  const uint8_t SENSOR = 12;
  const uint8_t REGULATOR = 13;

  // Create a map to store part colors
  std::map<uint8_t, std::string> colorMap = {{RED, "Red"},
                                             {GREEN, "Green"},
                                             {BLUE, "Blue"},
                                             {ORANGE, "Orange"},
                                             {PURPLE, "Purple"}};

  // Create a map to store part types
  std::map<uint8_t, std::string> partMap = {{BATTERY, "Battery"},
                                            {PUMP, "Pump"},
                                            {SENSOR, "Sensor"},
                                            {REGULATOR, "Regulator"}};
  /**
   * @brief Declaration of the callback function for the competition state
   * subscriber
   *
   */
  void competitionStateCallback(
      const ariac_msgs::msg::CompetitionState::SharedPtr msg);
  /**
   * @brief Declaration of the callback function for the order subscriber
   *
   */
  void orderCallback(const ariac_msgs::msg::Order::SharedPtr msg);
  /**
   * @brief Declaration of the function to start the competition
   *
   */
  void startCompetition();
  /**
   * @brief Declaration of the function to handle the response of the start
   * competition service
   *
   */
  void handleStartCompetitionResponse(
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  /**
   * @brief Declaration of process order function
   *
   */
  void processOrders();
  /**
   * @brief Declaration of AGV 1 status callback function
   * @param msg
   */
  void agv1StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr msg);
  /**
   * @brief Declaration of AGV 2 status callback function
   *
   * @param msg
   */
  void agv2StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr msg);
  /**
   * @brief Declaration of AGV 3 status callback function
   *
   * @param msg
   */
  void agv3StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr msg);
  /**
   * @brief Declaration of AGV 4 status callback function
   *
   * @param msg
   */
  void agv4StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr msg);
  /**
   * @brief Decklaration of the function to lock the tray
   *
   * @param agv_id
   */
  void lockTray(int agv_id);
  /**
   * @brief Decleration of move AGV function
   *
   * @param agv_id
   * @param destination
   */
  void moveAGV(int agv_id, int destination);
  /**
   * @brief Declaration of AGV service handle
   *
   * @param future
   */
  void handleAGVStatusResponse(
      rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future);
  /**
   * @brief Declaration of Submitting Orders
   *
   */
  void submitOrder();
  /**
   * @brief Declaration of ending competition
   *
   */
  void endCompetition();
  /**
   * @brief Declaration of the callback function for the kit tray camera sensor
   * 1
   *
   * @param msg
   */
  void kts1Callback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
  /**
   * @brief Declaration of the callback function for the kit tray camera sensor
   * 2
   *
   * @param msg
   */
  void kts2Callback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
  /**
   * @brief Declaration of the callback function for the left bins camera
   *
   * @param msg
   */
  void leftBinsCameraCallback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
  /**
   * @brief Declaration of the callback function for the right bins camera
   *
   * @param msg
   */
  void rightBinsCameraCallback(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
  /**
   * @brief Declaration of the function to process the parts
   *
   * @param msg
   */
  geometry_msgs::msg::Pose worldFramePose(geometry_msgs::msg::Pose part_pose,
                                          geometry_msgs::msg::Pose sensor_pose);
  /**
   * @brief Declaration of the function to process the parts
   *
   * @param msg
   */
  void printOrderWiseParts(Order order);
};