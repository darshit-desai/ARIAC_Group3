/**
 * @file utils_interface.hpp
 * @author Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu),
 *   Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
 * @brief Utility Interface class handles the information contained in order
 * message and kitting task details
 * @version 0.1
 * @date 2024-03-29
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

// Importing the required libraries
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <chrono>
#include <string>
#include <vector>

/**
 * @brief Class for parsing the kitting task details
 *
 */
class Kitting {
 public:
  /**
   * @brief Constructor for a new Kitting object
   *
   * @param msg
   */
  Kitting(const ariac_msgs::msg::Order &msg)
      : agv_id_(msg.kitting_task.agv_number),
        tray_id_(msg.kitting_task.tray_id),
        parts_(msg.kitting_task.parts),
        destination_(msg.kitting_task.destination) {}
  /**
   * @brief Get the Agv Id object
   *
   * @return int
   */
  int getAgvId();
  /**
   * @brief Get the Tray Id object
   *
   * @return int
   */
  int getTrayId();
  /**
   * @brief Get the Parts object
   *
   * @return std::vector<ariac_msgs::msg::KittingPart>
   */
  std::vector<ariac_msgs::msg::KittingPart> getParts();
  /**
   * @brief Get the Destination object
   *
   * @return unsigned int
   */
  unsigned int getDestination();

 private:
  int agv_id_;                                       ///< agv id
  int tray_id_;                                      ///< tray id
  std::vector<ariac_msgs::msg::KittingPart> parts_;  ///< parts
  unsigned int destination_;                         ///< destination
};

/**
 * @brief Class for parsing the order details
 *
 */
class Order {
 public:
  /**
   * @brief Constructor for a new Order object
   *
   * @param msg
   */
  Order(const ariac_msgs::msg::Order &msg)
      : order_(msg),
        order_id_(msg.id),
        order_type_(msg.type),
        order_priority_(msg.priority),
        order_task_(msg),
        order_time_(std::chrono::system_clock::now()),
        incomplete_(true),
        time_passed_(0.0) {}
  /**
   * @brief Get the Order Id object
   *
   * @return std::string
   */
  std::string getOrderId();
  /**
   * @brief Get the Order Type object
   *
   * @return int
   */
  int getOrderType();
  /**
   * @brief Get the Order Priority object
   *
   * @return true
   * @return false
   */
  bool getOrderPriority();
  /**
   * @brief Get the Kitting Task object
   *
   * @return Kitting class object
   */
  Kitting getOrderTask();
  /**
   * @brief Get the Time Passed in seconds
   *
   * @return double
   */
  double getTimePassed();
  /**
   * @brief Set the Time Passed in seconds
   *
   * @param time_passed
   */
  void setTimePassed(double time_passed);
  /**
   * @brief Returns whether the order is incomplete or not
   *
   * @return true
   * @return false
   */
  bool isIncomplete();
  /**
   * @brief Sets the incomplete parameter to false
   *
   */
  void completeOrder();

 private:
  ariac_msgs::msg::Order order_;  ///< order message for parsing
  std::string order_id_;          ///< order id of the order
  int order_type_;                ///< order type like kitting, assembly etc
  bool order_priority_;           ///< order priority high or low
  Kitting order_task_;            ///< kitting task details
  std::chrono::time_point<std::chrono::system_clock> order_time_;
  bool incomplete_;     ///< flag to check if the order is incomplete
  double time_passed_;  ///< time passed since the order was received
};
