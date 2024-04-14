/**
 * @file utils_interface.hpp
 * @author Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu),
 *   Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
 * @brief Utility Interface class handles the information contained in order
 * message and kitting task details
 * @version 0.2
 * @date 2024-03-29
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

// Importing the required libraries
#include <bits/stdc++.h>

#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <boost/functional/hash.hpp>
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

/**
 * @brief class to store details of parts
 *
 */
class Parts {
 public:
  Parts(ariac_msgs::msg::Part part, geometry_msgs::msg::Pose pose_of_part)
      : part_type_(part.type),
        part_color_(part.color),
        part_pose_(pose_of_part) {}
  /**
   * @brief Get the Part Type of the object
   *
   * @return unsigned int
   */
  unsigned int getPartType();
  /**
   * @brief Get the Part Color of the object
   *
   * @return unsigned int
   */
  unsigned int getPartColor();
  geometry_msgs::msg::Pose getPartPose();

 private:
  unsigned int part_type_;              ///< part type
  unsigned int part_color_;             ///< part color
  geometry_msgs::msg::Pose part_pose_;  ///< part pose
};

/**
 * @brief class to store details of trays
 *
 */
class Trays {
 public:
  Trays(unsigned int id, geometry_msgs::msg::Pose tray_pose)
      : tray_id_(id), tray_pose_(tray_pose) {}
  /**
   * @brief Get the Tray Id of the object
   *
   * @return unsigned int
   */
  unsigned int getTrayId();
  /**
   * @brief Get the Tray Pose object
   *
   * @return geometry_msgs::msg::Pose
   */
  geometry_msgs::msg::Pose getTrayPose();

 private:
  unsigned int tray_id_;                ///< tray id
  geometry_msgs::msg::Pose tray_pose_;  ///< tray pose
};

/**
 * @brief Define a hash function for pairs
 *
 * @return Return the combined hash value
 */
struct hash_pair {
  // Overload the function call operator for pairs
  template <class T1, class T2>
  size_t operator()(const std::pair<T1, T2> &p) const {
    std::size_t seed = 0;  // Initialize the hash seed
    // Combine the hash values of the pair's first and second elements
    boost::hash_combine(seed, std::hash<T1>{}(p.first));
    boost::hash_combine(seed, std::hash<T2>{}(p.second));
    return seed;
  }
};