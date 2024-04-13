/**
 * @file competition_demo.cpp
 * @author Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu),
 *   Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
 * @brief Competitor Interface class handles the competition state, order
 * processing, AGV status and AGV movement
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

// Importing the required libraries
#include "rwa4_group3/competitor_interface.hpp"

//=========================================
void CompetitorInterface::competitionStateCallback(
    const ariac_msgs::msg::CompetitionState::SharedPtr msg) {
  competition_state_ = msg->competition_state;
  // Start the competition if the competition state is 1
  if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Competition STARTED, Competition state: " << competition_state_);
    startCompetition();
  } else if (msg->competition_state ==
             ariac_msgs::msg::CompetitionState::ENDED) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Competition ENDED, Competition state: " << competition_state_);
  }
}
//=========================================
void CompetitorInterface::startCompetition() {
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  while (
      !start_competition_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  auto future_result = start_competition_client_->async_send_request(
      request, std::bind(&CompetitorInterface::handleStartCompetitionResponse,
                         this, std::placeholders::_1));
}
//=========================================
void CompetitorInterface::handleStartCompetitionResponse(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Competition started successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Competition failed to start");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Service call failed to start competition");
  }
}
//=========================================
void CompetitorInterface::orderCallback(
    const ariac_msgs::msg::Order::SharedPtr msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), msg->id);
  Order order(*msg);
  

  if (msg->priority == true) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Adding high-priority order to the high priority order vector: "
            << order.getOrderId());
    high_priority_orders_.push_back(order);
  } else {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Adding low-priority order to the low priority order vector: "
            << order.getOrderId());
    low_priority_orders_.push_back(order);
  }
}
//=========================================
void CompetitorInterface::agv1StatusCallback(
    const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  agv1_location_ = msg->location;
}
//=========================================
void CompetitorInterface::agv2StatusCallback(
    const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  agv2_location_ = msg->location;
}
//=========================================
void CompetitorInterface::agv3StatusCallback(
    const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  agv3_location_ = msg->location;
}
//=========================================
void CompetitorInterface::agv4StatusCallback(
    const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  agv4_location_ = msg->location;
}
//=========================================
void CompetitorInterface::handleAGVStatusResponse(
    rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future) {
  auto status = future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO_STREAM(this->get_logger(), "AGV moved successfully");
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to move AGV");
    }
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Service call failed to move AGV");
  }
}
//=========================================
void CompetitorInterface::submitOrder() {
  std::string order_id = "";
  // Check if there are any completed orders
  if (!completed_orders_.empty()) {
    auto order = completed_orders_.front();
    // Check if the AGV_{number} is at the destination
    if (order.getOrderTask().getAgvId() == 1) {
      if (agv1_location_ == order.getOrderTask().getDestination()) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Submitting order: " << order.getOrderId());
        order_id = order.getOrderId();
        completed_orders_.erase(completed_orders_.begin());
      }
    } else if (order.getOrderTask().getAgvId() == 2) {
      if (agv2_location_ == order.getOrderTask().getDestination()) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Submitting order: " << order.getOrderId());
        order_id = order.getOrderId();
        completed_orders_.erase(completed_orders_.begin());
      }
    } else if (order.getOrderTask().getAgvId() == 3) {
      if (agv3_location_ == order.getOrderTask().getDestination()) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Submitting order: " << order.getOrderId());
        order_id = order.getOrderId();
        completed_orders_.erase(completed_orders_.begin());
      }
    } else if (order.getOrderTask().getAgvId() == 4) {
      if (agv4_location_ == order.getOrderTask().getDestination()) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Submitting order: " << order.getOrderId());
        order_id = order.getOrderId();
        completed_orders_.erase(completed_orders_.begin());
      }
    }
    // Submit the order
    if (order_id != "") {
      // Create a client to submit the order
      auto client = this->create_client<ariac_msgs::srv::SubmitOrder>(
          "/ariac/submit_order");
      // Wait for the service to be available
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR_STREAM(
              this->get_logger(),
              "Client interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "service not available, waiting again...");
      }
      // Create a request to submit the order
      auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
      request->order_id = order_id;
      auto future_result = client->async_send_request(request);
    }
  }
}
//=========================================
void CompetitorInterface::endCompetition() {
  // Create a client to end the competition
  auto client =
      this->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  // Create a request to end the competition
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = client->async_send_request(request);
}
//=========================================
void CompetitorInterface::moveAGV(int agv_id, int destination) {
  // Create a client to move the AGV
  auto client = this->create_client<ariac_msgs::srv::MoveAGV>(
      "/ariac/move_agv" + std::to_string(agv_id));
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  // Create a request to move the AGV
  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;
  auto future_result = client->async_send_request(
      request, std::bind(&CompetitorInterface::handleAGVStatusResponse, this,
                         std::placeholders::_1));
}
//=========================================
void CompetitorInterface::lockTray(int agv_id) {
  // Create a client to lock the tray
  auto client = this->create_client<std_srvs::srv::Trigger>(
      "/ariac/agv" + std::to_string(agv_id) + "_lock_tray");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  // Create a request to lock the tray
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_result = client->async_send_request(request);
}

//=========================================
void CompetitorInterface::processOrders() {
  // Check for printing the order wise parts
  if (!high_priority_orders_.empty()) {
    printOrderWiseParts(high_priority_orders_.front());
  } else if (!low_priority_orders_.empty()) {
    printOrderWiseParts(low_priority_orders_.front());
  }



  if (active_order_.empty() && high_priority_orders_.empty() &&
      low_priority_orders_.empty() && interrupted_orders_.empty()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "No orders to process...");
  } else if (active_order_.empty() && !interrupted_orders_.empty()) {
    // There was interrupted order
    // Move interrupted order to active order
    active_order_.push_back(interrupted_orders_.front());
    interrupted_orders_.erase(interrupted_orders_.begin());
  } else if (active_order_.empty() && !high_priority_orders_.empty()) {
    // First order high priority
    // Move high priority order to active order
    active_order_.push_back(high_priority_orders_.front());
    high_priority_orders_.erase(high_priority_orders_.begin());

  } else if (active_order_.empty() && high_priority_orders_.empty() &&
             !low_priority_orders_.empty()) {
    // First order low priority
    // Move low priority order to active order
    active_order_.push_back(low_priority_orders_.front());
    low_priority_orders_.erase(low_priority_orders_.begin());
  } else if (!active_order_.empty() && !high_priority_orders_.empty() &&
             active_order_.front().getOrderPriority() == false) {
    // Interrupting a low priority order
    // Move active order to interrupted order
    interrupted_orders_.push_back(active_order_.front());
    active_order_.erase(active_order_.begin());
    // Move high priority order to active order
    active_order_.push_back(high_priority_orders_.front());
    high_priority_orders_.erase(high_priority_orders_.begin());
  }

  // Processing the orders
  if (!active_order_.empty()) {
    // Get the active order
    Order &order = active_order_.front();
    // Update the time passed for the order
    order.setTimePassed(time_interval_ * 0.001 + order.getTimePassed());
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Time passed for order: "
                           << order.getTimePassed()
                           << " Order ID: " << order.getOrderId()
                           << " Order Priority: " << order.getOrderPriority());

    // Check if the order is completed
    if (order.getTimePassed() >= 15) {
      if (order.getOrderPriority() == true) {
        RCLCPP_INFO_STREAM(this->get_logger(), "High Priority Order completed: "
                                                   << order.getOrderId());
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Low Priority Order completed: "
                                                   << order.getOrderId());
      }
      // Complete the order
      order.completeOrder();
      lockTray(order.getOrderTask().getAgvId());
      moveAGV(order.getOrderTask().getAgvId(),
              order.getOrderTask().getDestination());
      completed_orders_.push_back(order);
      active_order_.erase(active_order_.begin());
    }
  }
  // Submit the order
  submitOrder();
  if (active_order_.empty() && completed_orders_.empty() &&
      competition_state_ == 3) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "All orders announced and processed, Proceeding to end competition...");
    endCompetition();
  } else if (active_order_.empty() && completed_orders_.empty() &&
             competition_state_ == 2) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Processing orders...");
  }
}
//=========================================
void CompetitorInterface::kts1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
  if (!received_parts_trays1_) {
    received_parts_trays1_ = true;
    geometry_msgs::msg::Pose sensor_position;
    sensor_position = msg->sensor_pose;
    // std::cout<<"Tray 1\n";
    std::vector<ariac_msgs::msg::KitTrayPose> tray_poses;
    tray_poses = msg->tray_poses;
    for (size_t i = 0; i < tray_poses.size(); i++) {
      geometry_msgs:: msg::Pose world_tray_pose = worldFramePose(tray_poses[i].pose, sensor_position);
      Trays tray_obj(tray_poses[i].id, world_tray_pose);
      // If key doesn't exist in the unordered_map insert it
      if (trays1_.find(tray_obj.getTrayId()) == trays1_.end()) {
        trays1_.insert({tray_obj.getTrayId(), {tray_obj}});
      } else {
        // If key exists in the unordered_map insert it
        trays1_[tray_obj.getTrayId()].push_back(tray_obj);
      }
    }
  }
}
//=========================================
void CompetitorInterface::kts2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
  if (!received_parts_trays2_) {
    received_parts_trays2_ = true;
    geometry_msgs::msg::Pose sensor_position;
    sensor_position = msg->sensor_pose;
    // std::cout<<"Tray 2\n";
    std::vector<ariac_msgs::msg::KitTrayPose> tray_poses;
    tray_poses = msg->tray_poses;
    for (size_t i = 0; i < tray_poses.size(); i++) {
      geometry_msgs:: msg::Pose world_tray_pose = worldFramePose(tray_poses[i].pose, sensor_position);
      Trays tray_obj(tray_poses[i].id, world_tray_pose);
      // If key doesn't exist in the unordered_map insert it
      if (trays2_.find(tray_obj.getTrayId()) == trays2_.end()) {
        trays2_.insert({tray_obj.getTrayId(), {tray_obj}});
      } else {
        // If key exists in the unordered_map insert it
        trays2_[tray_obj.getTrayId()].push_back(tray_obj);
      }
    }
  }
}
//=========================================
void CompetitorInterface::leftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
  if (!received_parts_left_) {
    received_parts_left_ = true;
    geometry_msgs::msg::Pose sensor_position;
    sensor_position = msg->sensor_pose;
    // std::cout<<"Left Bins\n";
    std::vector<ariac_msgs::msg::PartPose> left_part_poses;
    left_part_poses = msg->part_poses;
    for (size_t i = 0; i < left_part_poses.size(); i++) {
      geometry_msgs::msg::Pose world_part_pose = worldFramePose(left_part_poses[i].pose, sensor_position);
      Parts part_obj(left_part_poses[i].part, world_part_pose);
      // If key doesn't exist in the unordered_map insert it
      if (left_bins_.find(std::make_pair(part_obj.getPartType(), part_obj.getPartColor())) == left_bins_.end()) {
        left_bins_.insert({std::make_pair(part_obj.getPartType(), part_obj.getPartColor()), {part_obj}});
      } else {
        // If key exists in the unordered_map insert it
        left_bins_[std::make_pair(part_obj.getPartType(), part_obj.getPartColor())].push_back(part_obj);
      }
    }
  }
}
//=========================================
void CompetitorInterface::rightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
  if (!received_parts_right_) {
    received_parts_right_ = true;
    geometry_msgs::msg::Pose sensor_position;
    sensor_position = msg->sensor_pose;
    // std::cout<<"Right Bins\n";
    std::vector<ariac_msgs::msg::PartPose> right_part_poses;
    right_part_poses = msg->part_poses;
    for (size_t i = 0; i < right_part_poses.size(); i++) {
      geometry_msgs::msg::Pose world_part_pose = worldFramePose(right_part_poses[i].pose, sensor_position);
      Parts part_obj(right_part_poses[i].part, world_part_pose);
      // If key doesn't exist in the unordered_map insert it
      if (right_bins_.find(std::make_pair(part_obj.getPartType(), part_obj.getPartColor())) == right_bins_.end()) {
        right_bins_.insert({std::make_pair(part_obj.getPartType(), part_obj.getPartColor()), {part_obj}});
      } else {
        // If key exists in the unordered_map insert it
        right_bins_[std::make_pair(part_obj.getPartType(), part_obj.getPartColor())].push_back(part_obj);
      }
    }
  }
}
//=========================================
geometry_msgs::msg::Pose CompetitorInterface::worldFramePose(geometry_msgs::msg::Pose part_pose, geometry_msgs::msg::Pose sensor_pose) {
  KDL::Frame part_frame, sensor_frame, world_frame;
  tf2::fromMsg(part_pose, part_frame);
  tf2::fromMsg(sensor_pose, sensor_frame);
  world_frame = sensor_frame * part_frame;
  return tf2::toMsg(world_frame);
}
//=========================================
void CompetitorInterface::printOrderWiseParts(Order order) {
  bool tray_found = false;
  // Print boolean result of trays1_ empty or not
  std::cout << "Trays 1 empty: " << trays1_.empty() << std::endl;
  if (!tray_found) {
    std::cout<<"Tray 1 checking\n";
    std::cout << "Tray 1 count: " << trays1_.count(order.getOrderTask().getTrayId()) << std::endl;
    if (trays1_.count(order.getOrderTask().getTrayId()) > 0) {
      tray_found = true;
      Trays tray_obj = trays1_[order.getOrderTask().getTrayId()].front();
      geometry_msgs::msg::Pose tray_pose = tray_obj.getTrayPose();
      //Convert orientation from quaternion to RPY
      tf2::Quaternion q(
        tray_pose.orientation.x,
        tray_pose.orientation.y,
        tray_pose.orientation.z,
        tray_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO_STREAM(this->get_logger(), "\n=====================================\n"
      << "\nID: " << tray_obj.getTrayId() 
      << "\nPosition (xyz): [" << tray_pose.position.x << ", " << tray_pose.position.y << ", " << tray_pose.position.z << "]"
      << "\nOrientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]"
      << "\n=====================================\n");
      // Erase the tray from the vector
      trays1_[order.getOrderTask().getTrayId()].erase(trays1_[order.getOrderTask().getTrayId()].begin());
      // If the vector is empty erase key from the map
      if (trays1_[order.getOrderTask().getTrayId()].empty()) {
        trays1_.erase(order.getOrderTask().getTrayId());
      }
    }
  }
  // Print boolean result of trays2_ empty or not
  std::cout << "Trays 2 empty: " << trays2_.empty() << std::endl; 
  if (!tray_found) {
    std::cout<<"Tray 2 checking\n";
    std::cout << "Tray 2 count: " << trays2_.count(order.getOrderTask().getTrayId()) << std::endl;
    if (trays2_.find(order.getOrderTask().getTrayId()) != trays2_.end()) {
      tray_found = true;
      Trays tray_obj = trays2_[order.getOrderTask().getTrayId()].front();
      geometry_msgs::msg::Pose tray_pose = tray_obj.getTrayPose();
      //Convert orientation from quaternion to RPY
      tf2::Quaternion q(
        tray_pose.orientation.x,
        tray_pose.orientation.y,
        tray_pose.orientation.z,
        tray_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO_STREAM(this->get_logger(), "\n=====================================\n"
      << "\nID: " << tray_obj.getTrayId() 
      << "\nPosition (xyz): [" << tray_pose.position.x << ", " << tray_pose.position.y << ", " << tray_pose.position.z << "]"
      << "\nOrientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]"
      << "\n=====================================\n");
      // Erase the tray from the vector
      trays2_[order.getOrderTask().getTrayId()].erase(trays2_[order.getOrderTask().getTrayId()].begin());
      // If the vector is empty erase key from the map
      if (trays2_[order.getOrderTask().getTrayId()].empty()) {
        trays2_.erase(order.getOrderTask().getTrayId());
      }
    }
  }
  bool all_parts_found = false;
  std::vector<ariac_msgs::msg::KittingPart> parts = order.getOrderTask().getParts();
  // Print boolean result of left_bins_ empty or not
  std::cout << "Left Bins empty: " << left_bins_.empty() << std::endl;
  if (!left_bins_.empty() && !all_parts_found) {
    std::cout<<"Left Bins checking\n";
    for (size_t i = 0; i < parts.size(); i++) {
      unsigned int part_type = parts[i].part.type;
      unsigned int part_color = parts[i].part.color;
      std::pair<unsigned int, unsigned int> part_key = std::make_pair(part_type, part_color);
      std::cout << "Count left bins part_type: " << left_bins_.count(part_key) << std::endl;
      if (left_bins_.find(part_key) != left_bins_.end()) {
        Parts part_obj = left_bins_[part_key].front();
        geometry_msgs::msg::Pose part_pose = part_obj.getPartPose();
        //Convert orientation from quaternion to RPY
        tf2::Quaternion q(
          part_pose.orientation.x,
          part_pose.orientation.y,
          part_pose.orientation.z,
          part_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO_STREAM(this->get_logger(), "\n=====================================\n" 
        << colorMap[part_obj.getPartColor()] << " " << partMap[part_obj.getPartType()] << "\nPosition (xyz): [" << part_pose.position.x << ", " << part_pose.position.y << ", " << part_pose.position.z << "]" << "\nOrientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]"
        << "\n=====================================\n");
        // Erase the part from the vector
        left_bins_[part_key].erase(left_bins_[part_key].begin());
        // If the vector is empty erase key from the map
        if (left_bins_[part_key].empty()) {
          left_bins_.erase(part_key);
        }
        // Erase the found part from the vector
        std::cout << "Which part is being erased index: " << i << "\n";
        // parts.erase(i);
      }
    }
  }
  // Print boolean result of right_bins_ empty or not
  std::cout << "Right Bins empty: " << right_bins_.empty() << std::endl; 
  if (!right_bins_.empty() && !all_parts_found) {
    std::cout<<"Right Bins checking\n";
    for (size_t i = 0; i < parts.size(); i++) {
      unsigned int part_type = parts[i].part.type;
      unsigned int part_color = parts[i].part.color;
      std::pair<unsigned int, unsigned int> part_key = std::make_pair(part_type, part_color);
      std::cout << "Count right bins part_type in left bins: " << right_bins_.count(part_key) << std::endl;
      
      if (right_bins_.find(part_key) != right_bins_.end()) {
        Parts part_obj = right_bins_[part_key].front();
        geometry_msgs::msg::Pose part_pose = part_obj.getPartPose();
        //Convert orientation from quaternion to RPY
        tf2::Quaternion q(
          part_pose.orientation.x,
          part_pose.orientation.y,
          part_pose.orientation.z,
          part_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO_STREAM(this->get_logger(), "\n=====================================\n" 
        << colorMap[part_obj.getPartColor()] << " " << partMap[part_obj.getPartType()] << "\nPosition (xyz): [" << part_pose.position.x << ", " << part_pose.position.y << ", " << part_pose.position.z << "]" << "\nOrientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]"
        << "\n=====================================\n");
        // Erase the part from the vector
        right_bins_[part_key].erase(right_bins_[part_key].begin());
        // If the vector is empty erase key from the map
        if (right_bins_[part_key].empty()) {
          right_bins_.erase(part_key);
        }
        // Erase the found part from the vector
        std::cout << "Which part is being erased index in right bins: " << i << "\n";
        // parts.erase(i);
      }
    }
  }


  // // Print everything in trays1_
  // for (auto tray : trays1_) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "Tray ID: " << tray.first);
  //   for (auto tray_obj : tray.second) {
  //     geometry_msgs::msg::Pose tray_pose = tray_obj.getTrayPose();
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Position (xyz): [" << tray_pose.position.x << ", " << tray_pose.position.y << ", " << tray_pose.position.z << "]");
  //     //Convert orientation from quaternion to RPY
  //     tf2::Quaternion q(
  //       tray_pose.orientation.x,
  //       tray_pose.orientation.y,
  //       tray_pose.orientation.z,
  //       tray_pose.orientation.w);
  //     tf2::Matrix3x3 m(q);
  //     double roll, pitch, yaw;
  //     m.getRPY(roll, pitch, yaw);
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Orientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]");
  //   }
  // }
  // // Print everything in trays2_
  // for (auto tray : trays2_) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "Tray ID: " << tray.first);
  //   for (auto tray_obj : tray.second) {
  //     geometry_msgs::msg::Pose tray_pose = tray_obj.getTrayPose();
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Position (xyz): [" << tray_pose.position.x << ", " << tray_pose.position.y << ", " << tray_pose.position.z << "]");
  //     //Convert orientation from quaternion to RPY
  //     tf2::Quaternion q(
  //       tray_pose.orientation.x,
  //       tray_pose.orientation.y,
  //       tray_pose.orientation.z,
  //       tray_pose.orientation.w);
  //     tf2::Matrix3x3 m(q);
  //     double roll, pitch, yaw;
  //     m.getRPY(roll, pitch, yaw);
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Orientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]");
  //   }
  // }
  // // Print everything in left_bins_
  // for (auto part : left_bins_) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "Part Type: " << part.first.first << " Part Color: " << part.first.second);
  //   for (auto part_obj : part.second) {
  //     RCLCPP_INFO_STREAM(this->get_logger(), colorMap[part_obj.getPartColor()] << " " << partMap[part_obj.getPartType()]);
  //     geometry_msgs::msg::Pose part_pose = part_obj.getPartPose();
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Position (xyz): [" << part_pose.position.x << ", " << part_pose.position.y << ", " << part_pose.position.z << "]");
  //     //Convert orientation from quaternion to RPY
  //     tf2::Quaternion q(
  //       part_pose.orientation.x,
  //       part_pose.orientation.y,
  //       part_pose.orientation.z,
  //       part_pose.orientation.w);
  //     tf2::Matrix3x3 m(q);
  //     double roll, pitch, yaw;
  //     m.getRPY(roll, pitch, yaw);
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Orientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]");
  //   }
  // }
  // // Print everything in right_bins_
  // for (auto part : right_bins_) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "Part Type: " << part.first.first << " Part Color: " << part.first.second);
  //   for (auto part_obj : part.second) {
  //     RCLCPP_INFO_STREAM(this->get_logger(), colorMap[part_obj.getPartColor()] << " " << partMap[part_obj.getPartType()]);
  //     geometry_msgs::msg::Pose part_pose = part_obj.getPartPose();
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Position (xyz): [" << part_pose.position.x << ", " << part_pose.position.y << ", " << part_pose.position.z << "]");
  //     //Convert orientation from quaternion to RPY
  //     tf2::Quaternion q(
  //       part_pose.orientation.x,
  //       part_pose.orientation.y,
  //       part_pose.orientation.z,
  //       part_pose.orientation.w);
  //     tf2::Matrix3x3 m(q);
  //     double roll, pitch, yaw;
  //     m.getRPY(roll, pitch, yaw);
  //     RCLCPP_INFO_STREAM(this->get_logger(), "Orientation (rpy): [" << roll << ", " << pitch << ", " << yaw << "]");
  //   }
  // }
  // // Print everything in parts
  // std::vector<ariac_msgs::msg::KittingPart> parts = order.getOrderTask().getParts();
  // for (size_t i = 0; i < parts.size(); i++) {
  //   RCLCPP_INFO_STREAM(this->get_logger(), colorMap[parts[i].part.color] << " " << partMap[parts[i].part.type]);
  // }
}

/**
 * @brief Main function to initialize the competitor node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  // Initialize the ROS node
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<CompetitorInterface>("competitor_interface_group3");
  // Create a multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  // Add the node to the executor
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
