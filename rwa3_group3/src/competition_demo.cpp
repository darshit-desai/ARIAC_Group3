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
#include "rwa3_group3/competitor_interface.hpp"

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
  RCLCPP_INFO_STREAM(this->get_logger(), "Received order:\n" << msg->id);
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
