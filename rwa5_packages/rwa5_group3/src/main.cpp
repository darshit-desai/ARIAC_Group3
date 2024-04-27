/**
 * @file main.cpp
 * @author Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu),
 *   Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
 * @brief
 * @version 0.1
 * @date 2024-04-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <rclcpp/rclcpp.hpp>

#include "floor_robot_cpp.hpp"

// ================================
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto floor_robot_node = std::make_shared<FloorRobot>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(floor_robot_node);
  try {
    std::thread([&executor]() { executor.spin(); }).detach();

    // start the competition
    floor_robot_node->start_competition();
    // move the robot to home pose
    floor_robot_node->go_home();
    // complete orders
    floor_robot_node->complete_orders();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    executor.cancel();
    rclcpp::shutdown();
  }
}
