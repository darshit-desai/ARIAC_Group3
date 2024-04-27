#include "floor_robot_cpp_only.hpp"
#include <rclcpp/rclcpp.hpp>

// ================================
int
main (int argc, char *argv[])
{
  rclcpp::init (argc, argv);
  auto floor_robot_node = std::make_shared<FloorRobot> ();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node (floor_robot_node);
  try {
    std::thread([&executor]()
                      { executor.spin(); })
              .detach();

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
  // end the competition
  // floor_robot_node->end_competition();
  // try
  //   {
  //     executor.spin ();
  //   }
  // catch (const std::exception &e)
  //   {
  //     std::cerr << e.what () << '\n';
  //     executor.cancel ();
  //     rclcpp::shutdown ();
  //   }
}
// #include "floor_robot_cpp_only.hpp"
// #include <rclcpp/rclcpp.hpp>
// #include <thread>

// // ================================
// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto floor_robot_node = std::make_shared<FloorRobot>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(floor_robot_node);

//     std::thread ([&executor]() {
//         executor.spin();
//     }).detach();

//     // start the competition
//     floor_robot_node->start_competition();
//     // move the robot to home pose
//     floor_robot_node->go_home();

//     // Run complete_orders in a separate thread
//     std::thread complete_orders_thread([&floor_robot_node]() {
//         floor_robot_node->complete_orders();
//     });

//     // Run submit_orders in a separate thread
//     std::thread submit_orders_thread([&floor_robot_node]() {
//         floor_robot_node->submit_order();
//     });

//     // Wait for the threads to finish
//     complete_orders_thread.join();
//     submit_orders_thread.join();

//     // end the competition
//     floor_robot_node->end_competition();

//     // Wait for the spin thread to finish
//     // spin_thread.join();

//     rclcpp::shutdown();
//     return 0;
// }
