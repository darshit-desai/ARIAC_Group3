/**
 * @file floor_robot_cpp.cpp
 * @author Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu),
 *   Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
 * @brief
 * @version 0.1
 * @date 2024-04-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "floor_robot_cpp.hpp"

#include "utils.hpp"

FloorRobot::FloorRobot()
    : Node("rwa5_group3_cpp"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)),
                   "floor_robot"),
      planning_scene_() {
  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);
  floor_robot_.setPlanningTime(20.0);
  floor_robot_.setNumPlanningAttempts(20);
  floor_robot_.allowReplanning(true);

  // callback groups
  rclcpp::SubscriptionOptions options;
  subscription_cbg_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = subscription_cbg_;
  // subscriber callback to /rwa5_group3/floor_robot/go_home topic
  moveit_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/rwa5_group3/floor_robot/go_home", 10,
      std::bind(&FloorRobot::floor_robot_sub_cb, this, std::placeholders::_1),
      options);

  // subscription  to ariac/orders topic to get orders
  rclcpp::SubscriptionOptions orders_options;
  order_timer_cbg_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  orders_options.callback_group = order_timer_cbg_;
  // subscription to /ariac/orders
  orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
      "/ariac/orders", 1,
      std::bind(&FloorRobot::orders_cb, this, std::placeholders::_1),
      orders_options);
  // client to /ariac/perform_quality_check
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>(
      "/ariac/perform_quality_check", rmw_qos_profile_services_default,
      order_timer_cbg_);

  // subscription to /ariac/competition_state
  rclcpp::SubscriptionOptions competition_state_options;
  competition_timer_cbg_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  competition_state_options.callback_group = competition_timer_cbg_;
  // subscription to /ariac/competition_state
  competition_state_sub_ =
      this->create_subscription<ariac_msgs::msg::CompetitionState>(
          "/ariac/competition_state", 1,
          std::bind(&FloorRobot::competition_state_cb, this,
                    std::placeholders::_1),
          competition_state_options);

  // Submit order timer callback
  submit_order_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FloorRobot::submit_order, this), competition_timer_cbg_);

  // subscription to /ariac/floor_robot_gripper/state
  gripper_cbg_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions gripper_options;
  gripper_options.callback_group = gripper_cbg_;
  floor_gripper_state_sub_ =
      this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
          "/ariac/floor_robot_gripper_state",
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
          std::bind(&FloorRobot::floor_gripper_state_cb, this,
                    std::placeholders::_1),
          gripper_options);

  // client to /ariac/floor_robot_change_gripper
  floor_robot_tool_changer_ =
      this->create_client<ariac_msgs::srv::ChangeGripper>(
          "/ariac/floor_robot_change_gripper");
  // client to /ariac/floor_robot_enable_gripper
  floor_robot_gripper_enable_ =
      this->create_client<ariac_msgs::srv::VacuumGripperControl>(
          "/ariac/floor_robot_enable_gripper");

  rclcpp::SubscriptionOptions agv_status_options;
  agv_timer_cbg_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  agv_status_options.callback_group = agv_timer_cbg_;
  // subscription to /ariac/agv1_status
  agv1_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv1_status", 10,
      std::bind(&FloorRobot::agv1_status_cb, this, std::placeholders::_1),
      agv_status_options);
  // subscription to /ariac/agv2_status
  agv2_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv2_status", 10,
      std::bind(&FloorRobot::agv2_status_cb, this, std::placeholders::_1),
      agv_status_options);
  // subscription to /ariac/agv3_status
  agv3_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv3_status", 10,
      std::bind(&FloorRobot::agv3_status_cb, this, std::placeholders::_1),
      agv_status_options);
  // subscription to /ariac/agv4_status
  agv4_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv4_status", 10,
      std::bind(&FloorRobot::agv4_status_cb, this, std::placeholders::_1),
      agv_status_options);

  // client to rwa5_group3/AdvancedCamera
  advanced_camera_client_ =
      this->create_client<rwa5_group3::srv::AdvancedCamera>("advanced_camera");

  // add models to the planning scene
  add_models_to_planning_scene();

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

//=============================================//
FloorRobot::~FloorRobot() { floor_robot_.~MoveGroupInterface(); }

//=============================================//
bool FloorRobot::start_competition() {
  // Wait for competition state to be ready
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY) {
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();
  RCLCPP_INFO(get_logger(), "Competition started");

  return result.get()->success;
}

//=============================================//
bool FloorRobot::end_competition() {
  while (!completed_orders_.empty()) {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "Waiting for AGVs to reach warehouse and Order submission...");
  }
  if (competition_state_ != ariac_msgs::msg::CompetitionState::ENDED) {
    RCLCPP_INFO_STREAM(get_logger(), "Ending competition...");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

    std::string srv_name = "/ariac/end_competition";

    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = client->async_send_request(request);
    result.wait();
    RCLCPP_INFO_STREAM(get_logger(), "Competition ended");
    return result.get()->success;
  }
  return false;
}

//=============================================//
bool FloorRobot::lock_tray(int agv_num) {
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

//=============================================//
void FloorRobot::move_agv(int agv_num, int destination) {
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

  client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;

  auto result = client->async_send_request(request);
}

//=============================================//
void FloorRobot::agv1_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg) {
  agv_locations_[1] = msg->location;
}

//=============================================//
void FloorRobot::agv2_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg) {
  agv_locations_[2] = msg->location;
}

//=============================================//
void FloorRobot::agv3_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg) {
  agv_locations_[3] = msg->location;
}

//=============================================//
void FloorRobot::agv4_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg) {
  agv_locations_[4] = msg->location;
}

//=============================================//
void FloorRobot::orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg) {
  if (msg->priority == true) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Adding high-priority order to the high priority order vector: "
            << msg->id);
    high_priority_orders_.push_back(*msg);
  } else {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Adding low-priority order to the low priority order vector: "
            << msg->id);
    low_priority_orders_.push_back(*msg);
  }
}

//=============================================//
void FloorRobot::floor_robot_sub_cb(
    const std_msgs::msg::String::ConstSharedPtr msg) {
  if (msg->data == "go_home") {
    if (go_home()) {
      RCLCPP_INFO(get_logger(), "Going home");
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to go home");
    }
  }
}

//=============================================//
void FloorRobot::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) {
  competition_state_ = msg->competition_state;
}

//=============================================//
void FloorRobot::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) {
  floor_gripper_state_ = *msg;
}

geometry_msgs::msg::Pose FloorRobot::get_pose_in_world_frame(
    std::string frame_id) {
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "Could notsetJointValueTarget get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

//=============================================//
void FloorRobot::add_single_model_to_planning_scene(
    std::string name, std::string mesh_file,
    geometry_msgs::msg::Pose model_pose) {
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("rwa5_group3");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_.addCollisionObjects(collision_objects);
}

//=============================================//
void FloorRobot::add_models_to_planning_scene() {
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {
      {"bin1", std::pair<double, double>(-1.9, 3.375)},
      {"bin2", std::pair<double, double>(-1.9, 2.625)},
      {"bin3", std::pair<double, double>(-2.65, 2.625)},
      {"bin4", std::pair<double, double>(-2.65, 3.375)},
      {"bin5", std::pair<double, double>(-1.9, -3.375)},
      {"bin6", std::pair<double, double>(-1.9, -2.625)},
      {"bin7", std::pair<double, double>(-2.65, -2.625)},
      {"bin8", std::pair<double, double>(-2.65, -3.375)}};

  geometry_msgs::msg::Pose bin_pose;
  for (auto const &bin : bin_positions) {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 3.14159);

    add_single_model_to_planning_scene(bin.first, "bin.stl", bin_pose);
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions =
      {
          {"as1", std::pair<double, double>(-7.3, 3)},
          {"as2", std::pair<double, double>(-12.3, 3)},
          {"as3", std::pair<double, double>(-7.3, -3)},
          {"as4", std::pair<double, double>(-12.3, -3)},
      };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const &station : assembly_station_positions) {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation =
        Utils::get_quaternion_from_euler(0, 0, 0);

    add_single_model_to_planning_scene(station.first, "assembly_station.stl",
                                       assembly_station_pose);
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
      {"as1_insert", std::pair<double, double>(-7.7, 3)},
      {"as2_insert", std::pair<double, double>(-12.7, 3)},
      {"as3_insert", std::pair<double, double>(-7.7, -3)},
      {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;
  for (auto const &insert : assembly_insert_positions) {
    assembly_insert_pose.position.x = insert.second.first;
    assembly_insert_pose.position.y = insert.second.second;
    assembly_insert_pose.position.z = 1.011;
    assembly_insert_pose.orientation =
        Utils::get_quaternion_from_euler(0, 0, 0);

    add_single_model_to_planning_scene(insert.first, "assembly_insert.stl",
                                       assembly_insert_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose = geometry_msgs::msg::Pose();
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene("conveyor", "conveyor.stl", conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 3.14159);

  add_single_model_to_planning_scene("kts1_table", "kit_tray_table.stl",
                                     kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene("kts2_table", "kit_tray_table.stl",
                                     kts2_table_pose);
}

//=============================================//
geometry_msgs::msg::Quaternion FloorRobot::set_robot_orientation(
    double rotation) {
  tf2::Quaternion tf_q;
  tf_q.setRPY(0, 3.14159, rotation);

  geometry_msgs::msg::Quaternion q;

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q;
}

//=============================================//
bool FloorRobot::move_to_target() {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(floor_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

//=============================================//
bool FloorRobot::move_through_waypoints(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf) {
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction =
      floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(
      floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_.execute(trajectory));
}

//=============================================//
void FloorRobot::wait_for_attach_completion(double timeout) {
  // Wait for part to be attached
  rclcpp::Time start = now();
  int count = 0;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    
    waypoints.push_back(starting_pose);

    move_through_waypoints(waypoints, 0.1, 0.1);
    
    usleep(200);
    RCLCPP_INFO_STREAM(get_logger(), "Starting pose z: "
                                         << starting_pose.position.z);
    RCLCPP_INFO(get_logger(), "Count: %d", count);
    RCLCPP_INFO(get_logger(), "Part attached: %d", floor_gripper_state_.attached);

    count++;

    if (now() - start > rclcpp::Duration::from_seconds(timeout)) {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

//=============================================//
void FloorRobot::wait_for_attach_completion_pump(double timeout) {
  // Wait for part to be attached
  rclcpp::Time start = now();
  int count = 0;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.0002;
    
    waypoints.push_back(starting_pose);

    move_through_waypoints(waypoints, 0.1, 0.1);
    
    usleep(200);
    RCLCPP_INFO_STREAM(get_logger(), "Starting pose z: "
                                         << starting_pose.position.z);
    RCLCPP_INFO(get_logger(), "Count: %d", count);
    RCLCPP_INFO(get_logger(), "Part attached: %d", floor_gripper_state_.attached);

    count++;

    if (now() - start > rclcpp::Duration::from_seconds(timeout)) {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

//=============================================//
bool FloorRobot::go_home() {
  // Move floor robot to home joint state
  floor_robot_.setNamedTarget("home");
  return move_to_target();
}

//=============================================//
bool FloorRobot::set_gripper_state(bool enable) {
  if (floor_gripper_state_.enabled == enable) {
    if (floor_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else
      RCLCPP_INFO(get_logger(), "Already disabled");

    return false;
  }

  // Call enable service
  auto request =
      std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = floor_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

//=============================================//
bool FloorRobot::change_gripper(std::string changing_station,
                                std::string gripper_type) {
  // Move gripper into tool changer
  auto tc_pose = get_pose_in_world_frame(changing_station + "_tool_changer_" +
                                         gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        set_robot_orientation(0.0)));

  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z,
                                        set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1)) return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

  if (gripper_type == "trays") {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  } else if (gripper_type == "parts") {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto future = floor_robot_tool_changer_->async_send_request(request);

  future.wait();
  if (!future.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1)) return false;

  return true;
}

//=============================================//
bool FloorRobot::pick_and_place_tray(int tray_id, int agv_num) {
  // Check if kit tray is on one of the two tables
  geometry_msgs::msg::Pose tray_pose;
  std::string station;
  bool found_tray = false;

  // Check table 1
  for (auto tray : kts1_trays_) {
    if (tray.id == tray_id) {
      station = "kts1";
      tray_pose = Utils::multiply_poses(kts1_camera_pose_, tray.pose);
      found_tray = true;
      break;
    }
  }
  // Check table 2
  if (!found_tray) {
    for (auto tray : kts2_trays_) {
      if (tray.id == tray_id) {
        station = "kts2";
        tray_pose = Utils::multiply_poses(kts2_camera_pose_, tray.pose);
        found_tray = true;
        break;
      }
    }
  }
  if (!found_tray) return false;

  // get tray rotation
  double tray_rotation = Utils::get_yaw_from_pose(tray_pose);

  // Move floor robot to the corresponding kit tray table
  if (station == "kts1") {
    floor_robot_.setJointValueTarget(floor_kts1_js_);
  } else {
    floor_robot_.setJointValueTarget(floor_kts2_js_);
  }
  move_to_target();

  // Change gripper to tray gripper
  if (floor_gripper_state_.type != "tray_gripper") {
    change_gripper(station, "trays");
  }
  // high priority orders interrupt low priority orders
  if (!high_priority_orders_.empty() && active_orders_.front().priority) {
    return false;
  }

  // Move to tray
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(
      tray_pose.position.x, tray_pose.position.y, tray_pose.position.z + 0.2,
      set_robot_orientation(tray_rotation)));
  waypoints.push_back(Utils::build_pose(tray_pose.position.x,
                                        tray_pose.position.y,
                                        tray_pose.position.z + pick_offset_,
                                        set_robot_orientation(tray_rotation)));
  move_through_waypoints(waypoints, 0.3, 0.3);

  set_gripper_state(true);

  wait_for_attach_completion(3.0);

  // Add kit tray to planning scene
  std::string tray_name = "kit_tray_" + std::to_string(tray_id);
  add_single_model_to_planning_scene(tray_name, "kit_tray.stl", tray_pose);
  floor_robot_.attachObject(tray_name);

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      tray_pose.position.x, tray_pose.position.y, tray_pose.position.z + 0.2,
      set_robot_orientation(tray_rotation)));
  move_through_waypoints(waypoints, 0.3, 0.3);

  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

  move_to_target();

  auto agv_tray_pose =
      get_pose_in_world_frame("agv" + std::to_string(agv_num) + "_tray");
  auto agv_rotation = Utils::get_yaw_from_pose(agv_tray_pose);

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + 0.3, set_robot_orientation(agv_rotation)));

  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_,
      set_robot_orientation(agv_rotation)));

  move_through_waypoints(waypoints, 0.2, 0.1);

  set_gripper_state(false);

  // object is detached in the planning scene
  floor_robot_.detachObject(tray_name);
  planning_scene_.removeCollisionObjects({tray_name});

  // publish to robot state

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.2, 0.1);
  lock_tray(agv_num);
  return true;
}

//=============================================//
bool FloorRobot::pick_bin_part(ariac_msgs::msg::Part part_to_pick) {
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a "
                                       << part_colors_[part_to_pick.color]
                                       << " "
                                       << part_types_[part_to_pick.type]);

  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose;
  bool found_part = false;
  std::string bin_side;

  // Check left bins
  for (auto part : left_bins_parts_) {
    if (part.part.type == part_to_pick.type &&
        part.part.color == part_to_pick.color) {
      part_pose = part.pose;
      found_part = true;
      bin_side = "left_bins";
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "=====================================\n"
                             << "\n"
                             << "\n=====================================\n"
                             << "\n Found part in left bins\n"
                             << "=====================================\n"
                             << "\n"
                             << "=====================================\n"
                             << "\n");
      break;
    }
  }
  // Check right bins
  if (!found_part) {
    for (auto part : right_bins_parts_) {
      if (part.part.type == part_to_pick.type &&
          part.part.color == part_to_pick.color) {
        part_pose = part.pose;
        found_part = true;
        bin_side = "right_bins";
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "=====================================\n"
                               << "\n"
                               << "\n=====================================\n"
                               << "\n Found part in right bins\n"
                               << "=====================================\n"
                               << "\n"
                               << "=====================================\n"
                               << "\n");
        break;
      }
    }
  }
  if (!found_part) {
    RCLCPP_ERROR(get_logger(), "Unable to locate part");
    return false;
  }

  double part_rotation = Utils::get_yaw_from_pose(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper") {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "==============Changing gripper to part gripper==============");
    std::string station;
    if (part_pose.position.y < 0) {
      station = "kts1";
    } else {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1") {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    } else {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    move_to_target();

    change_gripper(station, "parts");
  }
  RCLCPP_INFO_STREAM(get_logger(), "Part pose: " << part_pose.position.x << " "
                                                 << part_pose.position.y << " "
                                                 << part_pose.position.z);

  floor_robot_.setJointValueTarget("linear_actuator_joint",
                                   rail_positions_[bin_side]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y, part_pose.position.z + 0.5,
      set_robot_orientation(part_rotation)));

  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y,
      part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_,
      set_robot_orientation(part_rotation)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  set_gripper_state(true);

  wait_for_attach_completion(3.0);

  // Add part to planning scene
  std::string part_name =
      part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  add_single_model_to_planning_scene(
      part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(
      Utils::build_pose(part_pose.position.x, part_pose.position.y,
                        part_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  return true;
}

//=============================================//
bool FloorRobot::place_part_in_tray(int agv_num, int quadrant) {
  if (!floor_gripper_state_.attached) {
    RCLCPP_ERROR(get_logger(), "No part attached");
    return false;
  }

  // Move to agv
  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();
  if (!floor_gripper_state_.attached) {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        <<"\n\n"
            << "\nPart got dropped while picking from bin attempting to pick up again"
          << "\n\n"  
            << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    // Update the parts vector since it might have dropped the part on the bins
    FloorRobot::update_parts_vector();
    // Update the agv vector since it might have dropped the part on the agv
    FloorRobot::update_agv_vector();
    return false;
  }

  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose =
      get_pose_in_world_frame("agv" + std::to_string(agv_num) + "_tray");
  RCLCPP_INFO_STREAM(get_logger(), "agv_tray_pose: " << agv_tray_pose.position.x
                                                    << " "
                                                    << agv_tray_pose.position.y
                                                    << " "
                                                    << agv_tray_pose.position.z);
  auto part_drop_offset = Utils::build_pose(quad_offsets_[quadrant].first,
                                            quad_offsets_[quadrant].second, 0.0,
                                            geometry_msgs::msg::Quaternion());
  
  auto part_drop_pose = Utils::multiply_poses(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, set_robot_orientation(0)));

  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z +
          part_heights_[floor_robot_attached_part_.type] + drop_height_,
      set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);
  

  // Drop part in quadrant
  set_gripper_state(false);

  std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" +
                          part_types_[floor_robot_attached_part_.type];
  floor_robot_.detachObject(part_name);
  planning_scene_.removeCollisionObjects({part_name});

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.2, 0.1);
  // Update the agv vector since it was successful in placing the part
  FloorRobot::update_agv_vector();
  return true;
}

//=============================================//
//  function to complete orders called from main
// loops till all order are completed and then ends competition
bool FloorRobot::complete_orders() {
  // Wait for first order to be published
  while (high_priority_orders_.size() == 0 &&
         low_priority_orders_.size() == 0) {
  }

  bool success = false;
  while (true) {
    // Check if competition ended
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
      // Add a 1 second delay
      RCLCPP_INFO_STREAM(get_logger(),
                         "Competition ended, all tasks are completed, you can "
                         "now kill this app using Ctrl+C");
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    // all orders vectors are empty wait for orders
    else if (high_priority_orders_.size() == 0 &&
             low_priority_orders_.size() == 0 && active_orders_.size() == 0 &&
             interrupted_orders_.size() == 0 && completed_orders_.size() == 0) {
      if (competition_state_ !=
              ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE &&
          competition_state_ != ariac_msgs::msg::CompetitionState::ENDED) {
        // wait for more orders
        RCLCPP_INFO(get_logger(), "Waiting for orders...");
        while (high_priority_orders_.size() == 0 &&
               low_priority_orders_.size() == 0 && active_orders_.size() == 0 &&
               interrupted_orders_.size() == 0) {
        }
      }
      // all annoucements of orders completed
      else if (competition_state_ == ariac_msgs::msg::CompetitionState::
                                         ORDER_ANNOUNCEMENTS_DONE &&
               competition_state_ != ariac_msgs::msg::CompetitionState::ENDED) {
        RCLCPP_INFO(get_logger(), "Completed all orders");

        if (!success) {
          success = FloorRobot::end_competition();
          success = true;
        }
      }
    }
    // no active orders but ordered in interupted orders
    if (active_orders_.empty() && !interrupted_orders_.empty()) {
      // There was interrupted order
      // Move interrupted order to active order
      active_orders_.push_back(interrupted_orders_.front());
      interrupted_orders_.erase(interrupted_orders_.begin());
    }
    // no active orders but ordered in high priority orders
    else if (active_orders_.empty() && !high_priority_orders_.empty()) {
      // First order high priority
      // Move high priority order to active order
      active_orders_.push_back(high_priority_orders_.front());
      high_priority_orders_.erase(high_priority_orders_.begin());
    }
    // no active and high priority orders but ordered in low priority orders
    else if (active_orders_.empty() && high_priority_orders_.empty() &&
             !low_priority_orders_.empty()) {
      // First order low priority
      // Move low priority order to active order
      active_orders_.push_back(low_priority_orders_.front());
      low_priority_orders_.erase(low_priority_orders_.begin());
    }
    // if active orders is not empty
    if (!active_orders_.empty()) {
      FloorRobot::update_kts_vector();
      FloorRobot::update_parts_vector();
      // if kitting task is completed
      if (FloorRobot::complete_kitting_task(active_orders_.front()) == true) {
        completed_orders_.push_back(active_orders_.front());
        active_orders_.erase(active_orders_.begin());
      }
      // if kitting task is interrupted by high priority order
      else {
        FloorRobot::update_kts_vector();
        FloorRobot::update_parts_vector();
        interrupted_orders_.push_back(active_orders_.front());
        active_orders_.erase(active_orders_.begin());
        active_orders_.push_back(high_priority_orders_.front());
        high_priority_orders_.erase(high_priority_orders_.begin());
        continue;
      }
    }
  }
  return success;
}
void FloorRobot::submit_order() {
  std::string order_id = "";
  if (!completed_orders_.empty()) {
    auto order = completed_orders_.front();
    if (order.kitting_task.agv_number == 1) {
      if (agv_locations_[1] == ariac_msgs::msg::AGVStatus::WAREHOUSE) {
        order_id = order.id;
      }
    } else if (order.kitting_task.agv_number == 2) {
      if (agv_locations_[2] == ariac_msgs::msg::AGVStatus::WAREHOUSE) {
        order_id = order.id;
      }
    } else if (order.kitting_task.agv_number == 3) {
      if (agv_locations_[3] == ariac_msgs::msg::AGVStatus::WAREHOUSE) {
        order_id = order.id;
      }
    } else if (order.kitting_task.agv_number == 4) {
      if (agv_locations_[4] == ariac_msgs::msg::AGVStatus::WAREHOUSE) {
        order_id = order.id;
      }
    }
  }
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
    completed_orders_.erase(completed_orders_.begin());
    order_id = "";
  }
}

void FloorRobot::update_parts_vector() {
  auto part_update_request =
      std::make_shared<rwa5_group3::srv::AdvancedCamera::Request>();
  part_update_request->request_bins = true;
  part_update_request->request_kts = false;
  auto result_part_update =
      advanced_camera_client_->async_send_request(part_update_request);
  result_part_update.wait();
  if (!result_part_update.get()->response_left_bins &&
      !result_part_update.get()->response_right_bins) {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "\n==================================================\n"
            << "\nWoops! PART UPDATE FAILED CHECK CODE"
            << "\n==================================================\n");
  } else {
    left_bins_parts_ = result_part_update.get()->left_bins.part_poses;
    right_bins_parts_ = result_part_update.get()->right_bins.part_poses;
  }
}

void FloorRobot::update_kts_vector() {
  auto kts_update_request =
      std::make_shared<rwa5_group3::srv::AdvancedCamera::Request>();
  kts_update_request->request_bins = false;
  kts_update_request->request_kts = true;
  auto result_kts_update =
      advanced_camera_client_->async_send_request(kts_update_request);
  result_kts_update.wait();
  if (!result_kts_update.get()->response_kts1 &&
      !result_kts_update.get()->response_kts2) {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "\n==================================================\n"
            << "\nWoops! PART UPDATE FAILED CHECK CODE"
            << "\n==================================================\n");
  } else {
    kts1_trays_ = result_kts_update.get()->kts1.tray_poses;
    kts2_trays_ = result_kts_update.get()->kts2.tray_poses;
    kts1_camera_pose_ = result_kts_update.get()->kts1.sensor_pose;
    kts2_camera_pose_ = result_kts_update.get()->kts2.sensor_pose;
  }
}

void FloorRobot::update_agv_vector() {
  auto agv_update_request =
      std::make_shared<rwa5_group3::srv::AdvancedCamera::Request>();
  agv_update_request->request_bins = false;
  agv_update_request->request_kts = false;
  agv_update_request->request_agv1 = true;
  agv_update_request->request_agv2 = true;
  agv_update_request->request_agv3 = true;
  agv_update_request->request_agv4 = true;
  auto result_agv_update =
      advanced_camera_client_->async_send_request(agv_update_request);
  result_agv_update.wait();
  if (!result_agv_update.get()->response_agv1 &&
      !result_agv_update.get()->response_agv2 &&
      !result_agv_update.get()->response_agv3 &&
      !result_agv_update.get()->response_agv4) {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "\n==================================================\n"
            << "\nWoops! PART UPDATE FAILED CHECK CODE"
            << "\n==================================================\n");
  } else {
    agv_change_number_ = 0;
    if(agv1_parts_.size() != result_agv_update.get()->agv1_parts.part_poses.size())
    {
      std::vector<ariac_msgs::msg::PartPose> temp_list = result_agv_update.get()->agv1_parts.part_poses;
      for (size_t i = 0; i < agv1_parts_.size(); i++)
      {
        size_t v_size = temp_list.size();
        for (size_t j = 0; j < v_size; j++)
        {
          if (agv1_parts_[i].part.type == temp_list[j].part.type && agv1_parts_[i].part.color == temp_list[j].part.color)
          {
            temp_list.erase(temp_list.begin() + j);
            break;
          }
        }
      }
      if (temp_list.size() > 0)
      {
        agv1_changed_part_ = temp_list[0];
        agv1_parts_ = result_agv_update.get()->agv1_parts.part_poses;
        agv_change_number_ = 1;
      }
      
    }
    if(agv2_parts_.size() != result_agv_update.get()->agv2_parts.part_poses.size())
    {
      std::vector<ariac_msgs::msg::PartPose> temp_list = result_agv_update.get()->agv2_parts.part_poses;
      for (size_t i = 0; i < agv2_parts_.size(); i++)
      {
        size_t v_size = temp_list.size();
        for (size_t j = 0; j < v_size; j++)
        {
          if (agv2_parts_[i].part.type == temp_list[j].part.type && agv2_parts_[i].part.color == temp_list[j].part.color)
          {
            temp_list.erase(temp_list.begin() + j);
            break;
          }
        }
      }
      if (temp_list.size() > 0)
      {
        agv2_changed_part_ = temp_list[0];
        agv2_parts_ = result_agv_update.get()->agv2_parts.part_poses;
        agv_change_number_ = 2;
      }
    }
    if(agv3_parts_.size() != result_agv_update.get()->agv3_parts.part_poses.size())
    {
      std::vector<ariac_msgs::msg::PartPose> temp_list = result_agv_update.get()->agv3_parts.part_poses;
      for (size_t i = 0; i < agv3_parts_.size(); i++)
      {
        size_t v_size = temp_list.size();
        for (size_t j = 0; j < v_size; j++)
        {
          if (agv3_parts_[i].part.type == temp_list[j].part.type && agv3_parts_[i].part.color == temp_list[j].part.color)
          {
            temp_list.erase(temp_list.begin() + j);
            break;
          }
        }
      }
      if (temp_list.size() > 0)
      {
        agv3_changed_part_ = temp_list[0];
        agv3_parts_ = result_agv_update.get()->agv3_parts.part_poses;
        agv_change_number_ = 3;
      }
    }
    if(agv4_parts_.size() != result_agv_update.get()->agv4_parts.part_poses.size())
    {
      std::vector<ariac_msgs::msg::PartPose> temp_list = result_agv_update.get()->agv4_parts.part_poses;
      for (size_t i = 0; i < agv4_parts_.size(); i++)
      {
        size_t v_size = temp_list.size();
        for (size_t j = 0; j < v_size; j++)
        {
          if (agv4_parts_[i].part.type == temp_list[j].part.type && agv4_parts_[i].part.color == temp_list[j].part.color)
          {
            temp_list.erase(temp_list.begin() + j);
            break;
          }
        }
      }
      if (temp_list.size() > 0)
      {
        agv4_changed_part_ = temp_list[0];
        agv4_parts_ = result_agv_update.get()->agv4_parts.part_poses;
        agv_change_number_ = 4;
      }
    }
    // agv1_parts_ = result_agv_update.get()->agv1_parts.part_poses;
    // agv2_parts_ = result_agv_update.get()->agv2_parts.part_poses;
    // agv3_parts_ = result_agv_update.get()->agv3_parts.part_poses;
    // agv4_parts_ = result_agv_update.get()->agv4_parts.part_poses;
  }
}

//=============================================//
bool FloorRobot::complete_kitting_task(ariac_msgs::msg::Order order) {
  if (order.priority == 0) {
    // go_home();
    if (!high_priority_orders_.empty()) {
      RCLCPP_INFO_STREAM(
          get_logger(),
          "\n==================================================\n"
              << "\nWoops! Order interrupted at go_home"
              << "\n==================================================\n");
      return false;
    }
    if (tray_on_agv_[order.kitting_task.agv_number] !=
        order.kitting_task.tray_id) {
      if (pick_and_place_tray(order.kitting_task.tray_id,
                              order.kitting_task.agv_number) == true) {
        tray_on_agv_[order.kitting_task.agv_number] =
            order.kitting_task.tray_id;
        FloorRobot::update_kts_vector();
      } else {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\n==================================================\n"
                << "\nWoops! Order interrupted at pick_and_place_tray "
                   "after gripper change"
                << "\n==================================================\n");
        return false;
      }
    }
    if (!high_priority_orders_.empty()) {
      RCLCPP_INFO_STREAM(
          get_logger(),
          "\n==================================================\n"
              << "\nWoops! Order interrupted at pick_and_place_tray "
                 "after Putting THE LOW PRIORITY TRAY"
              << "\n==================================================\n");
      return false;
    }

    for (auto kit_part : order.kitting_task.parts) {
      if (parts_in_tray_.find(std::make_tuple(
              order.kitting_task.agv_number, order.kitting_task.tray_id,
              kit_part.quadrant)) != parts_in_tray_.end()) {
        continue;
      }
      FloorRobot::update_agv_vector();
      bool pick_result = pick_bin_part(kit_part.part);
      // Check part dropped after picking the part from the bin
      if (!floor_gripper_state_.attached) {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                << "\nPart got dropped while picking from bin attempting to pick up again"
                << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        // floor_robot_.setJointValueTarget(
        //     "linear_actuator_joint",
        //     rail_positions_["agv" + std::to_string(order.kitting_task.agv_number)]);
        floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
        move_to_target();
        FloorRobot::update_parts_vector();
        std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" +
                                    part_types_[floor_robot_attached_part_.type];
        floor_robot_.detachObject(part_name);
        planning_scene_.removeCollisionObjects({part_name});
        pick_result = pick_bin_part(kit_part.part);
      }
      // Check for part availability in the bins
      if (!pick_result) {
        RCLCPP_INFO_STREAM(
              get_logger(),
              "\n//////////////////////////////////////////////////////////\n"
                  << "\nWoops! Unable to find anymore parts in the bins"
                  << "\n//////////////////////////////////////////////////////////\n");
      }
      else {
        // Check part dropped again after moving to the agv
        if (!place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant)) {
          if (agv_change_number_ != 0 ){
            if (agv_change_number_ == 1) {
              if (FloorRobot::pick_dropped_part_from_agv(1, agv1_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;
              } else {
                pick_result = pick_bin_part(kit_part.part);
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }                                
            } else if (agv_change_number_ == 2) {
              if (FloorRobot::pick_dropped_part_from_agv(2, agv2_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;                
              } else {
                pick_result = pick_bin_part(kit_part.part);
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }
            } else if (agv_change_number_ == 3) {
              if (FloorRobot::pick_dropped_part_from_agv(3, agv3_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;
              } else {
                pick_result = pick_bin_part(kit_part.part);
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }                
            } else if (agv_change_number_ == 4) {
              if (FloorRobot::pick_dropped_part_from_agv(4, agv4_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;
              } else {
                pick_result = pick_bin_part(kit_part.part);
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }
            }
          } 
          // Second possibility it might have fallen in the agv 
          else {
            // Remove part from the gripper and planning scene
            std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" +
                                    part_types_[floor_robot_attached_part_.type];
            floor_robot_.detachObject(part_name);
            planning_scene_.removeCollisionObjects({part_name});
            FloorRobot::update_parts_vector();
            pick_result = pick_bin_part(kit_part.part);
            if (pick_result) {
              place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
              parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                          order.kitting_task.tray_id,
                                          kit_part.quadrant)] = kit_part.part;
            }
          }
          FloorRobot::update_parts_vector();
        }
      }
      FloorRobot::update_parts_vector();
      // Check quality of the part and take action
      if (do_quality_check(order.id) == false) {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\n*****************************************************\n"
            << "\n*****************************************************\n"
                << "\nWoops! Order Quality Check Failed"
                << "\n*****************************************************\n"
                << "\n*****************************************************\n");
        // pick_part_from_tray(order.kitting_task.agv_number, kit_part.quadrant, kit_part.part);
        pick_part_from_tray_using_alc(order.kitting_task.agv_number, kit_part.part);
        move_to_trashbin();
        if (pick_bin_part(kit_part.part) == true) {
          place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
          update_parts_vector();
        } else {
          RCLCPP_INFO_STREAM(
              get_logger(),
              "\n//////////////////////////////////////////////////////////\n"
               << "\n//////////////////////////////////////////////////////////\n" 
                  << "\nWoops! Unable to find anymore parts in the bins"
                  << "\n//////////////////////////////////////////////////////////\n"
                  << "\n//////////////////////////////////////////////////////////\n");
        }
      }
      // Interrupt order if high priority order is received
      if (!high_priority_orders_.empty()) {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\n==================================================\n"
                << "\nWoops! Order interrupted at pick_bin_part"
                << "\n==================================================\n");
        return false;
      }
    }
  }
  // For high priority orders
  else {
    pick_and_place_tray(order.kitting_task.tray_id,
                        order.kitting_task.agv_number);
    FloorRobot::update_kts_vector();
    tray_on_agv_[order.kitting_task.agv_number] = order.kitting_task.tray_id;
    for (auto kit_part : order.kitting_task.parts) {
      // parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
      //                                order.kitting_task.tray_id,
      //                                kit_part.quadrant)] = kit_part.part;
      if (parts_in_tray_.find(std::make_tuple(
              order.kitting_task.agv_number, order.kitting_task.tray_id,
              kit_part.quadrant)) != parts_in_tray_.end()) {
        continue;
      }
      bool pick_result = pick_bin_part(kit_part.part);
      // If the part dropped while picking from the bin
      if (!floor_gripper_state_.attached) {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                << "\nPart got dropped while picking from bin attempting to pick up again"
                << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        // Update the parts vector since it might have dropped the part on the bins
        // floor_robot_.setJointValueTarget(
        //     "linear_actuator_joint",
        //     rail_positions_["agv" + std::to_string(order.kitting_task.agv_number)]);
        floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
        move_to_target();
        FloorRobot::update_parts_vector();
        std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" +
                                    part_types_[floor_robot_attached_part_.type];
        floor_robot_.detachObject(part_name);
        planning_scene_.removeCollisionObjects({part_name});
        pick_result = pick_bin_part(kit_part.part);
      }
      // This condition is to check if the part is available in the bins
      if (!pick_result) {
        RCLCPP_INFO_STREAM(
              get_logger(),
              "\n//////////////////////////////////////////////////////////\n"
                  << "\nWoops! Unable to find anymore parts in the bins"
                  << "\n//////////////////////////////////////////////////////////\n");
      }
      else {
        // Check part dropped again after moving to the agv
        if (!place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant)) {
          // Second possibility it might have fallen in the agv
          if (agv_change_number_ != 0 ){
            if (agv_change_number_ == 1) {
              if (FloorRobot::pick_dropped_part_from_agv(1, agv1_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;
              } else {
                pick_result = pick_bin_part(kit_part.part);
                // Here it found the part in the bins
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }                                
            } else if (agv_change_number_ == 2) {
              if (FloorRobot::pick_dropped_part_from_agv(2, agv2_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;                
              } else {
                pick_result = pick_bin_part(kit_part.part);
                // Here it found the part in the bins
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }
            } else if (agv_change_number_ == 3) {
              if (FloorRobot::pick_dropped_part_from_agv(3, agv3_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;                
              } else {
                pick_result = pick_bin_part(kit_part.part);
                // Here it found the part in the bins
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }
            } else if (agv_change_number_ == 4) {
              if (FloorRobot::pick_dropped_part_from_agv(4, agv4_changed_part_)) {
                place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                            order.kitting_task.tray_id,
                                            kit_part.quadrant)] = kit_part.part;
              } else {
                pick_result = pick_bin_part(kit_part.part);
                // Here it found the part in the bins
                if (pick_result) {
                  place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
                  parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                              order.kitting_task.tray_id,
                                              kit_part.quadrant)] = kit_part.part;
                }
              }
            }
          }
          else {
            std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" +
                                    part_types_[floor_robot_attached_part_.type];
            floor_robot_.detachObject(part_name);
            planning_scene_.removeCollisionObjects({part_name});
            FloorRobot::update_parts_vector();
            // First possibility it might have fallen in bin
            pick_result = pick_bin_part(kit_part.part);
            // Here it found the part in the bins
            if (pick_result) {
              place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
              parts_in_tray_[std::make_tuple(order.kitting_task.agv_number,
                                          order.kitting_task.tray_id,
                                          kit_part.quadrant)] = kit_part.part;
            }
          }
        }
        FloorRobot::update_parts_vector();
      }
      FloorRobot::update_parts_vector();
      if (do_quality_check(order.id) == false) {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\n*****************************************************\n"
            << "\n*****************************************************\n"
                << "\nWoops! Order Quality Check Failed"
                << "\n*****************************************************\n"
                << "\n*****************************************************\n");
        // pick_part_from_tray(order.kitting_task.agv_number, kit_part.quadrant, kit_part.part);
        pick_part_from_tray_using_alc(order.kitting_task.agv_number, kit_part.part);
        move_to_trashbin();
        if (pick_bin_part(kit_part.part) == true) {
          place_part_in_tray(order.kitting_task.agv_number, kit_part.quadrant);
          update_parts_vector();
        } else {
          RCLCPP_INFO_STREAM(
              get_logger(),
              "\n//////////////////////////////////////////////////////////\n"
               << "\n//////////////////////////////////////////////////////////\n" 
                  << "\nWoops! Unable to find anymore parts in the bins"
                  << "\n//////////////////////////////////////////////////////////\n"
                  << "\n//////////////////////////////////////////////////////////\n");
        }
      }      
    }
  }

  

  // move agv to destination
  move_agv(order.kitting_task.agv_number, order.kitting_task.destination);

  return true;
}

bool FloorRobot::do_quality_check(std::string order_id) {
  // Check quality
  auto request =
      std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = order_id;
  auto result = quality_checker_->async_send_request(request);
  result.wait();

  if (!result.get()->all_passed) {
    RCLCPP_WARN_STREAM(get_logger(), "Quality check failed, there might be some issue");
    if (result.get()->quadrant1.faulty_part || result.get()->quadrant2.faulty_part || result.get()->quadrant3.faulty_part || result.get()->quadrant4.faulty_part) {
      RCLCPP_ERROR(get_logger(), "Issue with shipment, faulty parts found");
      return false;  
    }
    
  }
  RCLCPP_INFO_STREAM(get_logger(), "\n\t\t\t\t\t\t\t\t\t\t\n"
                                  << "It seems like the parts are not faulty so passing the quality check"
                                  << "\n\t\t\t\t\t\t\t\t\t\t\n");

  return true;
}
//=============================================//
void FloorRobot::update_specific_agv_vector(int agv_num) {
  auto agv_update_request =
      std::make_shared<rwa5_group3::srv::AdvancedCamera::Request>();
  agv_update_request->request_bins = false;
  agv_update_request->request_kts = false;
  if (agv_num == 1) {
    agv_update_request->request_agv1 = true;
  } else if (agv_num == 2) {
    agv_update_request->request_agv2 = true;
  } else if (agv_num == 3) {
    agv_update_request->request_agv3 = true;
  } else if (agv_num == 4) {
    agv_update_request->request_agv4 = true;
  }
  auto result_agv_update =
      advanced_camera_client_->async_send_request(agv_update_request);
  result_agv_update.wait();
  if (!result_agv_update.get()->response_agv1 &&
      !result_agv_update.get()->response_agv2 &&
      !result_agv_update.get()->response_agv3 &&
      !result_agv_update.get()->response_agv4) {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "\n==================================================\n"
            << "\nWoops! PART UPDATE FAILED CHECK CODE"
            << "\n==================================================\n");
  } else {
    if (agv_num == 1) {
      agv1_forfault_parts_ = result_agv_update.get()->agv1_parts.part_poses;
    } else if (agv_num == 2) {
      agv2_forfault_parts_ = result_agv_update.get()->agv2_parts.part_poses;
    } else if (agv_num == 3) {
      agv3_forfault_parts_ = result_agv_update.get()->agv3_parts.part_poses;
    } else if (agv_num == 4) {
      agv4_forfault_parts_ = result_agv_update.get()->agv4_parts.part_poses;
    }
  }
}

//=============================================//
bool FloorRobot::pick_part_from_tray_using_alc(int agv_num, ariac_msgs::msg::Part part_to_pick) {
  // if (!floor_gripper_state_.attached) {
  //   RCLCPP_ERROR(get_logger(), "No part attached");
  //   return false;
  // }
  // Move to agv
  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();
  // Call the service to find the part pose of the relevant agv
  FloorRobot::update_specific_agv_vector(agv_num);
  std::vector<ariac_msgs::msg::PartPose> agv_parts;
  if (agv_num == 1) {
    agv_parts = agv1_forfault_parts_;
  } else if (agv_num == 2) {
    agv_parts = agv2_forfault_parts_;
  } else if (agv_num == 3) {
    agv_parts = agv3_forfault_parts_;
  } else if (agv_num == 4) {
    agv_parts = agv4_forfault_parts_;
  }
  // Extract the part pose of the part to be picked
  geometry_msgs::msg::Pose part_pose;
  for (auto part : agv_parts) {
    if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color) {
      part_pose = part.pose;
      break;
    }
  }
  double part_rotation = Utils::get_yaw_from_pose(part_pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y, part_pose.position.z + 0.5,
      set_robot_orientation(part_rotation)));
  
  RCLCPP_INFO_STREAM(get_logger(), "part_drop_pose: " << part_pose.position.x
                                                    << " "
                                                    << part_pose.position.y
                                                    << " "
                                                    << part_pose.position.z
                                                    << " "
                                                    << part_to_pick.type
                                                    << " "
                                                    << part_pose.position.z +
          part_heights_[part_to_pick.type] + pick_offset_);
  
  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y,
      part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_,
      set_robot_orientation(part_rotation)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  set_gripper_state(true);

  wait_for_attach_completion(3.0);
  if (!floor_gripper_state_.attached) {
    RCLCPP_ERROR(get_logger(), "No part attached");
    return false;
  }
  // Add part to planning scene
  std::string part_name =
      part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  add_single_model_to_planning_scene(
      part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(
      Utils::build_pose(part_pose.position.x, part_pose.position.y,
                        part_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);
  return true;  
}


//=============================================//
bool FloorRobot::pick_part_from_tray(int agv_num, int quadrant, ariac_msgs::msg::Part part_to_pick) {
  // if (!floor_gripper_state_.attached) {
  //   RCLCPP_ERROR(get_logger(), "No part attached");
  //   return false;
  // }
  // Move to agv
  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();

  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose =
      get_pose_in_world_frame("agv" + std::to_string(agv_num) + "_tray");
  RCLCPP_INFO_STREAM(get_logger(), "agv_tray_pose: " << agv_tray_pose.position.x
                                                    << " "
                                                    << agv_tray_pose.position.y
                                                    << " "
                                                    << agv_tray_pose.position.z);
  auto part_drop_offset = Utils::build_pose(quad_offsets_[quadrant].first,
                                            quad_offsets_[quadrant].second, 0.0,
                                            geometry_msgs::msg::Quaternion());

  auto part_drop_pose = Utils::multiply_poses(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  // waypoints.push_back(Utils::build_pose(
  //     part_drop_pose.position.x, part_drop_pose.position.y,
  //     part_drop_pose.position.z + 0.3, set_robot_orientation(0)));

  // waypoints.push_back(Utils::build_pose(
  //     part_drop_pose.position.x, part_drop_pose.position.y,
  //     part_drop_pose.position.z +
  //         part_heights_[part_to_pick.type] + pick_offset_,
  //     set_robot_orientation(0)));
  RCLCPP_INFO_STREAM(get_logger(), "part_drop_pose: " << part_drop_pose.position.x
                                                    << " "
                                                    << part_drop_pose.position.y
                                                    << " "
                                                    << part_drop_pose.position.z
                                                    << " "
                                                    << part_to_pick.type
                                                    << " "
                                                    << part_drop_pose.position.z +
          part_heights_[part_to_pick.type] + 0.01);
  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z +
          part_heights_[part_to_pick.type] + 0.03,
      set_robot_orientation(0)));
  move_through_waypoints(waypoints, 0.3, 0.3);
  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z +
          part_heights_[part_to_pick.type] + 0.01,
      set_robot_orientation(0)));
  move_through_waypoints(waypoints, 0.3, 0.3);

  

  // Drop part in quadrant
  set_gripper_state(true);
  if (part_to_pick.type == ariac_msgs::msg::Part::PUMP) {
    wait_for_attach_completion_pump(60.0);
  } else {
    wait_for_attach_completion(10.0);
  }
  // Add part to planning scene
  std::string part_name =
      part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  add_single_model_to_planning_scene(
      part_name, part_types_[part_to_pick.type] + ".stl", part_drop_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;
  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);
  return true;
}

//=============================================//
bool FloorRobot::move_to_trashbin() {
  std::string trash_bin = "trash_bin";
  floor_robot_.setJointValueTarget("linear_actuator_joint",
                                   rail_positions_[trash_bin]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);                                   
  move_to_target();
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(
      -2.2, 0.0,
      0.76 + 0.3, set_robot_orientation(0)));

  waypoints.push_back(Utils::build_pose(
      -2.2, 0.0,
      0.76,
      set_robot_orientation(0)));
  move_through_waypoints(waypoints, 0.3, 0.3);
  set_gripper_state(false);
  std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" +
                          part_types_[floor_robot_attached_part_.type];
  floor_robot_.detachObject(part_name);
  planning_scene_.removeCollisionObjects({part_name});
  
  floor_robot_.setJointValueTarget("linear_actuator_joint",
                                   rail_positions_[trash_bin]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);                                   
  return move_to_target();
}

//=============================================//
bool FloorRobot::pick_dropped_part_from_agv(int agv_num, ariac_msgs::msg::PartPose partpose) {
  ariac_msgs::msg::Part part_to_pick = partpose.part;
  geometry_msgs::msg::Pose part_pose = partpose.pose;
  // Move to agv
  std::string part_name =
      part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  floor_robot_.detachObject(part_name);
  planning_scene_.removeCollisionObjects({part_name});
  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();
  
  double part_rotation = Utils::get_yaw_from_pose(part_pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y, part_pose.position.z + 0.5,
      set_robot_orientation(part_rotation)));
  
  RCLCPP_INFO_STREAM(get_logger(), "part_drop_pose: " << part_pose.position.x
                                                    << " "
                                                    << part_pose.position.y
                                                    << " "
                                                    << part_pose.position.z
                                                    << " "
                                                    << part_to_pick.type
                                                    << " "
                                                    << part_pose.position.z +
          part_heights_[part_to_pick.type] + pick_offset_);
  
  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y,
      part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_,
      set_robot_orientation(part_rotation)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  set_gripper_state(true);

  wait_for_attach_completion(3.0);
  if (!floor_gripper_state_.attached) {
    RCLCPP_ERROR(get_logger(), "No part attached");
    return false;
  }
  // Add part to planning scene
  // std::string part_name =
  //     part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  add_single_model_to_planning_scene(
      part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(
      Utils::build_pose(part_pose.position.x, part_pose.position.y,
                        part_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);
  return true;
}
