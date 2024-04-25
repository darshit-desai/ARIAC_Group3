



ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=rwa5_group3 sensor_config:=rgb_sensors trial_name:=rwa5_spring2024
ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
ros2 launch rwa5_group3 demo_cpp.launch.py rviz:=true