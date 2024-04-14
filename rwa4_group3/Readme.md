## Run the following command from the root of your workspace
- Go the workspace root in the terminal
- ```rosdep install --from-paths src -y --ignore-src```
- ```colcon build --packages-select rwa3_group3```
- ```source install/setup.bash```
- Run the command to launch the ARIAC competition```ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=rwa4_group3 sensor_config:=my_sensors trial_name:=rwa4_spring2024```

# Open other terminal and  the following command 
- Go the workspace root
- source ros2 galactic ```source /opt/ros/galactic/setup.bash```
- Wait for the ariac competition to be in the ready state and then launch the following command:
    - ```source install/setup.bash```
    - ```ros2 launch rwa3_group3 compeitition_demo```