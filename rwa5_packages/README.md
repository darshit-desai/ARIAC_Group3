## Install dependencies for YOLO object detection
- Make sure you have the latest version of python3 and python3-pip installed
- Install the YOLO object detection model dependencies on your Ubuntu 20.04 computer ```pip install ultralytics``` or update an existing installation by running ```pip install -U ultralytics```

## Run the following command from the root of your workspace
- Put the trial file inside ```src/ariac/ariac_gazebo/config/trials/``` directory
- Go the workspace root in the terminal
- ```rosdep install --from-paths src -y --ignore-src```
- ```colcon build --packages-select rwa5_group3```
- ```source install/setup.bash```
- Run the command to launch the ARIAC competition```ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=rwa5_group3 sensor_config:=rgb_sensors trial_name:=rwa5_spring2024```

## Open other terminal and  the following command 
- Go the workspace root
- source ros2 galactic ```source /opt/ros/galactic/setup.bash```
- Wait for the ariac competition to be in the ready state and the moveit to be in the ready to accept commands stage and then launch the following command:
    - ```source install/setup.bash```
    - ```ros2 launch rwa5_group3 rwa5_group3.launch.py rviz:=true```, This will launch the competitor package, rviz and the moveit node Note: remove `rviz:=true` if you want to run without rviz.