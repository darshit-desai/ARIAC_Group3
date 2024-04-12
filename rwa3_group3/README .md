## Run the following command from the root of your workspace
- Go the workspace root in the terminal
- ```rosdep install --from-paths src -y --ignore-src```
- ```colcon build --packages-select rwa3_group3```
- ```source install/setup.bash```


# Open other terminal and  the following command 
- Go the workspace root
- source ros2 galactic ```source /opt/ros/galactic/setup.bash```
- ```source install/setup.bash```
- ```ros2 launch rwa3_group3 compeitition_demo```

