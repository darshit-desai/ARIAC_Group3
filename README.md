## Install dependencies for YOLO object detection
- Make sure you have the latest version of python3 and python3-pip installed
- Install the YOLO object detection model dependencies on your Ubuntu 20.04 computer ```pip install ultralytics``` or update an existing installation by running ```pip install -U ultralytics```

## Run the following command from the root of your workspace
- Put the trial file inside ```~your_ws/src/ariac/ariac_gazebo/config/trials/``` directory
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

## GPU workload during a complete order run:

- Note: A minimum of 2 GB of GPU VRAM is required to run the complete simulation, Please ensure hardware specifications before jumping to simulation. The below screenshot is from one of the simulation runs show 1.8 GBs of VRAM usage on a RTX4070Ti GPU.

```BASH
Every 0.5s: nvidia-smi                 swarm-Alienware-Aurora-R16: Sat Apr 27 23:49:53 2024

Sat Apr 27 23:49:53 2024
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03             Driver Version: 535.129.03   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce RTX 4070        On  | 00000000:01:00.0  On |                  N/A |
| 31%   37C    P2              39W / 200W |   1643MiB / 12282MiB |     17%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+

+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
+---------------------------------------------------------------------------------------+

```

## Simulation Video:
The following video shows the execution of 3 kitting orders with a high priority order interrupting the low priority order at around 17 seconds (video time)

https://github.com/darshit-desai/ARIAC_Group3/assets/36150235/4231f15f-bb5f-4892-bf71-093cda9d9b6f


