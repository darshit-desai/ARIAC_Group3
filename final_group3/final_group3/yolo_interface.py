"""
File: yolo_interface.py
Author: Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu), Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
Date: 2024-04-27
Description: Python module containing class implementation for doing object detection of kit trays and bin parts
"""

from ultralytics import YOLO
from ultralytics.engine.results import Results
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from ariac_msgs.msg import BasicLogicalCameraImage
from ariac_msgs.msg import AdvancedLogicalCameraImage, PartPose, Part, KitTrayPose
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import rclpy
import PyKDL
import numpy as np
from torch import cuda
from cv_bridge import CvBridge
import cv2
from matplotlib import pyplot as plt
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from final_group3.srv import AdvancedCamera

"""Class for parsing RGB camera & logical camera inputs and give out poses"""


class YOLOInterface(Node):
    """
    Class for a robot commander node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    """

    def __init__(self, best_pt_file):
        """Constructor of the YOLO interface class

        Args:
            best_pt_file (string): Object detection Model path
        """
        super().__init__("YOLO_interface")
        self.cv_bridge = CvBridge()
        self.device = "cuda"
        print(best_pt_file)
        self.model = YOLO(best_pt_file)
        self.threshold = 0.5
        # Labels of parts
        self.names = [
            "blue_battery",
            "blue_pump",
            "blue_regulator",
            "blue_sensor",
            "green_battery",
            "green_pump",
            "green_regulator",
            "green_sensor",
            "orange_battery",
            "orange_pump",
            "orange_regulator",
            "orange_sensor",
            "purple_battery",
            "purple_pump",
            "purple_regulator",
            "purple_sensor",
            "red_battery",
            "red_pump",
            "red_regulator",
            "red_sensor",
        ]
        # Dictionaries containing color and part indices
        self.COLORS = {"red": 0, "green": 1, "blue": 2, "orange": 3, "purple": 4}
        self.EQUIPMENT = {"battery": 10, "pump": 11, "sensor": 12, "regulator": 13}
        # Dictionary for splitting labels
        self.divided_names = {
            idx: [
                self.COLORS[name.split("_")[0]],
                self.EQUIPMENT[name.split("_")[1]],
                name,
            ]
            for idx, name in enumerate(self.names)
        }
        # Dictionary for converting label integers to strings
        self.names_dict = {index: name for index, name in enumerate(self.names)}

        # Service server for publishing service requests of object detection
        self.advanced_camera_srv = self.create_service(
            AdvancedCamera, "advanced_camera", self.handle_request
        )

        # Subscribers for rgb cameras with callback groups
        group_bins = ReentrantCallbackGroup()
        # 8 subscribers for rgb cameras
        for i in range(1, 9):
            self.create_subscription(
                Image,
                f"/ariac/sensors/bins_camera_rgb_bin{i}/rgb_image",
                getattr(self, f"_rgb_camera_{i}_cb"),
                10,
                callback_group=group_bins,
            )
        # 8 subscribers for basic logical cameras
        for i in range(1, 9):
            self.create_subscription(
                BasicLogicalCameraImage,
                f"/ariac/sensors/bins_logical_camera{i}/image",
                getattr(self, f"_logical_camera_{i}_cb"),
                qos_profile_sensor_data,
                callback_group=group_bins,
            )
        # 2 subscribers for advanced logical cameras
        for i in range(1, 3):
            setattr(
                self,
                f"advanced_pub{i}",
                self.create_publisher(
                    AdvancedLogicalCameraImage,
                    f"/ariac/sensors/advanced_bins_logical_camera{i}/image",
                    10,  # qos profile
                ),
            )
        # 2 subscriber rgb camera for kit tray stations
        group_kts = MutuallyExclusiveCallbackGroup()
        for i in range(1, 3):
            self.create_subscription(
                Image,
                f"/ariac/sensors/kts{i}_rgb_camera/rgb_image",
                getattr(self, f"_kts_rgb_camera_{i}_cb"),
                10,
                callback_group=group_kts,
            )
        # 2 subscribers for basic logical cameras for kit tray stations
        for i in range(1, 3):
            self.create_subscription(
                BasicLogicalCameraImage,
                f"/ariac/sensors/kts{i}_logical_camera/image",
                getattr(self, f"_kts_logical_camera_{i}_cb"),
                qos_profile_sensor_data,
                callback_group=group_kts,
            )
        group_agvs = MutuallyExclusiveCallbackGroup()
        # 4 subscribers for advanced logical cameras for agvs
        for i in range(1, 5):
            self.create_subscription(
                AdvancedLogicalCameraImage,
                f"/ariac/sensors/agv{i}_advanced_logical_camera/image",
                getattr(self, f"_agv{i}_advanced_cb"),
                qos_profile_sensor_data,
                callback_group=group_agvs,
            )

        # Aruco marker object definitions
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.tag_detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        self.camera_subscriber = self.create_subscription(
            CameraInfo,
            "/ariac/sensors/kts1_rgb_camera/camera_info",
            self._camera_info_cb,
            10,
        )
        self.calib_matrix = None

        # Sensor pose initialization
        self.kts_sensor_pose1 = None
        self.kts_sensor_pose2 = None

        # RGB camera frame initialization
        self.current_frame1 = None
        self.current_frame2 = None
        self.current_frame3 = None
        self.current_frame4 = None
        self.current_frame5 = None
        self.current_frame6 = None
        self.current_frame7 = None
        self.current_frame8 = None

        # Basic logical camera pose array initialization
        self.sorted_pose_array1 = None
        self.sorted_pose_array2 = None
        self.sorted_pose_array3 = None
        self.sorted_pose_array4 = None
        self.sorted_pose_array5 = None
        self.sorted_pose_array6 = None
        self.sorted_pose_array7 = None
        self.sorted_pose_array8 = None

        for i in range(1, 9):
            setattr(self, f"sensor_pose{i}", None)
        for i in range(1, 5):
            setattr(self, f"agv{i}_part_poses", None)
        for i in range(1, 5):
            setattr(self, f"agv{i}_sensor_pose", None)

        self.kittraystation1_slots = {1: None, 2: None, 3: None}
        self.kittraystation2_slots = {4: None, 5: None, 6: None}
        self.sorted_ktspose_array1 = None
        self.sorted_ktspose_array2 = None

        # dictionary to map part type and color to the bin number
        self.slot_centroids_bin1 = {i: None for i in range(1, 10)}
        self.slot_centroids_bin2 = {i: None for i in range(1, 10)}
        self.slot_centroids_bin3 = {i: None for i in range(1, 10)}
        self.slot_centroids_bin4 = {i: None for i in range(1, 10)}
        self.slot_centroids_bin5 = {i: None for i in range(1, 10)}
        self.slot_centroids_bin6 = {i: None for i in range(1, 10)}
        self.slot_centroids_bin7 = {i: None for i in range(1, 10)}
        self.slot_centroids_bin8 = {i: None for i in range(1, 10)}

        # dictionary to map part pose to bin number
        self.slot_pose_bin1 = {i: None for i in range(1, 10)}
        self.slot_pose_bin2 = {i: None for i in range(1, 10)}
        self.slot_pose_bin3 = {i: None for i in range(1, 10)}
        self.slot_pose_bin4 = {i: None for i in range(1, 10)}
        self.slot_pose_bin5 = {i: None for i in range(1, 10)}
        self.slot_pose_bin6 = {i: None for i in range(1, 10)}
        self.slot_pose_bin7 = {i: None for i in range(1, 10)}
        self.slot_pose_bin8 = {i: None for i in range(1, 10)}

    # calback for camera calibration matrix
    def _camera_info_cb(self, msg):
        self.calib_matrix = msg.k

    # Yolo model prediction method
    def _model_prediction(self, img):
        """YOLO Model prediction method

        Args:
            img (cv2 image): RGB image stored by the subscriber callbacks

        Returns:
            tensor: Returns a tensor of tensors containing information
            about detected objects type and position in 2d.
        """
        results = self.model.predict(
            source=img,
            verbose=False,
            stream=False,
            conf=self.threshold,
            device=self.device,
        )
        return results[0].boxes.data

    def _rgb_camera_1_cb(self, msg):
        """Subscriber callback for RGB camera 1

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame1 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    def _rgb_camera_2_cb(self, msg):
        """Subscriber callback for RGB camera 2

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame2 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    def _rgb_camera_3_cb(self, msg):
        """Subscriber callback for RGB camera 3

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame3 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    def _rgb_camera_4_cb(self, msg):
        """Subscriber callback for RGB camera 4

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame4 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    def _rgb_camera_5_cb(self, msg):
        """Subscriber callback for RGB camera 5

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame5 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    def _rgb_camera_6_cb(self, msg):
        """Subscriber callback for RGB camera 6

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame6 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    def _rgb_camera_7_cb(self, msg):
        """Subscriber callback for RGB camera 7

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame7 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    # callback for rgb camera bin 8
    def _rgb_camera_8_cb(self, msg):
        """Subscriber callback for RGB camera 8

        Args:
            msg (sensor_msgs.msg.Image): Image message published by cameras
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame8 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

    def find_category_bin3258(self, centroid_x, centroid_y, bbox5, bin_cmd):
        """Localizing parts in slots according to pixel coordinates

        Args:
            centroid_x (int): centroid x of the bbox
            centroid_y (int): centroid y of the bbox
            bbox5 (int): category integer of the part
            bin_cmd (int): bin number in which the part is present
        """
        slot_centroids_bin = getattr(self, f"slot_centroids_bin{bin_cmd}")
        # contions for slot 1,2,3
        if centroid_y > 12 and centroid_y < 154:
            if centroid_x < 228 and slot_centroids_bin[1] is None:
                slot_centroids_bin[1] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[2] is None
            ):
                slot_centroids_bin[2] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[3] is None
            ):
                slot_centroids_bin[3] = self.divided_names[bbox5.item()]
        # conditions for slot 4,5,6
        if centroid_y > 154 and centroid_y < 295:
            if centroid_x < 228 and slot_centroids_bin[4] is None:
                slot_centroids_bin[4] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[5] is None
            ):
                slot_centroids_bin[5] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[6] is None
            ):
                slot_centroids_bin[6] = self.divided_names[bbox5.item()]
        # conditions for slot 7,8,9
        if centroid_y > 295 and centroid_y < 435:
            if centroid_x < 228 and slot_centroids_bin[7] is None:
                slot_centroids_bin[7] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[8] is None
            ):
                slot_centroids_bin[8] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[9] is None
            ):
                slot_centroids_bin[9] = self.divided_names[bbox5.item()]

        # Update the original dictionaries based on the modified
        # slot_centroids_bin
        if bin_cmd == 2:
            self.slot_centroids_bin2 = slot_centroids_bin
        elif bin_cmd == 3:
            self.slot_centroids_bin3 = slot_centroids_bin
        elif bin_cmd == 5:
            self.slot_centroids_bin5 = slot_centroids_bin
        elif bin_cmd == 8:
            self.slot_centroids_bin8 = slot_centroids_bin

    def find_category_bin4167(self, centroid_x, centroid_y, bbox5, bin_cmd):
        """Localizing parts in slots according to pixel coordinates

        Args:
            centroid_x (int): centroid x of the bbox
            centroid_y (int): centroid y of the bbox
            bbox5 (int): category integer of the part
            bin_cmd (int): bin number in which the part is present
        """
        slot_centroids_bin = getattr(self, f"slot_centroids_bin{bin_cmd}")
        # contions for slot 1,2,3
        if centroid_y > 12 and centroid_y < 154:
            if centroid_x < 228 and slot_centroids_bin[1] is None:
                slot_centroids_bin[1] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[2] is None
            ):
                slot_centroids_bin[2] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[3] is None
            ):
                slot_centroids_bin[3] = self.divided_names[bbox5.item()]
        # conditions for slot 4,5,6
        if centroid_y > 154 and centroid_y < 295:
            if centroid_x < 228 and slot_centroids_bin[4] is None:
                slot_centroids_bin[4] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[5] is None
            ):
                slot_centroids_bin[5] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[6] is None
            ):
                slot_centroids_bin[6] = self.divided_names[bbox5.item()]
        # conditions for slot 7,8,9
        if centroid_y > 295 and centroid_y < 435:
            if centroid_x < 228 and slot_centroids_bin[7] is None:
                slot_centroids_bin[7] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[8] is None
            ):
                slot_centroids_bin[8] = self.divided_names[bbox5.item()]
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[9] is None
            ):
                slot_centroids_bin[9] = self.divided_names[bbox5.item()]

        if bin_cmd == 1:
            self.slot_centroids_bin1 = slot_centroids_bin
        elif bin_cmd == 4:
            self.slot_centroids_bin4 = slot_centroids_bin
        elif bin_cmd == 6:
            self.slot_centroids_bin6 = slot_centroids_bin
        elif bin_cmd == 7:
            self.slot_centroids_bin7 = slot_centroids_bin

    def find_pose_bin3258(self, centroid_x, centroid_y, bin_cmd, pose_array):
        """Localizing parts in slots according to pixel coordinates projected
        from 3d to 2d

        Args:
            centroid_x (int): centroid x of the bbox
            centroid_y (int): centroid y of the bbox
            bin_cmd (int): bin number in which the part is present
            pose_array (nd.array): pose array containing the pose
        """
        slot_centroids_bin = getattr(self, f"slot_pose_bin{bin_cmd}")
        # contions for slot 1,2,3
        if centroid_y > 12 and centroid_y < 154:
            if centroid_x < 228 and slot_centroids_bin[1] is None:
                slot_centroids_bin[1] = pose_array
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[2] is None
            ):
                slot_centroids_bin[2] = pose_array
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[3] is None
            ):
                slot_centroids_bin[3] = pose_array
        # conditions for slot 4,5,6
        if centroid_y > 154 and centroid_y < 295:
            if centroid_x < 228 and slot_centroids_bin[4] is None:
                slot_centroids_bin[4] = pose_array
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[5] is None
            ):
                slot_centroids_bin[5] = pose_array
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[6] is None
            ):
                slot_centroids_bin[6] = pose_array
        # conditions for slot 7,8,9
        if centroid_y > 295 and centroid_y < 435:
            if centroid_x < 228 and slot_centroids_bin[7] is None:
                slot_centroids_bin[7] = pose_array
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[8] is None
            ):
                slot_centroids_bin[8] = pose_array
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[9] is None
            ):
                slot_centroids_bin[9] = pose_array

        # Update the original dictionaries based on the modified
        # slot_centroids_bin
        if bin_cmd == 2:
            self.slot_pose_bin2 = slot_centroids_bin
        elif bin_cmd == 3:
            self.slot_pose_bin3 = slot_centroids_bin
        elif bin_cmd == 5:
            self.slot_pose_bin5 = slot_centroids_bin
        elif bin_cmd == 8:
            self.slot_pose_bin8 = slot_centroids_bin

    def find_pose_bin4167(self, centroid_x, centroid_y, bin_cmd, pose_array):
        """Localizing parts in slots according to pixel coordinates projected
        from 3d to 2d

        Args:
            centroid_x (int): centroid x of the bbox
            centroid_y (int): centroid y of the bbox
            bin_cmd (int): bin number in which the part is present
            pose_array (nd.array): pose array containing the pose
        """
        slot_centroids_bin = getattr(self, f"slot_pose_bin{bin_cmd}")
        # contions for slot 1,2,3
        if centroid_y > 12 and centroid_y < 154:
            if centroid_x < 228 and slot_centroids_bin[1] is None:
                slot_centroids_bin[1] = pose_array
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[2] is None
            ):
                slot_centroids_bin[2] = pose_array
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[3] is None
            ):
                slot_centroids_bin[3] = pose_array
        # conditions for slot 4,5,6
        if centroid_y > 154 and centroid_y < 295:
            if centroid_x < 228 and slot_centroids_bin[4] is None:
                slot_centroids_bin[4] = pose_array
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[5] is None
            ):
                slot_centroids_bin[5] = pose_array
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[6] is None
            ):
                slot_centroids_bin[6] = pose_array
        # conditions for slot 7,8,9
        if centroid_y > 295 and centroid_y < 435:
            if centroid_x < 228 and slot_centroids_bin[7] is None:
                slot_centroids_bin[7] = pose_array
            elif (
                centroid_x > 228 and centroid_x < 370 and slot_centroids_bin[8] is None
            ):
                slot_centroids_bin[8] = pose_array
            elif (
                centroid_x > 370 and centroid_x < 500 and slot_centroids_bin[9] is None
            ):
                slot_centroids_bin[9] = pose_array

        if bin_cmd == 1:
            self.slot_pose_bin1 = slot_centroids_bin
        elif bin_cmd == 4:
            self.slot_pose_bin4 = slot_centroids_bin
        elif bin_cmd == 6:
            self.slot_pose_bin6 = slot_centroids_bin
        elif bin_cmd == 7:
            self.slot_pose_bin7 = slot_centroids_bin

    def _logical_camera_1_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose1 = msg.sensor_pose
        # Sorting the pose array based on y and z coordinates
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array1 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array1 = None

    def _logical_camera_2_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose2 = msg.sensor_pose
        # Sorting the pose array based on y and z
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array2 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array2 = None

    def _logical_camera_3_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose3 = msg.sensor_pose
        # Sorting the pose array based on y and z
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array3 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array3 = None

    def _logical_camera_4_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose4 = msg.sensor_pose
        # Sorting the pose array based on y and z
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array4 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array4 = None

    def _logical_camera_5_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose5 = msg.sensor_pose
        #   Sorting the pose array based on y and z
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array5 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array5 = None

    def _logical_camera_6_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose6 = msg.sensor_pose
        # Sorting the pose array based on y and z
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array6 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array6 = None

    def _logical_camera_7_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose7 = msg.sensor_pose
        #   Sorting the pose array based on y and z
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array7 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array7 = None

    def _logical_camera_8_cb(self, msg):
        """Subscriber callback for logical camera above bins

        Args:
            msg (list of geometry_msgs): Pose message of the parts
        """
        pose_list = []
        for pose in msg.part_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.sensor_pose8 = msg.sensor_pose
        # Sorting the pose array based on y and z
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_pose_array8 = pose_array[
                np.lexsort((-pose_array[:, 1], -pose_array[:, 2]))
            ]
        else:
            self.sorted_pose_array8 = None

    def compute_part_pose_in_world(
        self, part_pose_in_camera_frame, camera_pose_in_world_frame
    ):
        """Computing part frame in the world frame using kdl

        Args:
            part_pose_in_camera_frame (geometry_msgs): part pose in camera frame
            camera_pose_in_world_frame (geometry_msgs): camera pose in world frame

        Returns:
            geometry_msgs: Part pose in world frame
        """
        # First frame
        camera_orientation = camera_pose_in_world_frame.orientation
        camera_x = camera_pose_in_world_frame.position.x
        camera_y = camera_pose_in_world_frame.position.y
        camera_z = camera_pose_in_world_frame.position.z

        frame_camera_world = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ),
            PyKDL.Vector(camera_x, camera_y, camera_z),
        )

        # Second frame
        part_orientation = part_pose_in_camera_frame.orientation
        part_x = part_pose_in_camera_frame.position.x
        part_y = part_pose_in_camera_frame.position.y
        part_z = part_pose_in_camera_frame.position.z

        frame_part_camera = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                part_orientation.x,
                part_orientation.y,
                part_orientation.z,
                part_orientation.w,
            ),
            PyKDL.Vector(part_x, part_y, part_z),
        )

        # Multiply the two frames
        frame_part_world = frame_camera_world * frame_part_camera

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame_part_world.p.x()
        pose.position.y = frame_part_world.p.y()
        pose.position.z = frame_part_world.p.z()

        # convert the rotation matrix to quaternion and return
        q = frame_part_world.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def plot_from_3d_2d(self, arr):
        """
        Function to convert 3D points to 2D pixel coordinates
        Args: arr (np.array): 3D points
        Returns: np.array: 2D pixel coordinates
        """
        f_x = self.calib_matrix[0]
        c_x = self.calib_matrix[2]
        f_y = self.calib_matrix[4]
        c_y = self.calib_matrix[5]
        X_pixel = c_x - (f_x * arr[:, 1] / arr[:, 0])
        Y_pixel = c_y - (f_y * arr[:, 2] / arr[:, 0])
        return np.concatenate((X_pixel[:, np.newaxis], Y_pixel[:, np.newaxis]), axis=1)

    def aggregate_parts(self):
        """Function which is responsible for calling YOLO detection
           and aggregating part pose, type and color in advance logical camera
            msg type which can be published

        Returns:
            bool: pose flag for success or failure bin side right
            bool: pose flag for success or failure bin side left
            list[geometry_msgs]: list of geometry msgs bin side right
            list[geometry_msgs]: list of geometry msgs bin side left
        """
        # initialize the part pose array
        part_pose_arr = []
        # initialize the advance logical camera msg for right side
        adv_msg_right = AdvancedLogicalCameraImage()
        # initialize the advance logical camera msg for left side
        adv_msg_left = AdvancedLogicalCameraImage()

        # loop over each camera fram to assign slot to part type and color
        for i in range(1, 9):
            current_frame = getattr(self, f"current_frame{i}")
            bbox_conf_category = self._model_prediction(current_frame)
            bin_cmd = i
            slot_centroids_bin = getattr(self, f"slot_centroids_bin{i}")
            slot_centroids_bin = {j: None for j in range(1, 10)}
            setattr(self, f"slot_centroids_bin{i}", slot_centroids_bin)

            # iterate over each detected part and assign to the slot
            for bbox in bbox_conf_category:
                bbox_coordinates = bbox[:4].tolist()
                centroid_x = (bbox_coordinates[0] + bbox_coordinates[2]) / 2
                centroid_y = (bbox_coordinates[1] + bbox_coordinates[3]) / 2
                if bin_cmd == 3 or bin_cmd == 5 or bin_cmd == 8 or bin_cmd == 2:
                    self.find_category_bin3258(centroid_x, centroid_y, bbox[5], bin_cmd)
                elif bin_cmd == 1 or bin_cmd == 4 or bin_cmd == 6 or bin_cmd == 7:
                    self.find_category_bin4167(centroid_x, centroid_y, bbox[5], bin_cmd)
        # loop over each camera frame to slot to part pose in each bin
        for i in range(1, 9):
            # get the sorted pose array for each bin
            current_arr = getattr(self, f"sorted_pose_array{i}")
            if current_arr is not None:
                # slice the array to get only xyz coordinates
                sliced_xyz_arr = current_arr[:, :3]
                # convert 3D points to 2D pixel coordinates
                xy_pix_points = self.plot_from_3d_2d(sliced_xyz_arr)
                # bin number
                bin_pose_cmd = i
                # dictionary for slot pose
                slot_pose_bin = getattr(self, f"slot_pose_bin{i}")
                # initialize dictionary for slot pose each time functionc called
                slot_pose_bin = {j: None for j in range(1, 10)}
                # set to slot pose class attribute associated with the bin number
                setattr(self, f"slot_pose_bin{i}", slot_pose_bin)
                # iterate over each pixel coordinate and assign to the slot
                for jdx, xy in enumerate(xy_pix_points):
                    x, y = xy[0], xy[1]
                    if (
                        bin_pose_cmd == 3
                        or bin_pose_cmd == 5
                        or bin_pose_cmd == 8
                        or bin_pose_cmd == 2
                    ):
                        self.find_pose_bin3258(x, y, bin_pose_cmd, current_arr[jdx, :])
                    elif (
                        bin_pose_cmd == 1
                        or bin_pose_cmd == 4
                        or bin_pose_cmd == 6
                        or bin_pose_cmd == 7
                    ):
                        self.find_pose_bin4167(x, y, bin_pose_cmd, current_arr[jdx, :])

        for i in range(1, 9):
            slot_centroids_bin = getattr(self, f"slot_centroids_bin{i}")
            slot_pose_bin = getattr(self, f"slot_pose_bin{i}")
            sorted_pose_array = getattr(self, f"sorted_pose_array{i}")
            sensor_pose = getattr(self, f"sensor_pose{i}")
            print("-------------------------------------------------------\n")
            print(f"slot_centroid_bin{i}: {slot_centroids_bin}")
            print("-------------------------------------------------------\n")
            print("-------------------------------------------------------\n")
            print(f"slot_pose_bin{i}: {slot_pose_bin}")
            print("-------------------------------------------------------\n")
            part_pose_publish = []

            if slot_pose_bin is not None:
                for key in slot_centroids_bin:
                    if slot_centroids_bin[key] is not None:
                        found_pose = slot_pose_bin[key]
                        part_pose_publish.append([slot_centroids_bin[key], found_pose])
                    else:
                        continue

            if len(part_pose_publish) > 0:
                for part_p in part_pose_publish:
                    part_pose = PartPose()
                    part = Part()
                    part.color = part_p[0][0]
                    part.type = part_p[0][1]
                    part_pose.part = part
                    part_pose.pose.position.x = part_p[1][0]
                    part_pose.pose.position.y = part_p[1][1]
                    part_pose.pose.position.z = part_p[1][2]
                    part_pose.pose.orientation.x = part_p[1][3]
                    part_pose.pose.orientation.y = part_p[1][4]
                    part_pose.pose.orientation.z = part_p[1][5]
                    part_pose.pose.orientation.w = part_p[1][6]
                    part_pose.pose = self.compute_part_pose_in_world(
                        part_pose.pose, sensor_pose
                    )
                    part_pose_arr.append(part_pose)
            if i == 4:
                adv_msg_right.part_poses = part_pose_arr
                adv_msg_right.tray_poses = []
                part_pose_arr = []
            if i == 8:
                adv_msg_left.part_poses = part_pose_arr
                adv_msg_left.tray_poses = []
        return True, True, adv_msg_left, adv_msg_right

    def _kts_rgb_camera_1_cb(self, msg):
        """Kit tray station rgb camera callback and aruco id detection

        Args:
            msg (sensor_msgs.msg.Image): Image message
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.tag_detector.detectMarkers(current_frame)
        corners = np.array(corners)
        corners = corners.reshape(-1, 4, 2)
        self.kittraystation1_slots = {1: None, 2: None, 3: None}
        if ids is not None:
            for idx, corner in enumerate(corners):
                centroid_x = 0
                centroid_y = 0
                for c in corner:
                    centroid_x += c[0]
                    centroid_y += c[1]
                centroid_x /= 4.0
                centroid_y /= 4.0
                if centroid_x < 240:
                    self.kittraystation1_slots[1] = ids[idx]
                elif centroid_x < 380:
                    self.kittraystation1_slots[2] = ids[idx]
                else:
                    self.kittraystation1_slots[3] = ids[idx]

    def _kts_rgb_camera_2_cb(self, msg):
        """Kit tray station rgb camera callback and aruco id detection

        Args:
            msg (sensor_msgs.msg.Image): Image message
        """
        current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = self.tag_detector.detectMarkers(current_frame)
        corners = np.array(corners)
        corners = corners.reshape(-1, 4, 2)
        self.kittraystation2_slots = {4: None, 5: None, 6: None}
        if ids is not None:
            for idx, corner in enumerate(corners):
                centroid_x = 0
                centroid_y = 0
                for c in corner:
                    centroid_x += c[0]
                    centroid_y += c[1]
                centroid_x /= 4.0
                centroid_y /= 4.0
                if centroid_x < 240:
                    self.kittraystation2_slots[4] = ids[idx]
                elif centroid_x < 380:
                    self.kittraystation2_slots[5] = ids[idx]
                else:
                    self.kittraystation2_slots[6] = ids[idx]

    def _kts_logical_camera_1_cb(self, msg):
        """Kit tray station logical camera callback

        Args:
            msg (list[geometry_msgs]): List of geometry messages
        """
        pose_list = []
        for pose in msg.tray_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.kts_sensor_pose1 = msg.sensor_pose
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_ktspose_array1 = pose_array[pose_array[:, 1].argsort()[::-1]]
        else:
            self.sorted_ktspose_array1 = None

    def _kts_logical_camera_2_cb(self, msg):
        """Kit tray station logical camera callback

        Args:
            msg (list[geometry_msgs]): List of geometry messages
        """
        pose_list = []
        for pose in msg.tray_poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            x_quat = pose.orientation.x
            y_quat = pose.orientation.y
            z_quat = pose.orientation.z
            w_quat = pose.orientation.w
            pose_list.append([x, y, z, x_quat, y_quat, z_quat, w_quat])
        self.kts_sensor_pose2 = msg.sensor_pose
        if len(pose_list) > 0:
            pose_array = np.array(pose_list)
            self.sorted_ktspose_array2 = pose_array[pose_array[:, 1].argsort()[::-1]]
        else:
            self.sorted_ktspose_array2 = None

    def aggregate_kittrays(self):
        """Kit tray station pose and aruco id aggregation function

        Returns:
            list[aruco_id, geometry_msgs]: List of aruco ids corresponding to its pose
        """
        adv_msg_list = []
        for i in range(1, 3):
            slot_centroids_kts = getattr(self, f"kittraystation{i}_slots")
            sorted_pose_array = getattr(self, f"sorted_ktspose_array{i}")
            sensor_pose = getattr(self, f"kts_sensor_pose{i}")
            kit_pose_publish = []
            print("===============================================\n")
            print(f"Kittraystationslots{i}: {slot_centroids_kts}")
            print("===============================================\n")
            if sorted_pose_array is not None and slot_centroids_kts is not None:
                sorted_pose_list = sorted_pose_array.tolist()
                idx = 0
                for key in slot_centroids_kts:
                    if slot_centroids_kts[key] is not None:
                        found_pose = sorted_pose_list[idx]
                        idx += 1
                        kit_pose_publish.append([slot_centroids_kts[key], found_pose])
                    else:
                        continue
            adv_msg = AdvancedLogicalCameraImage()
            if len(kit_pose_publish) > 0:
                kit_pose_arr = []
                for kit_t in kit_pose_publish:
                    kit_pose = KitTrayPose()
                    kit_pose.id = int(kit_t[0][0])
                    kit_pose.pose.position.x = kit_t[1][0]
                    kit_pose.pose.position.y = kit_t[1][1]
                    kit_pose.pose.position.z = kit_t[1][2]
                    kit_pose.pose.orientation.x = kit_t[1][3]
                    kit_pose.pose.orientation.y = kit_t[1][4]
                    kit_pose.pose.orientation.z = kit_t[1][5]
                    kit_pose.pose.orientation.w = kit_t[1][6]
                    kit_pose_arr.append(kit_pose)
                adv_msg.tray_poses = kit_pose_arr
                adv_msg.part_poses = []
                adv_msg.sensor_pose = sensor_pose
            adv_msg_list.append(adv_msg)
        return True, True, adv_msg_list[0], adv_msg_list[1]

    def _agv1_advanced_cb(self, msg):
        """AGV1 advanced camera callback

        Args:
            msg (AdvancedLogicalCameraImage): ARIAC Message from advanced logical camera
        """
        agv1_part_poses = msg.part_poses
        self.agv1_sensor_pose = msg.sensor_pose
        # self.agv1_tray_pose = msg.tray_poses
        part_pose_list = []
        for idx, partpose in enumerate(agv1_part_poses):
            pp_msg = PartPose()
            pp_msg.part = partpose.part
            part_p = partpose.pose
            world_part_p = self.compute_part_pose_in_world(
                part_p, self.agv1_sensor_pose
            )
            pp_msg.pose = world_part_p
            part_pose_list.append(pp_msg)
        self.agv1_part_poses = part_pose_list

    def _agv2_advanced_cb(self, msg):
        """AGV2 advanced camera callback

        Args:
            msg (AdvancedLogicalCameraImage): ARIAC Message from advanced logical camera
        """
        agv2_part_poses = msg.part_poses
        self.agv2_sensor_pose = msg.sensor_pose
        # self.agv2_tray_pose = msg.tray_poses
        part_pose_list = []
        for idx, partpose in enumerate(agv2_part_poses):
            pp_msg = PartPose()
            pp_msg.part = partpose.part
            part_p = partpose.pose
            world_part_p = self.compute_part_pose_in_world(
                part_p, self.agv2_sensor_pose
            )
            pp_msg.pose = world_part_p
            part_pose_list.append(pp_msg)
        self.agv2_part_poses = part_pose_list

    def _agv3_advanced_cb(self, msg):
        """AGV3 advanced camera callback

        Args:
            msg (AdvancedLogicalCameraImage): ARIAC Message from advanced logical camera
        """
        agv3_part_poses = msg.part_poses
        self.agv3_sensor_pose = msg.sensor_pose
        # self.agv3_tray_pose = msg.tray_poses
        part_pose_list = []
        for idx, partpose in enumerate(agv3_part_poses):
            pp_msg = PartPose()
            pp_msg.part = partpose.part
            part_p = partpose.pose
            world_part_p = self.compute_part_pose_in_world(
                part_p, self.agv3_sensor_pose
            )
            pp_msg.pose = world_part_p
            part_pose_list.append(pp_msg)
        self.agv3_part_poses = part_pose_list

    def _agv4_advanced_cb(self, msg):
        """AGV4 advanced camera callback

        Args:
            msg (AdvancedLogicalCameraImage): ARIAC Message from advanced logical camera
        """
        agv4_part_poses = msg.part_poses
        self.agv4_sensor_pose = msg.sensor_pose
        # self.agv4_tray_pose = msg.tray_poses
        part_pose_list = []
        for idx, partpose in enumerate(agv4_part_poses):
            pp_msg = PartPose()
            pp_msg.part = partpose.part
            part_p = partpose.pose
            world_part_p = self.compute_part_pose_in_world(
                part_p, self.agv4_sensor_pose
            )
            pp_msg.pose = world_part_p
            part_pose_list.append(pp_msg)
        self.agv4_part_poses = part_pose_list

    def handle_request(self, request, response):
        """Server request handler for handling requests from the competitor package for part and tray poses

        Args:
            request (bool): Boolean flag for requesting kit trays or bin part poses
            response (bool): Composite response of Boolean flag and list of poses
            for requesting kit trays or bin part poses

        Returns:
            bool: Returns success or failure
        """
        kts_flag = request.request_kts
        bins_flag = request.request_bins
        agv_flag1 = request.request_agv1
        agv_flag2 = request.request_agv2
        agv_flag3 = request.request_agv3
        agv_flag4 = request.request_agv4
        response.response_kts1 = response.response_kts2 = (
            response.response_left_bins
        ) = response.response_right_bins = response.response_agv1 = (
            response.response_agv2
        ) = response.response_agv3 = response.response_agv4 = False
        if kts_flag:
            (
                response.response_kts1,
                response.response_kts2,
                response.kts1,
                response.kts2,
            ) = self.aggregate_kittrays()
        if bins_flag:
            (
                response.response_left_bins,
                response.response_right_bins,
                response.left_bins,
                response.right_bins,
            ) = self.aggregate_parts()
        if agv_flag1:
            response.response_agv1 = True
            adv1_msg = AdvancedLogicalCameraImage()
            adv1_msg.part_poses = self.agv1_part_poses
            response.agv1_parts = adv1_msg
        if agv_flag2:
            response.response_agv2 = True
            adv2_msg = AdvancedLogicalCameraImage()
            adv2_msg.part_poses = self.agv2_part_poses
            response.agv2_parts = adv2_msg
        if agv_flag3:
            response.response_agv3 = True
            adv3_msg = AdvancedLogicalCameraImage()
            adv3_msg.part_poses = self.agv3_part_poses
            response.agv3_parts = adv3_msg
        if agv_flag4:
            response.response_agv4 = True
            adv4_msg = AdvancedLogicalCameraImage()
            adv4_msg.part_poses = self.agv4_part_poses
            response.agv4_parts = adv4_msg

        return response
