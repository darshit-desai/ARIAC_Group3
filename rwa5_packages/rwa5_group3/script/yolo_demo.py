#!/usr/bin/env python3

from rwa5_group3.yolo_interface import YOLOInterface
import rclpy
import os


def main(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    # Get the current directory of the script
    current_directory = os.path.dirname(os.path.realpath(__file__))

    # Construct the path to the 'model' folder
    model_directory = os.path.join(current_directory, 'model')

    # Construct the path to the 'best.pt' file
    best_pt_file = os.path.join(model_directory, 'best.pt')
    rclpy.init(args=args)
    node = YOLOInterface(best_pt_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shPROJECT_NAMEutdown()


if __name__ == "__main__":
    main()  # Execute the main function when the script is run