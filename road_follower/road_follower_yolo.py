import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix

from sensor_msgs.msg import CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped

from ros2_pydata import compressedimage_to_np, to_ackermann

from road_follower.yolo_pipeline import RoadControlPipeline
from road_follower.plot_utils import plot_mae

class RoadFollowerNode(Node):

    def __init__(self):
        super().__init__('road_follower')  # Initialize ROS 2 node

        # Storage for evaluation
        self.true_angle = None
        self.predicted_angles = []
        self.true_angles = []

        # QoS setup for sensor data (matches camera publishers, avoids QoS mismatch issues)
        self.qos_profile = qos_profile_sensor_data
        self.qos_profile.depth = 1  # Only keep latest message

        # Subscribe to compressed mask images
        self.sub = self.create_subscription(CompressedImage, '/mask', self.mask_callback, self.qos_profile)

        # Subscribe to ground truth steering
        self.gt_sub = self.create_subscription(AckermannDriveStamped, '/rc/ackermann_cmd', self.gt_callback, self.qos_profile)

        # Publish steering + speed commands
        self.pub = self.create_publisher(AckermannDriveStamped, '/autonomous/ackermann_cmd', self.qos_profile)

        self.speed = 0.8  # Constant forward speed

        self.ctrl = RoadControlPipeline()  # Your control logic lives here

        self.get_logger().info("Road Follower started!")

    def gt_callback(self, msg: AckermannDriveStamped):
        # Store latest ground truth steering
        self.true_angle = msg.drive.steering_angle

    def mask_callback(self, msg: CompressedImage):
        # Called whenever a new mask image arrives

        mask, _timestamp = compressedimage_to_np(msg)  # Convert ROS image → NumPy array

        steering_angle = self.ctrl.run_control_pipeline(mask)  # Compute steering from mask

        cmd = to_ackermann(self.speed, steering_angle, _timestamp)  # Create drive command

        self.pub.publish(cmd)  # Send command to vehicle

        # Store for evaluation
        self.predicted_angles.append(steering_angle)
        self.true_angles.append(self.true_angle)

    def plot_results(self, pkg_path):
        plot_mae(self.predicted_angles, self.true_angles, pkg_path)

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    node = RoadFollowerNode()  # Create node

    pkg_path = get_package_prefix('road_follower').replace('install', 'src')

    try:
        rclpy.spin(node) # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results(pkg_path) # Plot when shutting down
        node.destroy_node() # Cleanup

        if rclpy.ok():
            rclpy.shutdown() # Shutdown ROS 2

if __name__ == '__main__':
    main()