# road_follower.py
# Simple road following node that detects lanes and steers the vehicle

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from functools import partial

from .pipeline import preprocess_image, update_window, compute_offset, predict_steering, get_corners

class RoadFollowerNode(Node):

    def __init__(self):
        super().__init__("road_follower")

        # ===== CONFIGURATION =====
        # Speed control
        self.speed = 1.0  # Fixed speed (m/s)
        
        # Image processing parameters (from Colab)
        self.canny_min = 30  # Canny edge detection lower threshold
        self.canny_max = 90  # Canny edge detection upper threshold
        self.kernel = (7, 7)  # Blur kernel size
        
        # Image dimensions
        self.im_width = 640  # Camera image width
        self.im_height = 360 # Camera image height
        
        # Road detection parameters
        self.win_x = 610  # Initial window x-position (image center)
        self.win_y = 260  # Window y-position (bottom of image)
        self.win_w = 90  # Window width
        self.win_h = 30  # Window height
        self.half_road_width = 490  # Half width of road in pixels
        self.veh_xcenter = 380  # Vehicle center in image (pixels)
        
        # Steering control
        self.kp = 7.5e-4  # Proportional gain for steering
        self.bias = 0.27  # Steering bias offset

        # ===== ROS2 SETUP =====
        # Subscribe to camera images
        self.sub = self.create_subscription(
            Image, 
            "/camera/camera/color/image_raw", 
            self.image_callback, 
            10
        )
        
        # Publish steering commands
        self.pub = self.create_publisher(
            AckermannDrive, 
            "/autonomous/ackermann_cmd", 
            10
        )
        
        self.get_logger().info("Road Follower started!")


        # Bind all constant parameters once using partial so runtime calls only pass live data.
        # e.g. self.predict_steering(offset) instead of predict_steering(offset, self.kp, self.bias)

        self.get_corners = partial(get_corners, cy=self.win_y, win_w=self.win_w,
                                   win_h=self.win_h, image_w=self.im_width, image_h=self.im_height)
        
        self.preprocess_image = partial(preprocess_image, canny_min=self.canny_min,
                                        canny_max=self.canny_max, kernel=self.kernel)

        self.compute_offset = partial(compute_offset, half_road_width=self.half_road_width,
                                      veh_xcenter=self.veh_xcenter)

        self.predict_steering = partial(predict_steering, kp=self.kp, bias=self.bias)
        


    def image_callback(self, msg):
        """Process each camera frame and compute steering"""
        
        # Step 1: Convert ROS image message to numpy array
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        
        # Step 2: Preprocess image (Canny edge detection)
        im_canny = self.preprocess_image(image)
        
        # Step 3: Find the road using sliding window
        corners = self.get_corners(self.win_x)
        success, self.win_x = update_window(corners, im_canny)


        # Step 4: If road found, compute steering and drive
        if success:
            # Calculate how far off-center we are
            offset_px = self.compute_offset(self.win_x)
            
            # Convert offset to steering angle
            steering = self.predict_steering(offset_px)
            
            # Create and publish drive command
            cmd = AckermannDrive()
            cmd.steering_angle = float(steering)
            cmd.speed = float(self.speed)
            
            self.pub.publish(cmd)
        else:
            # No road detected - could add safety stop here
            self.get_logger().warn("Road not detected!")


def main(args=None):
    """Entry point for the node"""
    import rclpy
    
    rclpy.init(args=args)
    node = RoadFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()