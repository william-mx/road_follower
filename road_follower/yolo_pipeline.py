import numpy as np

class RoadControlPipeline:
    def __init__(self):
        # Image size
        self.im_width = 640
        self.im_height = 360

        # Vehicle position
        self.veh_xcenter = 380

        # Road geometry
        self.road_width_px = 490
        self.half_road_width = self.road_width_px // 2

        # Mask IDs
        self.id_left = 4
        self.id_right = 5

        # State
        self.state = "straight"

        # Threshold
        self.pixel_threshold = 100

        self.seq = 0

    def run_control_pipeline(self, mask):
        # TODO: Implement your full control logic here

        # Count pixels in entire mask
        cnt_left_ids = np.count_nonzero(mask == self.id_left)
        cnt_right_ids = np.count_nonzero(mask == self.id_right)

        print(f"{self.seq}: Left count: {cnt_left_ids}, Right count: {cnt_right_ids}")
        
        if cnt_left_ids > self.pixel_threshold and cnt_right_ids > self.pixel_threshold:
            self.state = "straight"
            steering_angle = 0.0

        elif cnt_left_ids > self.pixel_threshold:
            self.state = "right_curve"
            steering_angle = 0.2

        elif cnt_right_ids > self.pixel_threshold:
            self.state = "left_curve"
            steering_angle = -0.2

        else:
            self.state = "lost"
            steering_angle = 0.0

        self.seq+=1
        return steering_angle