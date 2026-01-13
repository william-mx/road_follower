import cv2
import numpy as np

def get_corners(cx, cy, win_w, win_h, image_w = 640, image_h = 360):
    # Calculate the search window coordinates, ensuring they are within image bounds
    x1 = max(0, int(cx - win_w / 2))
    y1 = max(0, int(cy - win_h / 2))
    x2 = min(image_w, x1 + win_w)
    y2 = min(image_h, y1 + win_h)
    return x1, x2, y1, y2

def preprocess_image(image, canny_min, canny_max, kernel):

    # Convert to grayscale
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to the grayscale image
    im_blur = cv2.GaussianBlur(im_gray, kernel, 0)

    # Apply Canny edge detection to the preprocessed image
    im_canny = cv2.Canny(im_blur, canny_min, canny_max) # [0 or 255]

    return im_canny

def update_window(corners, im_canny):
    # Adjust window center by detecting lane line pixels in the image patch
    # Success means pixels were found and a new center is calculated

    x1, x2, y1, y2 = corners

    # Extract the relevant image patch and identify non-zero pixels (lane line pixels)
    patch = im_canny[y1:y2, x1:x2]

    x2 = x1 + np.searchsorted(np.cumsum(np.sum(patch > 0, axis=0)), 240)
    patch = im_canny[y1:y2, x1:x2]

    _, xs = np.nonzero(patch)


    # Check if any lane line pixels are detected
    # If not, return False along with the current center
    if len(xs) == 0:
        win_x = (x1 + x2) // 2
        return False, win_x


    # Calculate the new center based on the mean of the detected pixels
    updated_center = int(x1 + np.mean(xs))

    return True, updated_center

def compute_offset(win_x, half_road_width, veh_xcenter):
    # Estimate the road center from the tracked lane position
    pox_x = win_x - half_road_width

    # Compute how far the vehicle is from the road center (in pixels)
    offset_px = pox_x - veh_xcenter

    # This offset is the control error used for steering
    return offset_px

def predict_steering(offsets, kp=1.0, bias=0.0):
    # Proportional controller: map lateral offsets to steering angles
    return kp * offsets + bias
