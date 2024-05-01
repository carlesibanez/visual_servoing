import cv2
import numpy as np

def apply_tuning(cv_image, tuning_params):
    # Extract the tuning parameters
    x_min = tuning_params['x_min']
    x_max = tuning_params['x_max']
    y_min = tuning_params['y_min']
    y_max = tuning_params['y_max']

    search_window = [x_min, y_min, x_max, y_max]
    rect_px = convert_rect_perc_to_pixels(search_window, cv_image)

    low_h = tuning_params['low_h']
    high_h = tuning_params['high_h']
    low_s = tuning_params['low_s']
    high_s = tuning_params['high_s']
    low_v = tuning_params['low_v']
    high_v = tuning_params['high_v']

    cropped_img = cv_image[rect_px[1]:rect_px[3], rect_px[0]:rect_px[2]]
    frame_HSV = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
    img_out = cv2.inRange(frame_HSV, 
                                  (low_h, low_s, low_v), 
                                  (high_h, high_s, high_v))

    kernel = np.ones((3,3),np.uint8)
    img_out = cv2.dilate(img_out, kernel, iterations=2)
    img_out = cv2.erode(img_out, kernel, iterations=2)

    return img_out

# TODO: Implement generate_visualization function, to draw the circle and rectangle on the image

def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    return [int(a*b/100) for a,b in zip(rect_perc, scale)]