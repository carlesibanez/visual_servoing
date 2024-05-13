import cv2
import numpy as np

def apply_tuning(cv_image, tuning_params, active_window):

    low_h = tuning_params['low_h']
    high_h = tuning_params['high_h']
    low_s = tuning_params['low_s']
    high_s = tuning_params['high_s']
    low_v = tuning_params['low_v']
    high_v = tuning_params['high_v']

    cropped_img = cv_image[active_window[1]:active_window[3], active_window[0]:active_window[2]]
    frame_HSV = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
    img_out = cv2.inRange(frame_HSV, 
                                  (low_h, low_s, low_v), 
                                  (high_h, high_s, high_v))

    kernel = np.ones((3,3),np.uint8)
    img_out = cv2.dilate(img_out, kernel, iterations=2)
    img_out = cv2.erode(img_out, kernel, iterations=2)

    return img_out

def generate_visualization(original_img, cv_image, active_window, center, tuning_params):
    
    color = (0, 255, 0)
    line = 5

    img_out = original_img.copy()
    img_out[active_window[1]:active_window[3], active_window[0]:active_window[2]] = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR)
    img_out = cv2.rectangle(img_out,(active_window[0],active_window[1]),(active_window[2],active_window[3]),color,line)

    if center is not None:
        img_out = cv2.circle(img_out, center, 7, (0, 255, 0), -1)
        # img_out = cv2.putText(img_out, "centroid", (center[0] - 20, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        img_out = cv2.line(img_out, (center[0], active_window[1]), (center[0], active_window[3]), (0, 255, 0), 3)

    
    return img_out
