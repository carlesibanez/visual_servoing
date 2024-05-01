import cv2

def no_op(x):
    pass

def tuning_window(tuning_params, window_name='Tuning Window'):
    # Create a window to display the image
    cv2.namedWindow('Tuning Window', cv2.WINDOW_NORMAL)

    # Create trackbars for tuning the parameters
    cv2.createTrackbar('x_min', window_name, tuning_params['x_min'], 100, no_op)
    cv2.createTrackbar('x_max', window_name, tuning_params['x_max'], 100, no_op)
    cv2.createTrackbar('y_min', window_name, tuning_params['y_min'], 100, no_op)
    cv2.createTrackbar('y_max', window_name, tuning_params['y_max'], 100, no_op)

    cv2.createTrackbar('low_h', window_name , tuning_params['low_h'], 180, no_op)
    cv2.createTrackbar('high_h', window_name , tuning_params['high_h'], 180, no_op)
    cv2.createTrackbar('low_s', window_name , tuning_params['low_s'], 255, no_op)
    cv2.createTrackbar('high_s', window_name , tuning_params['high_s'], 255, no_op)
    cv2.createTrackbar('low_v', window_name , tuning_params['low_v'], 255, no_op)
    cv2.createTrackbar('high_v', window_name , tuning_params['high_v'], 255, no_op)


def get_tuning_params(tuning_params, window_name='Tuning Window'):
    trackbar_names = tuning_params.keys()
    return {name: cv2.getTrackbarPos(name, window_name) for name in trackbar_names}

def visualize_results(original_img, cv_image, active_window, center, tuning_params, window_name='Tuning Window'):
    # Extract the tuning parameters
    x_min = tuning_params['x_min']
    x_max = tuning_params['x_max']
    y_min = tuning_params['y_min']
    y_max = tuning_params['y_max']

    color = (0, 255, 0)
    line = 5

    img_out = original_img.copy()
    img_out[active_window[1]:active_window[3], active_window[0]:active_window[2]] = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR)
    img_out = cv2.rectangle(img_out,(active_window[0],active_window[1]),(active_window[2],active_window[3]),color,line)

    if center is not None:
        img_out = cv2.circle(img_out, center, 7, (0, 255, 0), -1)
        # img_out = cv2.putText(img_out, "centroid", (center[0] - 20, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        img_out = cv2.line(img_out, (center[0], active_window[1]), (center[0], active_window[3]), (0, 255, 0), 3)

    
    cv2.imshow(window_name, img_out)
    return img_out


def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    return [int(a*b/100) for a,b in zip(rect_perc, scale)]

def wait_on_gui():
    cv2.waitKey(1)