import cv2

def no_op(x):
    pass

def tuning_window(tuning_params):
    # Create a window to display the image
    cv2.namedWindow('Tuning Window', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('Tuning Window', 800, 600)

    # Create trackbars for tuning the parameters
    cv2.createTrackbar('x_min', 'Tuning Window', tuning_params['x_min'], 100, no_op)
    cv2.createTrackbar('x_max', 'Tuning Window', tuning_params['x_max'], 100, no_op)
    cv2.createTrackbar('y_min', 'Tuning Window', tuning_params['y_min'], 100, no_op)
    cv2.createTrackbar('y_max', 'Tuning Window', tuning_params['y_max'], 100, no_op)
    # cv2.createTrackbar('x_min', 'Tuning', 0, 100, no_op)
    # cv2.createTrackbar('x_max', 'Tuning', 0, 100, no_op)
    # cv2.createTrackbar('y_min', 'Tuning', 0, 100, no_op)
    # cv2.createTrackbar('y_max', 'Tuning', 0, 100, no_op)

    # cv2.setTrackbarPos('x_min', 'Tuning', tuning_params['x_min'])
    # cv2.setTrackbarPos('x_max', 'Tuning', tuning_params['x_max'])
    # cv2.setTrackbarPos('y_min', 'Tuning', tuning_params['y_min'])
    # cv2.setTrackbarPos('y_max', 'Tuning', tuning_params['y_max'])

    # Display the image
    # cv2.imshow('Tuning Window', cv_image)

    # Wait for the user to press a key
    # key = cv2.waitKey(1)


def get_tuning_params(tuning_params):
    trackbar_names = tuning_params.keys()
    return {name: cv2.getTrackbarPos(name, 'Tuning Window') for name in trackbar_names}

def tuned_image(cv_image, tuning_params):
    # Extract the tuning parameters
    x_min = tuning_params['x_min']
    x_max = tuning_params['x_max']
    y_min = tuning_params['y_min']
    y_max = tuning_params['y_max']

    search_window = [x_min, y_min, x_max, y_max]
    rect_px = convert_rect_perc_to_pixels(search_window, cv_image)

    color = (0, 255, 0)
    line = 5

    # Apply the tuning parameters
    # cv_image = cv_image[y_min:y_max, x_min:x_max]
    cv_image = cv2.rectangle(cv_image,(rect_px[0],rect_px[1]),(rect_px[2],rect_px[3]),color,line)

    return cv_image


def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    return [int(a*b/100) for a,b in zip(rect_perc, scale)]

def wait_on_gui():
    cv2.waitKey(1)