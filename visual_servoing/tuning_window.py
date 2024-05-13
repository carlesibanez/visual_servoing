import cv2

def no_op(x):
    pass

def tuning_window(tuning_params, window_name='Tuning Window'):
    # Create a window to display the image
    cv2.namedWindow('Tuning Window', cv2.WINDOW_NORMAL)

    # Create trackbars for tuning the parameters
    cv2.createTrackbar('x_min', window_name, tuning_params['x_min'], tuning_params['img_width'], no_op)
    cv2.createTrackbar('x_max', window_name, tuning_params['x_max'], tuning_params['img_width'], no_op)
    cv2.createTrackbar('y_min', window_name, tuning_params['y_min'], tuning_params['img_height'], no_op)
    cv2.createTrackbar('y_max', window_name, tuning_params['y_max'], tuning_params['img_height'], no_op)

    cv2.createTrackbar('low_h', window_name , tuning_params['low_h'], 180, no_op)
    cv2.createTrackbar('high_h', window_name , tuning_params['high_h'], 180, no_op)
    cv2.createTrackbar('low_s', window_name , tuning_params['low_s'], 255, no_op)
    cv2.createTrackbar('high_s', window_name , tuning_params['high_s'], 255, no_op)
    cv2.createTrackbar('low_v', window_name , tuning_params['low_v'], 255, no_op)
    cv2.createTrackbar('high_v', window_name , tuning_params['high_v'], 255, no_op)


def get_tuning_params(tuning_params, window_name='Tuning Window'):
    trackbar_names = tuning_params.keys()
    return {name: cv2.getTrackbarPos(name, window_name) for name in trackbar_names if name != 'img_width' and name != 'img_height'}


def wait_on_gui():
    cv2.waitKey(1)