###########################
# Program for HSV setting #
# Folloe next steps:      #
###########################

# 1) Set global settings
import func
import numpy as np
import cv2
from typing import List, Tuple
ENABLE_FILTERS = True

ENABLE_WHITE_BALANCE = False

ENABLE_ADJUST_GAMMA = False
# if adjust gamma is active with precision 1 digit after the decimal point
ADJUST_GAMMA_LEVEL = 2
# cv2 settings window will show trackbar for gamma property
ENABABLE_ADJUST_GAMMA_TRACKBAR = True
# maximum value for cv2 adjust gamma trackbar
ADJUST_GAMMA_TRACKBAR_MAX_VALUE = 10

ENABLE_UNSHARP_MASK = True

ENABLE_CLAHE_CORRECTION = False

ENABLE_FUSION = False

# 2) Choose one of theese types and set right property (if None then ignores):
CAMERA_ID = 2  # device camera id
VIDEO_FILE = "video.mkv"  # "./video.mkv" # video file path
IMAGE_FILE = None  # image file path

# 3) Additional settings:

# if you chose camera:
CAMERA_DELAY = 1  # camera delay in milliseconds

# if you chose videp:
VIDEO_DELAY = 1  # video delay in milliseconds
REPLAY_VIDEO = True  # restart video on end


def create_mask(frame: np.ndarray, hsv_low: Tuple[int], hsv_high: Tuple[int]) -> np.ndarray:
    """Creates mask from hsv frame

    Args:
        frame (np.ndarray): [description]
        hsv_low (Tuple[int]): [description]
        hsv_high (Tuple[int]): [description]

    Returns:
        np.ndarray: [description]
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, hsv_low, hsv_high)


def detect_contours(frame: np.ndarray, hsv_low: Tuple[int], hsv_high: Tuple[int], mode: int = cv2.RETR_EXTERNAL, method: int = cv2.CHAIN_APPROX_SIMPLE) -> List[np.ndarray]:
    """Detects in hsv range

    Args:
        frame (np.ndarray): cv2 frame
        hsv_low (Tuple[int]): HSV low range
        hsv_high (Tuple[int]): HSV high range
        mode (int): cv2 contour retrieval mode
        method (int): cv2 contour approximation method

    Returns:
        List[np.ndarray]: List of cv2 contours
    """
    mask = create_mask(frame, hsv_low, hsv_high)
    contours = cv2.findContours(mask, mode, method)
    return contours


def constraint(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v


def process_main(frame: np.ndarray) -> np.ndarray:
    hue = cv2.getTrackbarPos("hue", "settings")
    saturation = cv2.getTrackbarPos("saturation", "settings")
    value = cv2.getTrackbarPos("value", "settings")
    hue_range = cv2.getTrackbarPos("hue range", "settings")
    saturation_range = cv2.getTrackbarPos("saturation range", "settings")
    value_range = cv2.getTrackbarPos("value range", "settings")
    hsv_low = (constraint(hue - hue_range, 360, 0), constraint(saturation -
               saturation_range, 255, 0), constraint(value - value_range, 255, 0))
    hsv_high = (constraint(hue + hue_range, 360, 0), constraint(saturation +
                saturation_range, 255, 0), constraint(value + value_range, 255, 0))
    mask = create_mask(frame, hsv_low, hsv_high)
    print(
        f"({hsv_low[0]},{hsv_low[1]},{hsv_low[2]}),({hsv_high[0]},{hsv_high[1]},{hsv_high[2]})")
    return mask


def process_corection(frame):
    if ENABLE_FILTERS:
        frame_corrected = frame.copy()
        if ENABLE_WHITE_BALANCE:
            frame_corrected = func.white_balance(frame_corrected)
        if ENABLE_ADJUST_GAMMA:
            frame_corrected = func.adjust_gamma(frame_corrected, constraint(cv2.getTrackbarPos(
                "gamma / 10", "settings") / 10, ADJUST_GAMMA_TRACKBAR_MAX_VALUE * 10, 0.1) if ENABABLE_ADJUST_GAMMA_TRACKBAR else ADJUST_GAMMA_LEVEL)
        if ENABLE_UNSHARP_MASK:
            frame_corrected = func.unsharp_mask(frame_corrected)
        if ENABLE_CLAHE_CORRECTION:
            frame_corrected = func.CLAHE(frame_corrected)
        if ENABLE_FUSION:
            frame_corrected = func.fusion([
                func.adjust_gamma(frame_corrected, 0.5),
                func.adjust_gamma(frame_corrected, 0.7),
                func.adjust_gamma(frame_corrected, 1),
                func.adjust_gamma(frame_corrected, 2),
                func.adjust_gamma(frame_corrected, 3)
            ])
        mask_corrected = process_main(frame_corrected)
        cv2.imshow("frame corrected", frame_corrected)
        cv2.imshow("mask corrected", mask_corrected)


if __name__ == "__main__":
    cv2.namedWindow("settings", flags=cv2.WINDOW_FREERATIO)
    cv2.createTrackbar("hue", "settings", 0, 360, lambda x: None)
    cv2.createTrackbar("saturation", "settings", 0, 255, lambda x: None)
    cv2.createTrackbar("value", "settings", 0, 255, lambda x: None)
    cv2.createTrackbar("hue range", "settings", 0, 360, lambda x: None)
    cv2.createTrackbar("saturation range", "settings", 0, 255, lambda x: None)
    cv2.createTrackbar("value range", "settings", 0, 255, lambda x: None)
    if ENABLE_FILTERS:
        if ENABLE_ADJUST_GAMMA and ENABABLE_ADJUST_GAMMA_TRACKBAR:
            cv2.createTrackbar("gamma / 10", "settings", int(ADJUST_GAMMA_LEVEL * 10),
                               ADJUST_GAMMA_TRACKBAR_MAX_VALUE * 10, lambda x: None)
    if CAMERA_ID is not None:
        cap = cv2.VideoCapture(CAMERA_ID)
        while True:
            r, frame = cap.read()
            assert r, "Video frame was not captured!"
            mask = process_main(frame)
            process_corection(frame)
            cv2.imshow("frame", frame)
            cv2.imshow("mask", mask)
            if cv2.waitKey(CAMERA_DELAY) != -1:
                cv2.destroyAllWindows()
                exit(0)
    elif VIDEO_FILE is not None:
        while True:
            cap = cv2.VideoCapture(VIDEO_FILE)
            while True:
                r, frame = cap.read()
                if not r and REPLAY_VIDEO:
                    break
                assert r, "Video ended or video frame was not captures"
                mask = process_main(frame)
                process_corection(frame)
                cv2.imshow("frame", frame)
                cv2.imshow("mask", mask)
                if cv2.waitKey(VIDEO_DELAY) != -1:
                    cv2.destroyAllWindows()
                    exit(0)
    elif IMAGE_FILE is not None:
        frame = cv2.imread(IMAGE_FILE)
        while True:
            mask = process_main(frame)
            process_corection(frame)
            cv2.imshow("frame", frame)
            cv2.imshow("mask", mask)
            if cv2.waitKey(1) != -1:
                cv2.destroyAllWindows()
                exit(0)
