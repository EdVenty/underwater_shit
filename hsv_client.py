__author__ = "Edventy <edventyh@gmail.com>"
__version__ = 0.2

##################################################################
# Video client with binary hsv filter and camera exposure setter #
# Works with 2 cameras with ids 0 and 1                          #
# Client side                                                    #
##################################################################

############################# Config ############################
# ----- Important settings -----
# HOST = "localhost"
HOST = '10.3.141.1'  # uneversal MUR robot ip
PORT = 65432

# ----- Set contours config -----
SHOW_CONTOURS = True # Show contours on the draw image.
CONTOURS_COLOR = (0, 0, 255)
HSV_TRACKBAR_MODE = 2 # 1 - with ranges, 2 - with min/max

# @------------------------------------@
# ----- Enable auto frame config -----
ENABLE_AUTO_FRAME_CONFIG = True # picks up all the frame settings from server

# ----- Or set manually -----
TRANSFER_MODE = 2 # 1 - pickle, 2 - imcode !SERVER MODE AND CLIENT MODE MUST BE THE SAME!
RESIZE_END_IMAGE_TO = (640, 480) # pixels. After recive image width and height will be resized to.
# @------------------------------------@

############################ End config #########################

import pickle
import base64
import socket
from typing import Any, Tuple
import hsv_server
import cv2
import time
import tkinter as tk
from threading import Thread
from queue import Queue
import numpy as np
import json

WINDOW_SETTINGS = "settings"

QUEUE_HSV_TYPE_HUE = "hue"
QUEUE_HSV_TYPE_SATURATION = "saturation"
QUEUE_HSV_TYPE_VALUE = "value"
QUEUE_HSV_TYPE_HUE_RANGE = "hue-range"
QUEUE_HSV_TYPE_SATURATION_RANGE = "saturation-range"
QUEUE_HSV_TYPE_VALUE_RANGE = "value-range"

QUEUE_HSV_TYPE_HUE_MIN = "hue-min"
QUEUE_HSV_TYPE_SATURATION_MIN = "saturation min"
QUEUE_HSV_TYPE_VALUE_MIN = "value min"
QUEUE_HSV_TYPE_HUE_MAX = "hue max"
QUEUE_HSV_TYPE_SATURATION_MAX = "saturation max"
QUEUE_HSV_TYPE_VALUE_MAX = "value max"

QUEUE_PRINT_HSV = "print-hsv"
QUEUE_CLOSE_WINDOW = "close-window"
QUEUE_CAM0_EXPO = "cam0-expo"
QUEUE_CAM1_EXPO = "cam1-expo"
QUEUE_CAM0_AUTO_EXPO = "cam0-auto-expo"
QUEUE_CAM1_AUTO_EXPO = "cam1-auto-expo"
QUEUE_CAM0_SWITCH_RECORD = "cam0-switch-record"
QUEUE_CAM0_SWITCH_RECORD = "cam0-switch-record"

def get_frame(s: socket.socket, cam_id: int) -> Any:
    hsv_server.send_msg(
        s, "{} {}".format(hsv_server.COMMA_GET_FRAME, cam_id).encode('utf-8'))
    data = hsv_server.recv_msg(s)
    if TRANSFER_MODE == 1:
        return pickle.loads(data)
    elif TRANSFER_MODE == 2:
        img = base64.b64decode(data)
        npimg = np.fromstring(img, dtype=np.uint8)
        return cv2.imdecode(npimg, 1)


def create_mask(frame: Any, hsv_low: Tuple[int], hsv_high: Tuple[int]) -> Any:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, hsv_low, hsv_high)


def constraint(value: float, min: float, max: float) -> float:
    return min if value < min else max if max < value else value


if __name__ == "__main__":
    q = Queue()
    record_recording = False

    def process_hsv(hsv, hsv_range):
        # hue = cv2.getTrackbarPos(TRACKBAR_HUE, WINDOW_SETTINGS)
        # saturation = cv2.getTrackbarPos(TRACKBAR_SATURATION, WINDOW_SETTINGS)
        # value = cv2.getTrackbarPos(TRACKBAR_VALUE, WINDOW_SETTINGS)
        (hue, saturation, value) = hsv
        (hue_range, saturation_range, value_range) = hsv_range

        # hue_range = cv2.getTrackbarPos(TRACKBAR_HUE_RANGE, WINDOW_SETTINGS)
        # saturation_range = cv2.getTrackbarPos(TRACKBAR_SATURATION_RANGE, WINDOW_SETTINGS)
        # value_range = cv2.getTrackbarPos(TRACKBAR_VALUE_RANGE, WINDOW_SETTINGS)

        if HSV_TRACKBAR_MODE == 1:
            hsv_min = (constraint(hue - hue_range, 0, 180), constraint(saturation -
                    saturation_range, 0, 255), constraint(value - value_range, 0, 255))
            hsv_max = (constraint(hue + hue_range, 0, 180), constraint(saturation +
                    saturation_range, 0, 255), constraint(value + value_range, 0, 255))
            return hsv_min, hsv_max
        else: return tuple(hsv), tuple(hsv_range)

    def tk_thread(q: Queue):
        window = tk.Tk()

        window.geometry("640x480")
        window.grid_columnconfigure(0, weight=1)
        [window.grid_rowconfigure(x, weight=1) for x in range(20)]

        # hsv_text = tk.Text()
        # hsv_text.grid(row=12, column=0)

        def update_value(type: int, value: Any):
            q.put([type, value])

        def close_program(code=0):
            update_value(QUEUE_CLOSE_WINDOW, code)
            exit(code)

        window.protocol("WM_DELETE_WINDOW", lambda: close_program())

        expo0_label = tk.Label(text="Cam 0 exposure")
        expo0_label.grid(row=13, column=0)
        expo0_autoexpo_switch_var = tk.BooleanVar()
        expo0_autoexpo_switch = tk.Checkbutton(variable=expo0_autoexpo_switch_var, indicatoron=False,
                                               text="Enable auto exposure", command=lambda: update_value(QUEUE_CAM0_AUTO_EXPO, None))
        expo0_autoexpo_switch.grid(row=14, column=0)
        expo0 = tk.Scale(window, from_=0, to=400, orient=tk.HORIZONTAL, command=lambda v: update_value(
            QUEUE_CAM0_EXPO, v) is expo0_autoexpo_switch_var.set(False), resolution=0.1)
        expo0.grid(row=15, column=0, sticky="EW")

        expo1_label = tk.Label(text="Cam 1 exposure")
        expo1_label.grid(row=16, column=0)
        expo1_autoexpo_switch_var = tk.BooleanVar()
        expo1_autoexpo_switch = tk.Checkbutton(variable=expo1_autoexpo_switch_var, indicatoron=False,
                                               text="Enable auto exposure", command=lambda: update_value(QUEUE_CAM1_AUTO_EXPO, None))
        expo1_autoexpo_switch.grid(row=17, column=0)
        expo1 = tk.Scale(window, from_=0, to=200, orient=tk.HORIZONTAL, command=lambda v: update_value(
            QUEUE_CAM1_EXPO, v) is expo1_autoexpo_switch_var.set(False), resolution=0.1)
        expo1.grid(row=18, column=0, sticky="EW")
        start_record_state = tk.StringVar(value="Start recording")
        
        def aaaaaaaa():
            global record_recording
            record_recording = not record_recording
            update_value(QUEUE_CAM0_SWITCH_RECORD, record_recording) 
            start_record_state.set("Start recording" if not record_recording else "Stop recording")
        start_record = tk.Button(
            textvariable=start_record_state, command=aaaaaaaa)
        start_record.grid(row=19, column=0, sticky="EW")

        hue_label = tk.Label(text="Hue" if HSV_TRACKBAR_MODE == 1 else "Hue min" if HSV_TRACKBAR_MODE == 2 else None)
        hue_label.grid(row=0, column=0)
        hue = tk.Scale(window, from_=0, to=180, orient=tk.HORIZONTAL,
                       command=lambda v: update_value(QUEUE_HSV_TYPE_HUE, v))
        hue.grid(row=1, column=0, sticky="EW")

        saturation_label = tk.Label(text="Saturation" if HSV_TRACKBAR_MODE == 1 else "Saturation min" if HSV_TRACKBAR_MODE == 2 else None)
        saturation_label.grid(row=2, column=0)
        saturation = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL,
                              command=lambda v: update_value(QUEUE_HSV_TYPE_SATURATION, v))
        saturation.grid(row=3, column=0, sticky="EW")

        value_label = tk.Label(text="Value" if HSV_TRACKBAR_MODE == 1 else "Value min" if HSV_TRACKBAR_MODE == 2 else None)
        value_label.grid(row=4, column=0)
        value = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL,
                         command=lambda v: update_value(QUEUE_HSV_TYPE_VALUE, v))
        value.grid(row=5, column=0, sticky="EW")

        hue_range_label = tk.Label(text="Hue range" if HSV_TRACKBAR_MODE == 1 else "Hue max" if HSV_TRACKBAR_MODE == 2 else None)
        hue_range_label.grid(row=6, column=0)
        hue_range = tk.Scale(window, from_=0, to=180, orient=tk.HORIZONTAL,
                             command=lambda v: update_value(QUEUE_HSV_TYPE_HUE_RANGE, v))
        hue_range.grid(row=7, column=0, sticky="EW")

        saturation_range_label = tk.Label(text="Saturation range" if HSV_TRACKBAR_MODE == 1 else "Saturation max" if HSV_TRACKBAR_MODE == 2 else None)
        saturation_range_label.grid(row=8, column=0)
        saturation_range = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL,
                                    command=lambda v: update_value(QUEUE_HSV_TYPE_SATURATION_RANGE, v))
        saturation_range.grid(row=9, column=0, sticky="EW")

        value_range_label = tk.Label(text="Value range" if HSV_TRACKBAR_MODE == 1 else "Value max" if HSV_TRACKBAR_MODE == 2 else None)
        value_range_label.grid(row=10, column=0)
        value_range = tk.Scale(window, from_=0, to=255, orient=tk.HORIZONTAL,
                               command=lambda v: update_value(QUEUE_HSV_TYPE_VALUE_RANGE, v))
        value_range.grid(row=11, column=0, sticky="EW")

        print_hsv = tk.Button(
            text="Print HSV", command=lambda: update_value(QUEUE_PRINT_HSV, True))
        print_hsv.grid(row=12, column=0, sticky="EW")

        window.mainloop()

    Thread(target=tk_thread, name="Tkinter thread", args=(q, )).start()
    
    fps = 0
    fps_counter = 0
    fps_next_check = 0
    recording_stage = None

    hsv_values, hsv_range = [0, 0, 0], [0, 0, 0]
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        hsv_server.send_msg(s, hsv_server.COMMA_GET_SETTINGS.encode('utf-8'))
        server_settings = json.loads(hsv_server.recv_msg(s).decode('utf-8'))
        server_version = server_settings[hsv_server.SETTING_NAME_VERSION]
        assert server_version == __version__, "Server version is not compatible!"
        if ENABLE_AUTO_FRAME_CONFIG:
            TRANSFER_MODE = server_settings[hsv_server.SETTING_NAME_TRANSFER_MODE]
            RESIZE_END_IMAGE_TO = server_settings[hsv_server.SETTING_NAME_FRAME_SIZE]
        while True:
            if time.time() >= fps_next_check:
                fps = fps_counter
                fps_counter = 0
                fps_next_check = time.time() + 1

            hsv_min, hsv_max = process_hsv(hsv_values, hsv_range)
            while not q.empty():
                type, value = q.get()
                if type == QUEUE_HSV_TYPE_HUE:
                    hsv_values[0] = int(value)
                elif type == QUEUE_HSV_TYPE_SATURATION:
                    hsv_values[1] = int(value)
                elif type == QUEUE_HSV_TYPE_VALUE:
                    hsv_values[2] = int(value)
                elif type == QUEUE_HSV_TYPE_HUE_RANGE:
                    hsv_range[0] = int(value)
                elif type == QUEUE_HSV_TYPE_SATURATION_RANGE:
                    hsv_range[1] = int(value)
                elif type == QUEUE_HSV_TYPE_VALUE_RANGE:
                    hsv_range[2] = int(value)
                elif type == QUEUE_PRINT_HSV:
                    print(
                        "HSV values: ({}, {}, {}), ({}, {}, {})".format(hsv_min[0], hsv_min[1], hsv_min[2], hsv_max[0], hsv_max[1], hsv_max[2]))
                elif type == QUEUE_CLOSE_WINDOW:
                    cv2.destroyAllWindows()
                    exit(0)
                elif type == QUEUE_CAM0_EXPO:
                    hsv_server.send_msg(
                        s, "{} 0 {}".format(hsv_server.COMMA_SET_EXPOSURE, value).encode('utf-8'))
                elif type == QUEUE_CAM1_EXPO:
                    hsv_server.send_msg(
                        s, "{} 1 {}".format(hsv_server.COMMA_SET_EXPOSURE, value).encode('utf-8'))
                elif type == QUEUE_CAM0_AUTO_EXPO:
                    hsv_server.send_msg(
                        s, "{} 0".format(hsv_server.COMMA_AUTO_EXPOSURE).encode('utf-8'))
                elif type == QUEUE_CAM1_AUTO_EXPO:
                    hsv_server.send_msg(
                        s, "{} 1".format(hsv_server.COMMA_AUTO_EXPOSURE).encode('utf-8'))
                elif type == QUEUE_CAM0_SWITCH_RECORD:
                    print(value)
                    if value:
                        if recording_stage is None:
                            recording_stage = 1
                    else:
                        recording_stage = 3
                q.task_done()

            frame = get_frame(s, 0)
            frame = cv2.resize(frame, RESIZE_END_IMAGE_TO)
            draw = frame.copy()
            h, w, c = frame.shape
            cv2.putText(draw, "FPS: {}".format(fps), (0, h - 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
            mask = create_mask(frame, hsv_min, hsv_max)
            if SHOW_CONTOURS:
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                if len(cnts) > 0:
                    cv2.drawContours(draw, cnts, -1, (0, 255, 0))
            cv2.imshow("frame", draw)
            cv2.imshow("mask", mask)

            if recording_stage == 1:
                # fourcc = cv2.CV_FOURCC(*'XVID')  # cv2.VideoWriter_fourcc() does not exist
                video_writer = cv2.VideoWriter("output.avi", None, 25, (w, h))
                recording_stage = 2

            if recording_stage == 2:
                video_writer.write(frame)

            if recording_stage == 3:
                video_writer.release()

            # frame2 = get_frame(s, 1)
            # frame2 = cv2.resize(frame2, RESIZE_END_IMAGE_TO)
            # draw2 = frame2.copy()
            # h, w, c = frame2.shape
            # cv2.putText(draw2, "FPS: {}".format(fps), (0, h - 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
            # mask2 = create_mask(frame2, hsv_min, hsv_max)
            # if SHOW_CONTOURS:
            #     cnts2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            #     if len(cnts2):
            #         cv2.drawContours(draw2, cnts2, -1, (0, 255, 0))
            # cv2.imshow("frame2", draw2)
            # def capture_pixel_frame2(event,x,y,flags,param):
            #     if event == cv2.EVENT_LBUTTONDOWN:
            #         hsv_pixel = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)[y, x, :]
            #         print("Pixel x: {} y: {} is ({}, {}, {})".format(x, y, hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]))
            # cv2.setMouseCallback("frame2", capture_pixel_frame2)
            # cv2.imshow("mask2", mask2)
            fps_counter += 1
            cv2.waitKey(1)
