__author__ = "Edventy <edventyh@gmail.com>"
__version__ = 0.2

##################################################################
# Video server with binary hsv filter and camera exposure setter #
# Works with 2 cameras with ids 0 and 1                          #
# Server side                                                    #
##################################################################

######### Config #########
TRANSFER_MODE = 2 # 1 - pickle, 2 - imcode !SERVER MODE AND CLIENT MODE MUST BE THE SAME!
RESIZE_TRANSFER_FRAME_TO = (640, 480) #pixels. Frame while sending to a client will be resized to.

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

HOST = ''
PORT = 65432
####### End config #######

import socket
import struct
import cv2
import pickle
import base64
import json

COMMA_GET_FRAME = "comma-get-frame"
COMMA_SET_EXPOSURE = "comma-set-exposure"
COMMA_AUTO_EXPOSURE = "comma-auto-exposure"
COMMA_GET_SETTINGS = "comma-get-version"

SETTING_NAME_TRANSFER_MODE = "transfer-mode"
SETTING_NAME_FRAME_SIZE = "frame_size"
SETTING_NAME_VERSION = "version"


def split_data(data: str): 
    comma, argument = None, None
    splited = data.split(' ', 1)
    if len(splited) == 1:
        comma = splited[0]
    elif len(splited) == 2:
        comma, argument = splited[0], splited[1]
    return (comma, argument)


def send_msg(sock, msg):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendall(msg)


def recv_msg(sock):
    # Read message length and unpack it into an integer
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(sock, msglen)


def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


if __name__ == "__main__":
    cap0 = cv2.VideoCapture(0)
    # cap1 = cv2.VideoCapture(1)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        while True:
            try:
                conn, addr = s.accept()
                with conn:
                    print('Connected by', addr)
                    while True:
                        data = recv_msg(conn)
                        if not data:
                            break
                        text = data.decode("utf-8")
                        comma, arg = split_data(text)
                        if comma == COMMA_GET_FRAME:
                            frame = None
                            if arg is not None:
                                cam_num = int(arg)
                                if cam_num == 0:
                                    r, frame = cap0.read()
                                # elif cam_num == 1:
                                #     r, frame = cap1.read()
                            frame = cv2.resize(
                                frame, RESIZE_TRANSFER_FRAME_TO)
                            if TRANSFER_MODE == 1:
                                send_msg(conn, pickle.dumps(frame))
                            elif TRANSFER_MODE == 2:
                                encoded, buffer = cv2.imencode('.jpg', frame)
                                send_msg(conn, base64.b64encode(buffer))
                        # elif comma == COMMA_SET_EXPOSURE:
                        #     splited_arg = arg.split(' ')
                        #     cam_num, expo = int(
                        #         splited_arg[0]), float(splited_arg[1])
                        #     if arg is not None:
                        #         if cam_num == 0:
                        #             cap0.set(cv2.CAP_PROP_EXPOSURE, expo)
                        #             print("camera 0 exposure changed to", expo)
                        #         elif cam_num == 1:
                        #             cap1.set(cv2.CAP_PROP_EXPOSURE, expo)
                        #             print("camera 1 exposure changed to", expo)
                        # elif comma == COMMA_AUTO_EXPOSURE:
                        #     if arg is not None:
                        #         cam_num = int(arg)
                        #         if cam_num == 0:
                        #             cap0.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                        #             print("camera 0 auto exposure on")
                        #         elif cam_num == 1:
                        #             cap1.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                        #             print("camera 1 auto exposure on")
                        elif comma == COMMA_GET_SETTINGS:
                            print("sending server settings")
                            send_msg(conn, json.dumps({
                                SETTING_NAME_FRAME_SIZE: (CAMERA_WIDTH, CAMERA_HEIGHT),
                                SETTING_NAME_TRANSFER_MODE: TRANSFER_MODE,
                                SETTING_NAME_VERSION: __version__
                            }).encode('utf-8'))
            except Exception as err:
                print("Connection error occured! Message:", err)
                
