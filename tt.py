import cv2
import numpy as np
import aboba2

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}

def find_aruco(frame, draw_frame):
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_4X4_50"])
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
	parameters=arucoParams)
    # cv2.aruco.drawDetectedMarkers(draw_frame, corners, ids)
    data = []
    if len(corners) > 0:
        # print(corners)
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            # print("[INFO] ArUco marker ID: {}".format(markerID))
            area = cv2.contourArea(markerCorner)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(draw_frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(draw_frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(draw_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(draw_frame, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(draw_frame, (cX, cY), 4, (0, 0, 255), -1)
            data.append((
                markerID,
                (topLeft, topRight, bottomRight, bottomLeft),
                (cX, cY),
                area
            ))
    return data

cap = cv2.VideoCapture(1)
hsv_min = (0, 0, 0)
hsv_max = (255, 255, 255)
huess = []
saturationss = []
valuess = []
while True:
    r, frame = cap.read()
    # frame = frame[60:420, :, :]
    frame = cv2.resize(frame, (frame.shape[1] //2, frame.shape[0] // 2))
    # cv2.GaussianBlur()
    draw = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    c0 = None
    c1 = None
    c2 = None
    c3 = None
    for id, (tl, tr, br, bl), (cX, cY), area in find_aruco(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), draw):
        if id == 0:
            c0 = (cX, cY)
        elif id == 1:
            c1 = (cX, cY)
        elif id == 2:
            c2 = (cX, cY)
        elif id == 3:
            c3 = (cX, cY)
    if c0 is not None and c1 is not None and c2 is not None and c3 is not None:
        # print("all")
        top_center = aboba2.Maath.vectors2_two_center(c0, c1)
        cv2.circle(draw, top_center, 3, (255, 0, 0), 3)
        right_center = aboba2.Maath.vectors2_two_center(c1, c3)
        cv2.circle(draw, right_center, 3, (255, 0, 0), 3)
        left_center = aboba2.Maath.vectors2_two_center(c0, c2)
        cv2.circle(draw, left_center, 3, (255, 0, 0), 3)
        bottom_center = aboba2.Maath.vectors2_two_center(c2, c3)
        cv2.circle(draw, bottom_center, 3, (255, 0, 0), 3)
        center = aboba2.Maath.vectors2_square_center(c0, c1, c2, c3)
        cv2.circle(draw, center, 3, (255, 0, 0), 3)
        hues = [
            hsv[top_center[1], top_center[0], [0]],
            hsv[left_center[1], left_center[0], [0]],
            hsv[right_center[1], right_center[0], [0]],
            hsv[bottom_center[1], bottom_center[0], [0]],
        ]
        saturations = [
            hsv[top_center[1], top_center[0], [1]],
            hsv[left_center[1], left_center[0], [1]],
            hsv[right_center[1], right_center[0], [1]],
            hsv[bottom_center[1], bottom_center[0], [1]],
        ]
        values = [
            hsv[top_center[1], top_center[0], [2]],
            hsv[left_center[1], left_center[0], [2]],
            hsv[right_center[1], right_center[0], [2]],
            hsv[bottom_center[1], bottom_center[0], [2]],
        ]
        huess.extend(hues)
        saturationss.extend(saturations)
        valuess.extend(values)
        # hsv_min = (int(min(hues)) - 10, int(min(saturations)) - 10, int(min(values)) - 10)
        # hsv_max = (int(max(hues)) + 10, int(max(saturations)) + 10, int(max(values)) + 10)
            # top_center = tuple([int(i) for i in aboba2.Maath.vectors2_two_center(tl, tr)])
            # cv2.circle(draw, top_center, 3, (255, 0, 0), 3)
            # # x, y = (cX + top_center[0]) // 2, (cY + top_center[1]) // 2
            # # cv2.circle(draw, (x, y), 3, (0, 255, 0), 3) 
            # # print(x, y)
            
            # left_center = tuple([int(i) for i in aboba2.Maath.vectors2_two_center(tl, bl)])
            # cv2.circle(draw, left_center, 3, (255, 0, 0), 3)

            # bottom_center = tuple([int(i) for i in aboba2.Maath.vectors2_two_center(bl, br)])
            # cv2.circle(draw, bottom_center, 3, (255, 0, 0), 3)

            # right_center = tuple([int(i) for i in aboba2.Maath.vectors2_two_center(tr, br)])
            # cv2.circle(draw, right_center, 3, (255, 0, 0), 3)

            # x = int(cX - 15 - area / 60)
            # # print(area, x)
            # cv2.circle(draw, (x, cY), 3, (0, 255, 0), 3)
            # # print(hsv[cY, x, :])
            # h, s, v = hsv[cY, x, :]
            # h, s, v = int(h), int(s), int(v)
            # hsv_min = (h - 10, s - 30, v - 30)
            # hsv_max = (h + 10, s + 30, v + 30)
    cv2.rectangle(frame, (0, 60), (640, 420), (255, 0, 0))
    print(frame.shape)
    avg_color_per_row = np.average(frame, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)
    draw -= np.full(draw.shape, avg_color).astype('uint8') 
    cv2.imshow("frame", frame)
    print(hsv_min, hsv_max)
    k = cv2.waitKey(1)
    if k == 13:
        hsv_min = (int(min(huess)) - 10, int(min(saturationss)) - 20, int(min(valuess)) - 20)
        hsv_max = (int(max(huess)) + 10, int(max(saturationss)) + 20, int(max(valuess)) + 20)
        print(hsv_min, hsv_max)
        huess = []
        saturationss = []
        valuess = []
    bin = cv2.inRange(hsv, hsv_min, hsv_max)
    
    cv2.imshow("mask", bin)
    cv2.imshow("draw", draw)
    