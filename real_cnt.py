import pymurapi as mur
import cv2 as cv
import math
import time
import numpy as np

auv = mur.mur_init()
low_hsv = (0, 70, 70)
max_hsv = (180, 255, 255)

Kp_depth = 0.5
Kd_depth = 0.5
Kp_yaw = 0.5
Kd_yaw = 0.4
time_new = 0


class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __itit__(self):
        pass

    def set_p_gain(self, value):
        self._kp = value

    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return output
        


def clamp_to180(angle):
    if angle > 180:
        return angle - 360
    if angle < -180:
        return angle + 360
    return angle


def clamp_to90(angle):
    if angle > 90:
        return angle - 180
    if angle < -90:
        return angle + 180
    return angle


def clamp(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v


def find_contour(img, low_hsv, max_hsv):
    imageHSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    img_bin = cv.inRange(imageHSV, low_hsv, max_hsv)
    cont, _ = cv.findContours(img_bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    return cont

def thresholding_find_contours(img, maxValue, blockSize, C):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    return cv.adaptiveThreshold(gray, maxValue, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, blockSize, C)


def keep_yaw(set_yaw):
    try:
        error = auv.get_yaw() - set_yaw
        error = clamp_to180(error)
        output = keep_yaw.regulator.process(error)
        output = clamp(output, 100, -100)
        auv.set_motor_power(1, output)
        auv.set_motor_power(0, -output)
        return error
    except AttributeError:
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(0.5)
        keep_yaw.regulator.set_d_gain(0.5)
    return False


def keep_depth(depth_to_set, speed_to_depth=1.0):
    try:
        try:
            error = auv.get_depth() - depth_to_set
            output = keep_depth.regulator.process(error)
            output = clamp(output * speed_to_depth, 100, -100)
            auv.set_motor_power(2, output)
            auv.set_motor_power(3, output)
        except ZeroDivisionError:
            time.sleep(0.001)
            error = auv.get_depth() - depth_to_set
            output = keep_depth.regulator.process(error)
            output = clamp(output * speed_to_depth, 100, -100)
            auv.set_motor_power(2, output)
            auv.set_motor_power(3, output)
    except AttributeError:
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)
        keep_depth.regulator.set_d_gain(Kd_depth)


def stable_yaw(set_yaw, stable_yaw, accuracy=1, stab_time=3):
    # стабилизация по углу центра контура
    global time_new
    try:
        error = set_yaw - stable_yaw
        output = keep_yaw.regulator.process(error)
        output = clamp(output, 100, -100)
        # if abs(error) > accuracy:
        auv.set_motor_power(1, -output)
        auv.set_motor_power(0, output)
        #     time_new = time.time()
        # if stab_time < time.time() - time_new:
        #     return True
        return error
    except AttributeError:
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(0.3)
        keep_yaw.regulator.set_d_gain(0.4)
    return False


def stable_y(y_center, speed=1, accuracy=0.3, stab_time=2):
    # стабилизация по глубине центра контура
    global time_new
    try:
        error = y_center
        output = stab_on_.regulator.process(error)
        output = clamp(output * speed, 100, -100)
        # if abs(error ** 2) > accuracy:
        auv.set_motor_power(3, -output)
        auv.set_motor_power(2, -output)
        # if stab_time < time.time() - time_new:
        #     print("stabled")
        #     return True
        return error
    except AttributeError:
        stab_on_.regulator = PD()
        stab_on_.regulator.set_p_gain(Kp_depth)
        stab_on_.regulator.set_d_gain(Kd_depth)
    return False


def stab_on_area(contour, area):  # стабилизация по площади контура
    try:
        error = cv.contourArea(contour) - area
        output = stab_on_area.regulator.process(error)
        output = clamp(output, 100, -100)
        auv.set_motor_power(1, -output)
        auv.set_motor_power(0, -output)
        return error
    except AttributeError:
        stab_on_area.regulator = PD()
        stab_on_area.regulator.set_p_gain(0.2)
        stab_on_area.regulator.set_d_gain(0.2)
    return False


def stab_on_(curve, accuracy=3.0, speed=1, stab_time=3):
    # стабилизация на фигуре: по глубине и по углу
    global time_new
    try:
        moments = cv.moments(curve)
        x = int(moments["m10"] / moments["m00"])
        y = int(moments["m01"] / moments["m00"])
        x_center = x - (320 / 2)
        y_center = y - (240 / 2)
        # tan_a = x_center / y_center
        # sin_a = x_center / ((x_center ** 2 + y_center ** 2) ** 0.5)
        # angle = math.asin(sin_a) * 180 / math.pi
        stable_yaw(x, 160)
        stable_y(y_center, speed)

    except ZeroDivisionError:
        return False
    return False


def define_form(cont):
    # опеределение формы
    if abs(cv.contourArea(cont)) > 100:
        hull = cv.convexHull(cont)
        # сглаженный контур - без потерь (нужно ли?)
        apr_shape = cv.approxPolyDP(hull, cv.arcLength(hull, True) * 0.04, True)
        #
        # аппроксимированная фигура
        if len(apr_shape) == 3:
            form = "triangle"

            return form, apr_shape
        elif len(apr_shape) == 4:
            form = "square"
            return form, apr_shape
        else:
            form = "undefined"
            return form, apr_shape
    else:
        form = "none"
        return form, cont


def define_course(form, curve, img):  # определение угла между текущим курсом и направлением стрелки внизу
    if (form == "triangle"):
        _, points = cv.minEnclosingTriangle(curve)
        # points=curve
        w1 = round(((points[0, 0, 1] - points[1, 0, 1]) ** 2 + (points[0, 0, 0] - points[1, 0, 0]) ** 2) ** 0.5)
        w2 = round(((points[1, 0, 1] - points[2, 0, 1]) ** 2 + (points[1, 0, 0] - points[2, 0, 0]) ** 2) ** 0.5)
        w3 = round(((points[2, 0, 1] - points[0, 0, 1]) ** 2 + (points[2, 0, 0] - points[0, 0, 0]) ** 2) ** 0.5)
        # print(points[0,0,1])
        cv.circle(img, (points[0, 0, 0], points[0, 0, 1]), 10, (0, 0, 255), 3)
        cv.circle(img, (points[1, 0, 0], points[1, 0, 1]), 10, (0, 255, 0), 3)
        cv.circle(img, (points[2, 0, 0], points[2, 0, 1]), 10, (255, 0, 255), 3)
        x_1 = points[0, 0, 0]
        y_1 = points[0, 0, 1]
        x_2 = points[1, 0, 0]
        y_2 = points[1, 0, 1]
        x_3 = points[2, 0, 0]
        y_3 = points[2, 0, 1]
        if w2 < w1 > w3:
            x_cent = int((x_1 + x_2) / 2)  # координаты центра длинной стороны
            y_cent = int((y_1 + y_2) / 2)
            cv.circle(img, (x_cent, y_cent), 10, (0, 255, 0), 3)
            x, y = x_3, y_3
        elif w1 < w2 > w3:
            x_cent = round((x_2 + x_3) / 2)  # координаты центра длинной стороны
            y_cent = round((y_2 + y_3) / 2)
            x, y = x_1, y_1
        elif w2 < w3 > w1:
            x_cent = round((x_1 + x_3) / 2)  # координаты центра длинной стороны
            y_cent = round((y_1 + y_3) / 2)
            x, y = x_2, y_2
        length_vector = ((x - x_cent) ** 2 + (y - y_cent) ** 2) ** 0.5
        vector_coord = ((x - x_cent), (y - y_cent))
        cos_a = (int(vector_coord[0]) * 320) / (int(length_vector * 320))
        angle = int(math.acos(cos_a) * 180 / math.pi)
        print(str(angle))
        return angle
    else:
        return 0


def defining_figure(cnt):
    c = cv.convexHull(cnt)  # сглаживаем фигуру
    # аппроксимируем фигуры, (!!!)
    # angle_arrow = cv2.approxPolyDP(c, cv2.arcLength(c, True) * 0.15, True)

    # рисуем треугольник, получаем площадь
    s_triangle, _ = cv.minEnclosingTriangle(c)
    # рисуем прямоугольник, получаем x,y верхнего левого угла,
    # высоту и длину и угол  поворота
    ((_, _), (w, h), _) = cv.minAreaRect(c)
    #  возвращает координаты центра x, y и радиус (строит круг)
    _, radius = cv.minEnclosingCircle(c)

    # вычесление площадей кргуа, квадрата, треугольника
    rectangle_area = w * h
    circle_area = radius ** 2 * math.pi

    # вычесление центра фигуры
    moments = cv.moments(c)  # получение моментов
    try:
        x = int(moments["m10"] / moments["m00"])  # центр
        y = int(moments["m01"] / moments["m00"])
    except ZeroDivisionError:
        return 0, 0, "none"

    # print("Triangle:", s_triangle, "Circle:", circle_area)
    if rectangle_area > circle_area < s_triangle:
        return x, y, "circle"
    elif circle_area > rectangle_area < s_triangle:
        return x, y, "square"
    elif rectangle_area > s_triangle < circle_area:
        return x, y, "triangle"



time_flag = time.time()
initial_yaw = auv.get_yaw()
while (time.time()-time_flag)<5:
    time.sleep(0.001)
time_flag = time.time()

passed = []

time_flag = time.time()
while True:
    try:
        error = keep_yaw(-90)
        # print(error)
        if abs(error) > 5:
            time_flag = time.time()
        if (time.time() - time_flag) > 4:
            print("180")
            break   
    except ZeroDivisionError: pass
    img = auv.get_image_front()
    cv.imshow("img", img)

auv.set_motor_power(2, -20)
auv.set_motor_power(3, -20)
auv.set_motor_power(0, -10)
auv.set_motor_power(1, -10)
time.sleep(3)

time_flag = None

while True:
    stabbing = False
    find_new_shape = False
    set_shape = False
    while True:
        # print("iter")
        img = auv.get_image_front()
        draw = img.copy()
        super_mega_break = False
        coord = []  # список контуров для сортировки
        
        for (low_hsv, max_hsv, color) in [
                # (low_hsv, max_hsv, "blue"),    
                ((122, 149, 148), (146, 251, 255), "blue"),
                ((70, 172, 183), (77, 222, 255), "green"),
                # ((4, 144, 199), (40, 255, 245), "orange")

            ]:
            cnt = find_contour(img, low_hsv, max_hsv)  # подбирать для заданных цветов
            # может стоит добавить выборку, здесь используются цвета вне фона.
            
            if cnt:
#                print("cnt")
                for c in cnt:
                    if abs(cv.contourArea(c)) > 500:
                        form, points = define_form(c)
                        cont = cv.convexHull(c)
                        try:
                            moments = cv.moments(points)
                            x = int(moments["m10"] / moments["m00"])
                            y = int(moments["m01"] / moments["m00"])
                            f = defining_figure(cont)
                            # print(f, color)
                            # if not (f, color) in passed:
                            shape = f[2] if f is not None else None
                            if not find_new_shape:
                                coord.append([x, y, cv.contourArea(cont), cont, f, color]) # составляется список всех контуров
                            
                            elif (shape, color) not in passed:
                                coord.append([x, y, cv.contourArea(cont), cont, f, color])
                            # в пределах работы камеры
                            (x, y, w, h) = cv.boundingRect(cont)
                            cv.rectangle(draw, (x, y), (x + w, y + h), (255, 0, 0))
                            cv.putText(draw, f[2] + ' ' + color, (x, y), cv.FONT_HERSHEY_COMPLEX_SMALL, 2, (255, 0, 0))
                        
                        except ZeroDivisionError:
                            time.sleep(0.001)
                        cv.drawContours(draw, cont, -1, (255, 0, 0), 4)
                        
                try:
                    if len(coord) == 0:  # если контура не найдены, то поиск в области слева начинается
                        # далее просто по кругу поворот
                        if not stabbing:
                            if len(passed) != 0:
                                print('counterclockwise')
                                auv.set_motor_power(1,15)
                            else:
                                print('clockwise')
                                auv.set_motor_power(0,15)
                        continue
                    # цикл стабилизации стар
                    # coord = list(filter(lambda x:  not in passed, coord))
                    coord.sort()
                    cord_left = coord[0]

                    print(passed, (cord_left[4][2], cord_left[5])) 
                    
                    if (cord_left[4][2], cord_left[5]) in passed or len(passed) == 0 or find_new_shape or set_shape:
                        if len(passed) == 0 or find_new_shape: 
                            cord_left = coord[0]
                            print("current", cord_left[4], cord_left[5])
                        else: 
                            find_new_shape = True
                            print("Finding new shape!")
                            continue
                            # if len(coord) <= 1: 
                            #     print("Too malo figures! Turning clockwise...")
                            #     auv.set_motor_power(0, 10)
                            #     auv.set_motor_power(1, -10)
                            #     time.sleep(1)
                            #     continue
                            # print(len(coord))
                            # coord2 = list(filter(lambda x: x[4][2] != cord_left[4][2] and x[5] != cord_left[5], coord))
                            # if len(coord2) == 0:
                            #     print("Next contour not found! Turning clockwise...")
                            #     auv.set_motor_power(0, 10)
                            #     auv.set_motor_power(1, -10)
                            #     time.sleep(1)
                            #     continue
                            # print("second", cord_left[4], cord_left[5])

                        if find_new_shape:
                            auv.set_motor_power(0, 10)
                            auv.set_motor_power(1, 10)
                            time.sleep(1)
                            find_new_shape = False
                            set_shape = True
                            
                        cord_x = cord_left[0]
                        cord_y = cord_left[1]
                        area = cord_left[2]
                        cv.circle(draw, (cord_x, cord_y), 5, (255, 0, 255), 2)
                        error_yaw = stable_yaw(cord_x, 160)
                        error_depth = stable_y(cord_y - 240 / 2, 1)
                        
                        print(error_yaw, error_depth)
                        # time.sleep(0.1)
                        # print(time_flag)
                        if time_flag is None:
                            time_flag = time.time()
                            stabbing = True
                        elif not((abs(error_depth) < 5) and (abs(error_yaw) < 5)):
                            print("stabbing...")
                            time_flag = time.time()
                            stabbing = True
                        elif (time.time() - time_flag) > 5:
                            print("done")
                            super_mega_break = True
                            stabbing = False
                            break   
                        secondary_yaw = auv.get_yaw()
                        # stab_on_area(cord_left[3], 1500)
                        # print(len(coord))
                except ValueError:
                    pass
            else: 
                # if len(passed) != 0:
                #     print("CLOCLWISE")
                #     auv.set_motor_power(1, -50)
                #     auv.set_motor_power(0, 50)
                # else:
                #     print("COUNTERCLOCKWISE")
                if not stabbing:
                    auv.set_motor_power(1, 50)
                    auv.set_motor_power(0, -50)
                # pass
                
        if super_mega_break: break

        # keep_depth(1)
        cv.imshow("draw", draw)
        thresh = thresholding_find_contours(img, 255, 255, 16)
        cv.imshow("thresh", thresh)
        cv.waitKey(1)

    # 
    # цикл по времени и ошибке
    # определить цвет и фигуру следующей таблички
    # keep_yaw(initial_yaw)
    print("Detecting shape...")
    cord_left = coord[0]
    f = cord_left[4][2]
    color = cord_left[5]
    passed.append((f, color))
    print(f)

    if f == "circle":
        last_time = time.time()
        d = auv.get_depth()
        area = cv.contourArea(cord_left[3])
        while(time.time() - last_time < 5):
            keep_depth(d)
            auv.set_motor_power(1, 50)
            auv.set_motor_power(0, 50)
        print("forwarded")
        auv.set_motor_power(1, -50)
        auv.set_motor_power(0, -50)
        time.sleep(3)
        print("backwarded")

    elif f == "square":
        pass
    elif f == "triangle":
        # auv.set_motor_power(1, -50)
        # auv.set_motor_power(0, 50)
        # time.sleep(2)
        ...
    elif f == "none": #звезда
        ...

    next_time = time.time()
    d = auv.get_depth()
    while True:
        try:
            err = keep_yaw(0)
            err_depth=keep_depth(d)
            # print(err)
            if abs(err)>5:
                next_time = time.time()
            if next_time + 4 < time.time():
                break
        except ZeroDivisionError: pass
        time.sleep(0.001)

    print("next!")
    print(passed)

    # betha = 90 - abs(initial_yaw - secondary_yaw)
    auv.set_motor_power(0, 50)
    auv.set_motor_power(1, 50)
    move = 2
    print("move for", move, "seconds...")
    time.sleep(move)

    # auv.set_motor_power(1, -50)
    # auv.set_motor_power(0, 50)
    # time.sleep(1)
    auv.set_motor_power(1, 0)
    auv.set_motor_power(0, 0)

# движение вперёд по времени: t = Kt*betha

# цикл стабилизации (по всем параметрам и площади) и проверка betha ~ 0
# определение фигуры и выполнение задания
# initial_yaw и ищется 2й контур слева или можно запомнить цвет/фигуру  ??? как отделить 2ю фигуру от 1й