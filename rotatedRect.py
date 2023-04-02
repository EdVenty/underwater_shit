import time
from typing import Tuple, Union
import cv2
import threading
import pymurapi as mur
import sched
import numpy as np
import math

auv = mur.mur_init()
time_new = 0
HSV_MIN = (10, 120, 20)
HSV_MAX = (60, 255, 255)
THRUSTER_DEPTH_LEFT = 2
THRUSTER_DEPTH_RIGHT = 3
THRUSTER_YAW_LEFT = 0
THRUSTER_YAW_RIGHT = 1

DIRECTION_THRUSTER_DEPTH_LEFT = +1
DIRECTION_THRUSTER_DEPTH_RIGHT = +1
DIRECTION_THRUSTER_YAW_LEFT = +1
DIRECTION_THRUSTER_YAW_RIGHT = +1

Kp_depth = 15  # кофецент пропорционального регулятора на глубину
Kd_depth = 0.15  # кофецент дифференциального регулятора на глубину
Kp_yaw = 0.1  # кофецент пропорционального регулятора на курс
Kd_yaw = 0.0001 # кофецент дифференциального регулятора на курс


def sort_array(array, column=0):
    array_sort = []
    for i in array:
        array_ = []
        for n in i:
            array_.append(n)
        array_sort.append(array_)
    array_sort.sort(key=lambda x: x[column])
    return array_sort


def calculate_angle_cos(x1, y1):
    x2 = 0 - 0
    y2 = 0 - 240
    # вычесляем угол
    angle = int(math.acos((x1 * x2 + y1 * y2) /
                          (((x1 ** 2 + y1 ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
    return angle

def calculate_angle_sin_vector(x1, y1):
    # вычесляем угол
    x2 = 0 - 0
    y2 = 0 - 240
    angle = int(math.asin((x1 * y2 - y1 * x2) /
                          (((x1 ** 2 + y1 ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
    return angle

def defining_arrow(image):  # вычесление угла у стрелки путём нахождения наибольшой стороны
    # image = auv.get_image_bottom()  # получение изображения с донной камеры в RGB формате.
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
    img_bin = cv2.inRange(imageHSV, HSV_MIN, HSV_MAX)  # бинаризация изображения.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # выделение контуров.
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[0]  # берём наибольшую маску
        if cv2.contourArea(c) > 500:  # проверяем не мусор ли это
            # c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            # рисуем треугольник, получаем площад
            s_triangle, angle_arrow = cv2.minEnclosingTriangle(c)
            #            angle_arrow = cv2.approxPolyDP(s_triangle, cv2.arcLength(s_triangle, True) * 0.04, True)

            # print(len(angle_arrow), "@@")
            # рисуем параллелограмм, получаем высоту и длину
            # ((_, _), (w, h), _) = cv2.minAreaRect(c)
            # проверка на то, что аппроксимировалась фигура по трём точкам
            # проверяем, что площадь треугольника меньше параллелипида, нарисованного вокруг фигуры
            if len(angle_arrow) != 3:
                return False
            #            if not (s_triangle < w * h):
            #                return False
            moments = cv2.moments(c)  # получение моментов
            # нахождение наибольшой стороны.
            #            W1 = (((angle_arrow[1, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
            #                  + ((angle_arrow[1, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5
            #
            #            W2 = (((angle_arrow[0, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
            #                  + ((angle_arrow[0, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5
            #
            #            W3 = (((angle_arrow[1, 0, 0] - angle_arrow[0, 0, 0]) ** 2)
            #                  + ((angle_arrow[1, 0, 1] - angle_arrow[0, 0, 1]) ** 2)) ** 0.5

            W1 = (((angle_arrow[1, 0, 0] - angle_arrow[0, 0, 0]) ** 2)
                  + ((angle_arrow[1, 0, 1] - angle_arrow[0, 0, 1]) ** 2)) ** 0.5 + \
                 (((angle_arrow[2, 0, 0] - angle_arrow[0, 0, 0]) ** 2)
                  + ((angle_arrow[2, 0, 1] - angle_arrow[0, 0, 1]) ** 2)) ** 0.5

            W2 = (((angle_arrow[0, 0, 0] - angle_arrow[1, 0, 0]) ** 2)
                  + ((angle_arrow[0, 0, 1] - angle_arrow[1, 0, 1]) ** 2)) ** 0.5 + \
                 (((angle_arrow[2, 0, 0] - angle_arrow[1, 0, 0]) ** 2)
                  + ((angle_arrow[2, 0, 1] - angle_arrow[1, 0, 1]) ** 2)) ** 0.5

            W3 = (((angle_arrow[1, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
                  + ((angle_arrow[1, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5 + \
                 (((angle_arrow[0, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
                  + ((angle_arrow[0, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5

            if W3 > W1 < W2:
                coordinte = [[angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]],
                             [angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]],
                             [angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]]]
            elif W3 > W2 < W1:
                coordinte = [[angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]],
                             [angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]],
                             [angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]]]
            elif W1 < W3 > W2:
                coordinte = [[angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]],
                             [angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]],
                             [angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]]]
            else:
                return False

            # вычисдения кооринат середины наибольшой стороны треугольника
            x_centre_arrow = (coordinte[1][0] + coordinte[2][0]) // 2
            y_centre_arrow = (coordinte[1][1] + coordinte[2][1]) // 2
            # вычесление векторов для расчёта угла стрелки
            x1 = coordinte[0][0] - x_centre_arrow
            y1 = coordinte[0][1] - y_centre_arrow
            # вычесляем угол
            # angle = calculate_angle_cos(x1, y1)
            angle = calculate_angle_sin_vector(x1, y1)
            # вычесляем знак угла1
            # print(angle)
            # angle = angle if coordinte[1][1] < coordinte[2][1] else -angle
            try:
                x = int(moments["m10"] / moments["m00"])  # координаты центра стрелки
                y = int(moments["m01"] / moments["m00"])
                return x, y, angle + int(auv.get_yaw())  # данные угла для регулятора по курсу
            except:
                return False
    return False

# вычесление угла полоски, работает при отклонение до 60. (надо доработать)
def defining_line(image: np.ndarray, convert_to_a_yaw_regulator_data: bool = False) -> Union[Tuple[int, int, int], Tuple[None, None, None]]:
    """Calculates angle of stripe. Works with a deviation up to 60 degrees.

    Args:
        image (np.ndarray): Cv2 frame.
        convert_to_a_yaw_regulator_data (bool, optional): Convert angle to a data to process with a regulator. Defaults to False.

    Returns:
        Union[Tuple[int, int, int], Tuple[None, None, None]]: Tuple of 3 numbers: 1)x center; 2)y center; 3)angle;
    """
    imageHSV = cv2.cvtColor(
        image, cv2.COLOR_BGR2HSV)  # перевод изображения из RGB в HSV формат.
    # бинаризация изображения.
    img_bin = cv2.inRange(imageHSV, HSV_MIN, HSV_MAX)
    # выделение контуров.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[
            0]  # берём наибольшую маску
        if cv2.contourArea(c) > 100:  # проверяем не мусор ли это
            c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            point_figure = cv2.approxPolyDP(
                c, cv2.arcLength(c, True) * 0.07, True)
            # рисуем треугольник, получаем площадь
            s_triangle, _ = cv2.minEnclosingTriangle(c)
            # рисуем параллелограмм, получаем высоту и длину
            ((_, _), (w, h), _) = cv2.minAreaRect(c)
            # проверка на то, что аппроксимировалась фигура по трём точкам
            # проверяем, что площадь треугольника меньше параллелипида, нарисованного вокруг фигуры
            if len(point_figure) != 4:
                return None, None, None
            if s_triangle < w * h:
                return None, None, None
            moments = cv2.moments(c)  # получение моментов
            # нахождение наибольшой стороны.
            point_figure = point_figure[:, 0, :]
            w = sort_array(point_figure, 1)
            w1 = sort_array(w[0:2])
            w2 = sort_array(w[2:4])

            # вычесление векторов для расчёта угла полоски
            x1 = w1[0][0] - w2[0][0]
            y1 = w1[0][1] - w2[0][1]
            # вычесляем угол
            angle = calculate_angle_cos(x1, y1)
            # вычесляем знак угла
            angle = angle if w1[0][0] < w2[0][0] else -angle
            # координаты центра стрелки
            x = int(moments["m10"] / moments["m00"])
            y = int(moments["m01"] / moments["m00"])
            if convert_to_a_yaw_regulator_data:
                # данные угла для регулятора по курсу
                return x, y, angle + int(auv.get_yaw())
            return x, y, angle
    return None, None, None

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
        timestamp = int(round(time.time() * 1000))  # в timestamp записываем
        # время(выраженное в секундах) и домножаем до милисекунд, round отбрасывает знаки после запятой
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        # вычесляем выходное значение на моторы по ПД регулятору и записываем в output
        self._timestamp = timestamp  # перезаписываем время
        self._prev_error = error  # перезаписываем ошибку
        return output


def clamp_to180(angle):  # ограничитель максимального зачения для курса
    if angle > 180:
        return angle - 360
    if angle < -180:
        return angle + 360

    return angle


def clamp(v, max_v, min_v):  # ограничитель максимального зачения для моторов
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v


def get_depth_correction(k=1):
    return auv.get_depth() * k

def keep_yaw(yaw_to_set, speed_to_yaw=0, error=...):  # ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
    try:
        time.sleep(0.001)
        if error is ...:
            error = yaw_to_set - auv.get_yaw()  # вычесление ошибки, действительное значение - заданное значение
        error = clamp_to180(error)  # проверяем ошибку на ограничение
        output = keep_yaw.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
        output = clamp(output, 100, -100)  # проверяем выходное значение на ограничение
        auv.set_motor_power(THRUSTER_YAW_LEFT, DIRECTION_THRUSTER_YAW_LEFT * clamp((speed_to_yaw - output), 100,
                                                                                   -100))  # передаём выходное значение на мотор 0
        auv.set_motor_power(THRUSTER_YAW_RIGHT, DIRECTION_THRUSTER_YAW_RIGHT * clamp((speed_to_yaw + output), 100,
                                                                                     -100))  # передаём выходное значение на мотор 1
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(Kp_yaw)  # запись пк на курс
        keep_yaw.regulator.set_d_gain(Kd_yaw)  # запись дк на курс

def keep_depth(depth_to_set, depth_SECRET=True):  # ПД регулятор по глубине, !!! без ожидания будет выдавать ошибку !!!
    speed_to_depth = 1
    try:
        time.sleep(0.001)
        error = get_depth_correction() - depth_to_set  # вычесление ошибки, действительное значение - заданное значение
        output = keep_depth.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
        output += 10
        if depth_SECRET:
            output = clamp(output * speed_to_depth, 30, -30)  # проверяем выходное значение на
        else:
            output = clamp(output * speed_to_depth, 100, -100)  # проверяем выходное значение на
        auv.set_motor_power(THRUSTER_DEPTH_LEFT,
                            output * DIRECTION_THRUSTER_DEPTH_LEFT)  # передаём выходное значение на мотор 2
        auv.set_motor_power(THRUSTER_DEPTH_RIGHT,
                            output * DIRECTION_THRUSTER_DEPTH_RIGHT)  # передаём выходное значение на мотор 3

    #        except ZeroDivisionError:
    #            time.sleep(0.001)
    #            error = get_depth_correction() - depth_to_set  # вычесление ошибки, действительное значение - заданное значение
    #            output = keep_depth.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
    #            output = clamp(output * speed_to_depth, 35, -35)  # проверяем выходное значение на ограничение
    #            auv.set_motor_power(0, output)  # передаём выходное значение на мотор 2
    #            auv.set_motor_power(3, output)  # передаём выходное значение на мотор 3
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)  # запись пк на глубину
        keep_depth.regulator.set_d_gain(Kd_depth)  # запись дк на глубину

def motor_control_regulator(time_control, yaw_to_set, depth_to_set, speed_to_yaw=0.0):
    # Функция управления моторами, принимает: time_control - время,
    # yaw_to_set - заданное значение курса,
    # speed_to_yaw - заданное значение скорости по курсу,
    if time_control > 0:
        time_new = time.time()
        while time_new + time_control > time.time():
            keep_yaw(yaw_to_set, speed_to_yaw)
            keep_depth(depth_to_set)
    else:
        keep_yaw(yaw_to_set, speed_to_yaw)
        keep_depth(depth_to_set)

def centering_h(y, Kof_smooth=1, accuracy=4.0):
    y_center = y - (480 / 2)
    try:
        time.sleep(0.001)
        output_forward = centering_h.regulator_forward.process(y_center)
        output_forward = clamp(output_forward * Kof_smooth, 70, -70)

        auv.set_motor_power(THRUSTER_DEPTH_LEFT, output_forward * DIRECTION_THRUSTER_DEPTH_LEFT)
        auv.set_motor_power(THRUSTER_DEPTH_RIGHT, output_forward * DIRECTION_THRUSTER_DEPTH_RIGHT)

        if abs(math.sqrt(y_center ** 2)) < accuracy:
            return True

    except AttributeError:
        centering_h.regulator_forward = PD()
        centering_h.regulator_forward.set_p_gain(1)
        centering_h.regulator_forward.set_d_gain(0)
    return False

def regul_angle(angle, angle_control=15, time_control=2):
    # проверяет совпадает ли направление робота со стрелкой
    # указанной количество времени, иначе перезапускает таймер
    global time_new
    if not (abs(auv.get_yaw() - angle) < angle_control):
        time_new = time.time()
    if time_control < time.time() - time_new:
        return True
    return False

yaw = auv.get_yaw()

while True:

    while True:
        frame = auv.get_image_bottom()
        frame = cv2.flip(frame, 0)
        data = defining_arrow(frame)
        mask = cv2.inRange(frame, HSV_MIN, HSV_MAX)
        # cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # if len(cnts) > 0:
        #     c = max(cnts, key=lambda x: cv2.contourArea(x))
        #     (x, y), (w, h), a = cv2.minAreaRect(c)
        #     print(a)

        print(data)
        if data:
            yaw = data[2]
            if regul_angle(data[2]):
                Yaw_const = data[2]
                break
        motor_control_regulator(0, yaw, 2, 0)
        cv2.imshow('draw', frame)
        cv2.waitKey(1)

    print("Arrow stabilized")

    motor_control_regulator(3, Yaw_const, 2, 50)

    centered_timer = None
    while True:
        frame = auv.get_image_bottom()

        x, y, angle = defining_line(frame, True)
        if x is not None:
            error = 320 / 2 - x
            print(error)
            # motor_control_regulator(0, 320 / 2 - x, 1)
            keep_depth(1)
            keep_yaw(0, error=error)
            if -10 < error < 10:
                if centered_timer is None: centered_timer = time.time()
                elif time.time() - centered_timer >= 4:
                    break
            else:
                centered_timer = None

        cv2.imshow('draw', frame)
        cv2.waitKey(1)
        
    print("Centered x")

    yaw_to_keep = auv.get_yaw()

    centered_timer = None
    while True:
        frame = auv.get_image_bottom()
        # frame = cv2.flip(frame, 0)

        x, y, angle = defining_line(frame, True)
        if y is not None:
            error = (240 / 2 - y) / 2
            print(error)
            # motor_control_regulator(0, 320 / 2 - x, 1)
            keep_depth(2)
            keep_yaw(yaw_to_keep, error)
            if -10 < error < 10:
                if centered_timer is None: centered_timer = time.time()
                elif time.time() - centered_timer >= 4:
                    break
            else:
                centered_timer = None

        cv2.imshow('draw', frame)
        cv2.waitKey(1)

    print("centered y")

    centered_timer = None
    while True:
        frame = auv.get_image_bottom()
        # frame = cv2.flip(frame, 0)

        x, y, angle = defining_line(frame, True)
        if y is not None:
            yaw = angle
            if regul_angle(angle, 10):
                Yaw_const = angle
                break
            # if -10 < error < 10:
            #     if centered_timer is None: centered_timer = time.time()
            #     elif time.time() - centered_timer >= 4:
            #         break
            # else:
            #     centered_timer = None

        motor_control_regulator(0, yaw, 2, 0)

        cv2.imshow('draw', frame)
        cv2.waitKey(1)

    print("Centered by angle")
    motor_control_regulator(5, Yaw_const, 2, 50)