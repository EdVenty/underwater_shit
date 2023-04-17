# Данный пример предназначен для тестирования моторов на аппарате!

# В данном примере мы подаем тягу на 4 мотора продолжительностью в 5 секунд.
# для моторов
# 0 - правый мотор на глубину -> 2
# 1 - левый мотор на курс -> 0
# 2 - правый мотор на курс -> 3
# 3 - левый мотор на глубину -> 1

# 0 - донная камера
# 1 - передняя камера

# id - http://10.3.141.1/
import cv2
import pymurapi as mur
import time
import math
import numpy as np

SIMULATOR = True

ROBOT = not SIMULATOR

auv = mur.mur_init()
time_new = 0

DEPTH = 2

if SIMULATOR:
    THRUSTER_DEPTH_LEFT = 2
    THRUSTER_DEPTH_RIGHT = 3
    THRUSTER_YAW_LEFT = 0
    THRUSTER_YAW_RIGHT = 1

    DIRECTION_THRUSTER_DEPTH_LEFT = +1
    DIRECTION_THRUSTER_DEPTH_RIGHT = +1
    DIRECTION_THRUSTER_YAW_LEFT = +1
    DIRECTION_THRUSTER_YAW_RIGHT = +1

else:
    THRUSTER_DEPTH_LEFT = 0
    THRUSTER_DEPTH_RIGHT = 3
    THRUSTER_YAW_LEFT = 1
    THRUSTER_YAW_RIGHT = 2

    DIRECTION_THRUSTER_DEPTH_LEFT = -1
    DIRECTION_THRUSTER_DEPTH_RIGHT = -1
    DIRECTION_THRUSTER_YAW_LEFT = 1
    DIRECTION_THRUSTER_YAW_RIGHT = 1

    cap1 = cv2.VideoCapture(1)
    cap0 = cv2.VideoCapture(0)

    mur_view = auv.get_videoserver()

Low_hsv_black = (0, 0, 0)
Max_hsv_black = (180, 30, 255)
Low_hsv_orange = (0, 10, 10)
Max_hsv_orange = (70, 255, 255)
# Low_hsw_orange = (30, 10, 15)
# Max_hsv_orange = (119, 182, 255)

Kp_depth = 150  # кофецент пропорционального регулятора на глубину
Kd_depth = 20  # кофецент дифференциального регулятора на глубину
Kp_yaw = 0.3  # кофецент пропорционального регулятора на курс
Kd_yaw = 1  # кофецент дифференциального регулятора на курс

colors_dict = {
    'red': ((160, 50, 50), (180, 255, 255)),
    'orange': ((10, 50, 50), (20, 255, 255)),
    'green': ((45, 50, 50), (75, 255, 255)),
    'yellow': ((20, 50, 50), (40, 255, 255)),
}


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
        output = self._kp * error + self._kd / \
            (timestamp - self._timestamp) * (error - self._prev_error)
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
    return auv.get_depth() * k - 0.1


def angle_correct(angle):
    return angle + int(auv.get_yaw())


def calculate_angle_sin_vector(x1, y1):
    # вычесляем угол
    x2 = 0 - 0
    y2 = 0 - 240
    try:
        angle = int(math.asin((x1 * y2 - y1 * x2) /
                              (((x1 ** 2 + y1 ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
    except ZeroDivisionError:
        angle = int(math.asin((x1 * y2 - y1 * x2) /
                              (((x1 ** 2 + y1 ** 2) ** 0.5 + 0.000001) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
    return angle


def calculate_angle_cos(x1, y1):
    x2 = 0 - 0
    y2 = 0 - 240
    # вычесляем угол
    try:
        angle = int(math.acos((x1 * x2 + y1 * y2) /
                              (((x1 ** 2 + y1 ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
    except:
        pass
    return angle


def calculate_angle(x1, y1):
    angle_ = calculate_angle_cos(x1, y1)
    angle = calculate_angle_sin_vector(x1, y1)
    try:
        angle = angle_ * (angle / abs(angle))
    except ZeroDivisionError:
        pass
    return int(angle)


def stop_motors():
    auv.set_motor_power(0, 0)
    auv.set_motor_power(1, 0)
    auv.set_motor_power(2, 0)
    auv.set_motor_power(3, 0)
    time.sleep(1)


def open_gr():
    auv.set_motor_power(4, 100)
    time.sleep(1)
    auv.set_motor_power(4, 0)
    time.sleep(1)


# ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
def keep_yaw(yaw_to_set, speed_to_yaw=0, error=...):
    try:
        if error is ...:
            # вычесление ошибки, действительное значение - заданное значение
            error = yaw_to_set - auv.get_yaw()
            error = clamp_to180(error)  # проверяем ошибку на ограничение
        time.sleep(0.001)
        # забиваем ошибку и получаем выходное значение на моторы
        output = keep_yaw.regulator.process(error)
        # проверяем выходное значение на ограничение
        output = clamp(output, 100, -100)
        auv.set_motor_power(THRUSTER_YAW_LEFT, DIRECTION_THRUSTER_YAW_LEFT * clamp((speed_to_yaw - output), 100,
                                                                                   -100))  # передаём выходное значение на мотор 0
        auv.set_motor_power(THRUSTER_YAW_RIGHT, DIRECTION_THRUSTER_YAW_RIGHT * clamp((speed_to_yaw + output), 100,
                                                                                     -100))  # передаём выходное значение на мотор 1
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(Kp_yaw)  # запись пк на курс
        keep_yaw.regulator.set_d_gain(Kd_yaw)  # запись дк на курс


# ПД регулятор по глубине, !!! без ожидания будет выдавать ошибку !!!
def keep_depth(depth_to_set):
    speed_to_depth = 1
    try:
        time.sleep(0.001)
        # вычесление ошибки, действительное значение - заданное значение
        error = get_depth_correction() - depth_to_set
        # забиваем ошибку и получаем выходное значение на моторы
        output = keep_depth.regulator.process(error)
        output += 10
        # проверяем выходное значение на
        output = clamp(output * speed_to_depth, 30, -30)
        auv.set_motor_power(THRUSTER_DEPTH_LEFT,
                            output * DIRECTION_THRUSTER_DEPTH_LEFT)  # передаём выходное значение на мотор 2
        auv.set_motor_power(THRUSTER_DEPTH_RIGHT,
                            output * DIRECTION_THRUSTER_DEPTH_RIGHT)  # передаём выходное значение на мотор 3
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)  # запись пк на глубину
        keep_depth.regulator.set_d_gain(Kd_depth)  # запись дк на глубину


def motor_control_regulator(time_control, yaw_to_set, depth_to_set, speed_to_yaw=0.0):
    # Функция управления моторами, принимает: time_control - время,
    # по умолчанию равное нулю, то есть работает один раз,
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


def centering_r(yaw, y, depth=DEPTH, Kof_smooth=0.5, accuracy=15.0):
    y_center = y - (240 / 2)
    try:
        time.sleep(0.001)
        output_forward = centering_r.regulator_forward.process(y_center)
        output_forward = clamp(output_forward * Kof_smooth, 40, -40)
        keep_yaw(yaw, -output_forward)
        keep_depth(depth)
        if abs(math.sqrt(y_center ** 2)) < accuracy:
            return True
    except AttributeError:
        centering_r.regulator_forward = PD()
        centering_r.regulator_forward.set_p_gain(1)
        centering_r.regulator_forward.set_d_gain(0)

    return False


def regul_distance(distance, time_control=3):
    global time_new
    if not (distance):
        time_new = time.time()
    if time_control < time.time() - time_new:
        return True
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


def angle_line_score(yaw, score=10):
    angle_sum = 0
    angle_sum_c = 0
    while True:
        img0 = get_img()
        data = defining_line(img0)
        view_img(data[3], Low_hsv_orange, Max_hsv_orange)
        motor_control_regulator(0, yaw, 0.5, 0)
        if data:
            angle_sum += angle_correct(data[2])
            angle_sum_c += 1
            if angle_sum_c >= score:
                return angle_sum / angle_sum_c


def get_img(cam=0):
    if SIMULATOR:
        if cam == 0:
            return auv.get_image_bottom()
        else:
            return auv.get_image_front()
    else:
        if cam == 0:
            return cap0.read()[1]
        else:
            return cap1.read()[1]


def view_img(img1, Low_hsv, Max_hsv):
    # перевод изображения из RGB в HSV формат.
    imageHSV = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    # бинаризация изображения.
    img_bin = cv2.inRange(imageHSV, Low_hsv, Max_hsv)
    if SIMULATOR:
        cv2.imshow('Image', img1)
        cv2.imshow('Bin', img_bin)

        cv2.waitKey(1)
    else:
        mur_view.show(img1, 0)
        mur_view.show(img_bin, 1)


def defining_arrow(image):  # вычесление угла у стрелки путём нахождения наибольшой стороны
    # image = auv.get_image_bottom()  # получение изображения с донной камеры в RGB формате.
    # перевод изображения из RGB в HSV формат.
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # бинаризация изображения.
    img_bin = cv2.inRange(imageHSV, Low_hsv_orange, Max_hsv_orange)
    # выделение контуров.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[
            0]  # берём наибольшую маску
        if cv2.contourArea(c) > 100:  # проверяем не мусор ли это
            c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            s_triangle, angle_arrow = cv2.minEnclosingTriangle(c)
            #            cv2.drawContours(image, cnt, -1, (255, 0, 0), 3)

            moments = cv2.moments(c)  # получение моментов
            # нахождение наибольшой стороны.
            W1 = (((angle_arrow[1, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
                  + ((angle_arrow[1, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5

            W2 = (((angle_arrow[0, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
                  + ((angle_arrow[0, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5

            W3 = (((angle_arrow[1, 0, 0] - angle_arrow[0, 0, 0]) ** 2)
                  + ((angle_arrow[1, 0, 1] - angle_arrow[0, 0, 1]) ** 2)) ** 0.5

            if W3 < W1 > W2:
                coordinte = [[angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]],
                             [angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]],
                             [angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]]]
            elif W3 < W2 > W1:
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
            angle = calculate_angle(x1, y1)
            try:
                # координаты центра стрелки
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                return x, y, angle, image  # данные угла для регулятора по курсу
            except:
                return False
    return False


def defining_figure(image, Low_hsv, Max_hsv):
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # бинаризация изображения. по цветам
    img_bin = cv2.inRange(imageHSV, Low_hsv, Max_hsv)

    # выделение контуров.
    c, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if c:  # проверяем не пустой ли список контуров
        c = sorted(c, key=cv2.contourArea, reverse=True)[
            0]  # берём наибольшую маску
        if cv2.contourArea(c) > 1000:  # проверяем не мусор ли это
            c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            # angle_arrow = cv2.approxPolyDP(c, cv2.arcLength(c, True) * 0.15, True)

            # рисуем треугольник, получаем площадь
            s_triangle, _ = cv2.minEnclosingTriangle(c)
            # рисуем прямоугольник, получаем x,y верхнего левого угла,
            # высоту и длину и угол  поворота
            ((_, _), (w, h), _) = cv2.minAreaRect(c)
            #  возвращает координаты центра x, y и радиус (строит круг)
            _, radius = cv2.minEnclosingCircle(c)

            # вычесление площадей кргуа, квадрата, треугольника
            rectangle_area = w * h
            circle_area = radius ** 2 * math.pi

            # вычесление центра фигуры
            moments = cv2.moments(c)  # получение моментов
            try:
                x = int(moments["m10"] / moments["m00"])  # центр
                y = int(moments["m01"] / moments["m00"])
            except ZeroDivisionError:
                return False
            if rectangle_area > circle_area < s_triangle:
                return x, y, "circle"
            elif circle_area > rectangle_area < s_triangle:
                return x, y, "square"
            elif rectangle_area > s_triangle < circle_area:
                return x, y, "triangle"
    return False


# вычесление угла полоски, работает при отклонение до 60. (надо доработать)
def defining_line(image):
    # image = auv.get_image_bottom()  # п олучение изображения с донной камеры в RGB формате.
    # перевод изображения из RGB в HSV формат.
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # бинаризация изображения.
    img_bin = cv2.inRange(imageHSV, Low_hsv_orange, Max_hsv_orange)
    # выделение контуров.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[
            0]  # берём наибольшую маску
        if cv2.contourArea(c) > 100:  # проверяем не мусор ли это
            c = cv2.convexHull(c)  # сглаживаем фигуру
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            moments = cv2.moments(c)  # получение моментов
            x1, y1 = box[1][0] - box[0][0], box[1][1] - box[0][1]
            x2, y2 = box[2][0] - box[1][0], box[2][1] - box[1][1]
            if (x2 ** 2 + y2 ** 2) ** 0.5 > (x1 ** 2 + y1 ** 2) ** 0.5:
                x1, y1 = x2, y2
            angle = calculate_angle_sin_vector(x1, y1)
            # вычесляем знак угла
            try:
                # координаты центра стрелки
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                return x, y, angle, image
            except:
                return False
    return False


def angle_score(yaw, score=10):
    angle_sum = 0
    angle_sum_c = 0
    while True:
        img0 = get_img()
        data = defining_arrow(img0)
        view_img(data[3], Low_hsv_orange, Max_hsv_orange)
        motor_control_regulator(0, yaw, 0.5, 0)
        if data:
            angle_sum += angle_correct(data[2])
            angle_sum_c += 1
            if angle_sum_c >= score:
                return angle_sum / angle_sum_c


def angle_line_score(yaw, score=10):
    angle_sum = 0
    angle_sum_c = 0
    while True:
        img0 = get_img()
        data = defining_line(img0)
        view_img(data[3], Low_hsv_orange, Max_hsv_orange)
        motor_control_regulator(0, yaw, 0.5, 0)
        if data:
            angle_sum += angle_correct(data[2])
            angle_sum_c += 1
            if angle_sum_c >= score:
                return angle_sum / angle_sum_c


def regul_angle_figure(yaw, hsv_min=Low_hsv_orange, hsv_max=Max_hsv_orange):
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        view_img(img, *colors_dict['red'])
        data = defining_figure(img, hsv_min, hsv_max)
        if data:
            data_ang = - \
                calculate_angle_sin_vector(
                    160 - data[0], 120 - data[1]) + auv.get_yaw()
            print(data_ang)
            motor_control_regulator(0, data_ang, 0.5, 0)
            yaw = data_ang
            if regul_angle(data_ang, 30, 1):
                return data_ang
        else:
            motor_control_regulator(0, yaw, 0.5, 0)


def regul_angle_figure_2(yaw):
    time_new = None
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        #        img = cv2.flip(img, 1)
        data = defining_figure(img, Low_hsv_orange, Max_hsv_orange)
        if data:
            error = (data[0] - 320 / 2)
            n_error = error
            if error > 0:
                auv.set_rgb_color(0, 0, 255)
#                 error = clamp(error, 255, 50)
            elif error < 0:
                auv.set_rgb_color(0, 255, 0)
#                 error = clamp(error, -255, -50)
            yaw = error
            print(data[0], n_error)
            # motor_control_regulator(0, error + auv.get_yaw(), DEPTH, 0)
            keep_depth(DEPTH)
            keep_yaw(0, error=error)
            if -40 < n_error < 40:
                if time_new is None:
                    time_new = time.time()
                elif time.time() - time_new >= 2:
                    return auv.get_yaw()
            else:
                time_new = None
        else:
            auv.set_rgb_color(255, 255, 0)
            motor_control_regulator(0, yaw, 0.5, 0)
            time_new = None
#        mur_view.show(0, img)


def regul_r_figure(yaw, hsv_min=Low_hsv_orange, hsv_max=Max_hsv_orange):
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        data = defining_figure(img, hsv_min, hsv_max)
        print(data)
        if data:
            if regul_distance(centering_r(yaw, data[1])):
                return True


def figure_search(yaw):
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        data = defining_figure(img, Low_hsv_orange, Max_hsv_orange)
        data2 = defining_figure(img, *colors_dict["red"])
        motor_control_regulator(0, yaw, 0.5, 20)
        if data:
            return True
        if data2:
            return "red"


Yaw_const = auv.get_yaw()
yaw = auv.get_yaw()

DEPTH = 1

if __name__ == '__main__':
    success = False
    try:
        auv.set_rgb_color(255, 0, 255) if ROBOT else ...
        auv.set_off_delay(0) if ROBOT else ...
        time.sleep(4)
        Yaw_const = auv.get_yaw()
        auv.set_off_delay(0.5) if ROBOT else ...
        motor_control_regulator(5, Yaw_const, DEPTH, 1)
        print("Starting main")
        auv.set_rgb_color(0, 255, 0) if ROBOT else ...

        yaw = regul_angle_figure(yaw, *colors_dict['red'])
        auv.set_rgb_color(0, 0, 255) if ROBOT else ...
        print("Centered yaw")

        regul_r_figure(yaw, *colors_dict['red'])
        auv.set_rgb_color(39, 237, 201) if ROBOT else ...

        print("Centered y")
        yaw = angle_score(yaw)

        motor_control_regulator(5, yaw, DEPTH, 0)
        auv.set_rgb_color(255, 255, 255) if ROBOT else ...
        motor_control_regulator(4, yaw, DEPTH, 40)

        while True:
            res = figure_search(yaw)

            if res == "red":
                while True:
                    keep_depth(-1)

            auv.set_rgb_color(0, 255, 0) if ROBOT else ...
            yaw = regul_angle_figure(yaw)
            auv.set_rgb_color(0, 0, 255) if ROBOT else ...
            print("Centered yaw")

            regul_r_figure(yaw)
            auv.set_rgb_color(39, 237, 201) if ROBOT else ...
            print("Centered x")
            yaw = angle_line_score(yaw)

            motor_control_regulator(5, yaw, DEPTH, 0)
            auv.set_rgb_color(255, 255, 255) if ROBOT else ...
            motor_control_regulator(4, yaw, DEPTH, 40)
    finally:
        if not success:
            auv.set_rgb_color(255, 0, 0) if ROBOT else ...
            auv.set_on_delay(0.1) if ROBOT else ...
            auv.set_off_delay(0.1) if ROBOT else ...
        keep_depth(-1)
