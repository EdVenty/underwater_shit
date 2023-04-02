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

auv = mur.mur_init()
mur_view = auv.get_videoserver()
time_new = 0
cap0 = cv2.VideoCapture(0)
cap1 = cv2.VideoCapture(1)

THRUSTER_DEPTH_LEFT = 2
THRUSTER_DEPTH_RIGHT = 1
THRUSTER_YAW_LEFT = 3
THRUSTER_YAW_RIGHT = 0

DIRECTION_THRUSTER_DEPTH_LEFT = +1
DIRECTION_THRUSTER_DEPTH_RIGHT = -1
DIRECTION_THRUSTER_YAW_LEFT = +1
DIRECTION_THRUSTER_YAW_RIGHT = -1

Low_hsv_black = (3, 192, 0)
Max_hsv_black = (163, 255, 68)
Low_hsw_orange = (19, 47, 20)
Max_hsv_orange = (119, 255, 182)

Kp_depth = 150  # кофецент пропорционального регулятора на глубину
Kd_depth = 20  # кофецент дифференциального регулятора на глубину
Kp_yaw = 0.8  # кофецент пропорционального регулятора на курс
Kd_yaw = 1  # кофецент дифференциального регулятора на курс


class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p_gain(self, value: float) -> None:
        """Sets regulator proportional coefficient

        Args:
            value (float): Value to set
        """
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


def calculate_angle_sin_vector(x1, y1):
    # вычесляем угол
    x2 = 0 - 0
    y2 = 0 - 240
    angle = int(math.asin((x1 * y2 - y1 * x2) /
                          (((x1 ** 2 + y1 ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
    return angle


def stop_motors():
    auv.set_motor_power(0, 0)
    auv.set_motor_power(1, 0)
    auv.set_motor_power(2, 0)
    auv.set_motor_power(3, 0)
    time.sleep(1)


def keep_yaw(yaw_to_set, speed_to_yaw=0):  # ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
    try:
        #        try:
        #            error = yaw_to_set - auv.get_yaw()  # вычесление ошибки, действительное значение - заданное значение
        #            error = clamp_to180(error)  # проверяем ошибку на ограничение
        #            output = keep_yaw.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
        #            output = clamp(output, 100, -100)  # проверяем выходное значение на ограничение
        #            auv.set_motor_power(1, clamp((speed_to_yaw - output), 50, -50))  # передаём выходное значение на мотор 0
        #            auv.set_motor_power(2, clamp((speed_to_yaw + output), 50, -50))  # передаём выходное значение на мотор 1
        #        except ZeroDivisionError:
        time.sleep(0.001)
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


def defining_figure(image):
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    img_bin = cv2.inRange(imageHSV, Low_hsw_orange, Max_hsv_orange)  # бинаризация изображения. по цветам

    c, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # выделение контуров.
    if c:  # проверяем не пустой ли список контуров
        c = sorted(c, key=cv2.contourArea, reverse=True)[0]  # берём наибольшую маску
        if cv2.contourArea(c) > 500:  # проверяем не мусор ли это
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
            _, img1 = cap1.read()
            imageHSV = cv2.cvtColor(img1, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
            mur_view.show(img1, 0)
    else:
        keep_yaw(yaw_to_set, speed_to_yaw)
        keep_depth(depth_to_set)


def motor_control_regulator_s(time_control, yaw_to_set, depth_to_set, speed_to_yaw=0.0):
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
        keep_depth(depth_to_set, True)


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


def centering_r(yaw, y, Kof_smooth=1, accuracy=10.0):
    y_center = y - (240 / 2)
    try:
        time.sleep(0.001)
        output_forward = centering_r.regulator_forward.process(y_center)
        output_forward = clamp(output_forward * Kof_smooth, 70, -70)
        keep_yaw(yaw, -output_forward)

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


def regul_angle(angle, angle_control=5, time_control=3):
    # проверяет совпадает ли направление робота со стрелкой
    # указанной количество времени, иначе перезапускает таймер
    global time_new
    if not (abs(auv.get_yaw() - angle) < angle_control):
        time_new = time.time()
    if time_control < time.time() - time_new:
        return True
    return False


def view_img(img1):
    imageHSV = cv2.cvtColor(img1, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
    img_bin = cv2.inRange(imageHSV, Low_hsw_orange, Max_hsv_orange)  # бинаризация изображения.
    mur_view.show(img1, 0)
    mur_view.show(img_bin, 1)


def defining_arrow(image):  # вычесление угла у стрелки путём нахождения наибольшой стороны
    # image = auv.get_image_bottom()  # получение изображения с донной камеры в RGB формате.
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
    img_bin = cv2.inRange(imageHSV, Low_hsw_orange, Max_hsv_orange)  # бинаризация изображения.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # выделение контуров.
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[0]  # берём наибольшую маску
        if cv2.contourArea(c) > 500:  # проверяем не мусор ли это
            c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            # рисуем треугольник, получаем площад
            s_triangle, angle_arrow = cv2.minEnclosingTriangle(c)
            #            angle_arrow = cv2.approxPolyDP(s_triangle, cv2.arcLength(s_triangle, True) * 0.04, True)

            print(len(angle_arrow), "@@")
            # рисуем параллелограмм, получаем высоту и длину
            ((_, _), (w, h), _) = cv2.minAreaRect(c)
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
            # elif W1 < W3 > W2:
            else:
                coordinte = [[angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]],
                             [angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]],
                             [angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]]]

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
                return x, y, -angle + int(auv.get_yaw())  # данные угла для регулятора по курсу
            except:
                return False
    return False


def defining_figure(image):
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    img_bin = cv2.inRange(imageHSV, Low_hsw_black, Max_hsv_black)  # бинаризация изображения. по цветам

    c, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # выделение контуров.
    if c:  # проверяем не пустой ли список контуров
        c = sorted(c, key=cv2.contourArea, reverse=True)[0]  # берём наибольшую маску
        if cv2.contourArea(c) > 500:  # проверяем не мусор ли это
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

            # cv2.imshow("w", img_bin)
            # cv2.waitKey(1)
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
                return x, y, "rectangle"
            elif rectangle_area > s_triangle < circle_area:
                return x, y, "triangle"
    return False


Yaw_const = auv.get_yaw()

# if __name__ == '__main__':
#    while True:
#        view_img()

# if __name__ == '__main__':
#    time.sleep(4)
#    print("Starting main")
#
#    motor_control_regulator(5, Yaw_const, 0.6, 0)
#
#    while True:
#        _, img0 = cap1.read()
#        data = defining_arrow(img0)
#        view_img(img0)
#
#        print(data)
#        if data:
#            motor_control_regulator(0, data[2], 0.6, 0)
#            if regul_angle(data[2]):
#                Yaw_const = data[2]
#                break

#
#    while True:
#        _, img0 = cap0.read()
#        data = defining_figure(img0)
#        motor_control_regulator(0, Yaw_const, 1.5, 20)
#        if data:
#            if data[2] == "rectangle":
#                break
#
#    while True:
#        _, img0 = cap0.read()
#        data = defining_figure(img0)
#        if data:
#            data = calculate_angle_sin_vector(160 - data[0], 240 - data[1]) + auv.get_yaw()
#            motor_control_regulator(0, data, 1.5, 0)
#            if regul_angle(data):
#                Yaw_const = data
#                break
#
#    while True:
#        _, img0 = cap0.read()
#        data = defining_figure(img0)
#        if data:
#            if regul_distance(centering_r(Yaw_const, data[1])):
#                break
#
#    motors_up()
#
#    print("Finish")
#    stop_motors()


#    ygg

if __name__ == '__main__':
    print("Starting main")

    # motor_control_regulator(5, 0, 0.6, 0)
    # motor_control_regulator(5, 20, 0.6, 0)
    # motor_control_regulator(10, 20, 0.6, 20)
    # motor_control_regulator(5, 20, -1, 0)
    # print("Finish")
    # stop_motors()
    while True:
        _, img0 = cap0.read()
        imageHSV = cv2.cvtColor(img0, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
        img_bin = cv2.inRange(imageHSV, Low_hsv_black, Max_hsv_black)  # бинаризация изображения.
        mur_view.show(img0, 0)
        mur_view.show(img_bin, 1)

