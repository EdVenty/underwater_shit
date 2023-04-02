import pymurapi as mur
import time
import cv2
import math
import numpy as np

auv = mur.mur_init()


Kp_depth = 35  # кофецент пропорционального регулятора на глубину
Kd_depth = 7  # кофецент дифференциального регулятора на глубину
Kp_yaw = 0.5  # кофецент пропорционального регулятора на курс
Kd_yaw = 1  # кофецент дифференциального регулятора на курс

low_hsw = (10, 50, 50)
max_hsv = (50, 255, 255)  # рамки бинаризации для выделение оранжевого цвета.

time_new = time.time()


#  класс для Пд-регулятора
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


def keep_depth(depth_to_set, speed_to_depth=1.0):  # ПД регулятор по глубине, !!! без ожидания будет выдавать ошибку !!!
    try:
        try:
            error = auv.get_depth() - depth_to_set  # вычесление ошибки, действительное значение - заданное значение
            output = keep_depth.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
            output = clamp(output * speed_to_depth, 100, -100)  # проверяем выходное значение на ограничение
            auv.set_motor_power(2, output)  # передаём выходное значение на мотор 2
            auv.set_motor_power(3,  output)  # передаём выходное значение на мотор 3
        except ZeroDivisionError:
            time.sleep(0.001)
            error = auv.get_depth() - depth_to_set  # вычесление ошибки, действительное значение - заданное значение
            output = keep_depth.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
            output = clamp(output * speed_to_depth, 100, -100)  # проверяем выходное значение на ограничение
            auv.set_motor_power(2, output)  # передаём выходное значение на мотор 2
            auv.set_motor_power(3, output)  # передаём выходное значение на мотор 3
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)  # запись пк на глубину
        keep_depth.regulator.set_d_gain(Kd_depth)  # запись дк на глубину


def keep_yaw(yaw_to_set, speed_to_yaw=0):  # ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
    try:
        try:
            error = yaw_to_set - auv.get_yaw()  # вычесление ошибки, действительное значение - заданное значение
            error = clamp_to180(error)  # проверяем ошибку на ограничение
            output = keep_yaw.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
            output = clamp(output, 100, -100)  # проверяем выходное значение на ограничение
            auv.set_motor_power(1, clamp((speed_to_yaw - output), 75, -75))  # передаём выходное значение на мотор 0
            auv.set_motor_power(0, clamp((speed_to_yaw + output), 75, -75))  # передаём выходное значение на мотор 1
        except ZeroDivisionError:
            time.sleep(0.001)
            error = auv.get_yaw() - yaw_to_set  # вычесление ошибки, действительное значение - заданное значение
            error = clamp_to180(error)  # проверяем ошибку на ограничение
            output = keep_yaw.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
            output = clamp(output, 100, -100)  # проверяем выходное значение на ограничение
            auv.set_motor_power(1, clamp((speed_to_yaw - output), 75, -75))  # передаём выходное значение на мотор 0
            auv.set_motor_power(0, clamp((speed_to_yaw + output), 75, -75))  # передаём выходное значение на мотор 1
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(Kp_yaw)  # запись пк на курс
        keep_yaw.regulator.set_d_gain(Kd_yaw)  # запись дк на курс


def stab_on_(x, y, Kof_smooth=1, camera_view=True, accuracy=4.0):
    x_center = x - (320 / 2)
    y_center = y - (240 / 2)
    if camera_view:
        try:
            time.sleep(0.001)
            output_forward = stab_on_.regulator_forward.process(y_center)
            output_side = stab_on_.regulator_side.process(x_center)
            output_forward = clamp(output_forward * Kof_smooth, 70, -70)
            output_side = clamp(output_side * Kof_smooth, 70, -70)

            auv.set_motor_power(0, -output_forward)
            auv.set_motor_power(1, -output_forward)
            auv.set_motor_power(4, -output_side)

            if abs(math.sqrt(x_center ** 2 + y_center ** 2)) < accuracy:
                return True

        except AttributeError:
            stab_on_.regulator_forward = PD()
            stab_on_.regulator_forward.set_p_gain(1.5)
            stab_on_.regulator_forward.set_d_gain(1.8)

            stab_on_.regulator_side = PD()
            stab_on_.regulator_side.set_p_gain(0.5)
            stab_on_.regulator_side.set_d_gain(1)
    else:
        pass
        output_side = stab_on_.regulator_side.process(x_center)
        output_side = clamp(output_side * Kof_smooth, 70, -70)
        auv.set_motor_power(4, output_side)

    return False


def defining_arrow():
    image = auv.get_image_bottom()  # получение изображения с донной камеры в RGB формате.
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
    img_bin = cv2.inRange(imageHSV, low_hsw, max_hsv)  # бинаризация изображения.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # выделение контуров.
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[0]  # берём наибольшую маску
        if cv2.contourArea(c) > 100:  # проверяем не мусор ли это
            c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            angle_arrow = cv2.approxPolyDP(c, cv2.arcLength(c, True) * 0.15, True)
            # рисуем треугольник, получаем площадь
            s_triangle, _ = cv2.minEnclosingTriangle(c)
            # рисуем параллелограмм, получаем высоту и длину
            ((_, _), (w, h), _) = cv2.minAreaRect(c)
            # проверка на то, что аппроксимировалась фигура по трём точкам
            # проверяем, что площадь треугольника меньше параллелипида, нарисованного вокруг фигуры
            if len(angle_arrow) != 3:
                return False
            if not(s_triangle < w * h):
                return False
            moments = cv2.moments(angle_arrow)  # получение моментов
            # нахождение наибольшой стороны.
            W1 = (((angle_arrow[1, 0, 0] - angle_arrow[0, 0, 0]) ** 2)
                  + ((angle_arrow[1, 0, 1] - angle_arrow[0, 0, 1]) ** 2)) ** 0.5 +\
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
            # elif W1 > W3 < W2:
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
            x2 = 0 - 0
            y2 = 0 - 240
            # вычесляем угол
            angle = calculate_angle(x1, x2, y1, y2)
            # вычесляем знак угла
            angle = angle if coordinte[1][1] > coordinte[2][1] else -angle
            try:
                x = int(moments["m10"] / moments["m00"])  # координаты центра стрелки
                y = int(moments["m01"] / moments["m00"])
                return x, y, angle
            except:
                return False
    return False


def defining_line():
    image = auv.get_image_bottom()  # получение изображения с донной камеры в RGB формате.
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
    img_bin = cv2.inRange(imageHSV, low_hsw, max_hsv)  # бинаризация изображения.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # выделение контуров.
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[0]  # берём наибольшую маску
        if cv2.contourArea(c) > 100:  # проверяем не мусор ли это

            c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            coordinate = cv2.approxPolyDP(c, cv2.arcLength(c, True) * 0.05, True)
            # рисуем треугольник, получаем площадь
            s_triangle, _ = cv2.minEnclosingTriangle(c)
            # рисуем параллелограмм, получаем высоту и длину
            ((_, _), (w, h), _) = cv2.minAreaRect(c)
            # проверка на то, что аппроксимировалась фигура по трём точкам
            # проверяем, что площадь треугольника меньше параллелипида, нарисованного вокруг фигуры
            if len(coordinate) != 4:
                return False
            if not (s_triangle > w * h):
                return False
            moments = cv2.moments(coordinate)  # получение моментов
            # нахождение длинной стороны.
            coordinate = coordinate[:, 0, :]
            # print((aarr(coordinate)))
            print(coordinate[:, 0])
            l = np.sort(coordinate)

            # cv2.circle(image, (l[0], l[0]), 3, (0, 0, 0), -1)
            # cv2.imshow("WWW", image)
            # cv2.waitKey(1)
            print(l)
            #  вычисдения кооринат середины наибольшой стороны треугольника
            # x_centre_arrow = (coordinte[1][0] + coordinte[2][0]) // 2
            # y_centre_arrow = (coordinte[1][1] + coordinte[2][1]) // 2
            # # вычесление векторов для расчёта угла стрелки
            # x1 = coordinte[0][0] - x_centre_arrow
            # y1 = coordinte[0][1] - y_centre_arrow
            # x2 = 0 - 0
            # y2 = 0 - 240
            # # вычесляем угол
            # angle = calculate_angle(x1, x2, y1, y2)
            # # вычесляем знак угла
            # angle = angle if coordinte[1][1] > coordinte[2][1] else -angle
            # try:
            #     x = int(moments["m10"] / moments["m00"])  # координаты центра стрелки
            #     y = int(moments["m01"] / moments["m00"])
            #     return x, y, angle
            # except:
            #     return False
    return False


def get_big_mask_img():
    image = auv.get_image_bottom()  # получение изображения с донной камеры в RGB формате.
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
    img_bin = cv2.inRange(imageHSV, low_hsw, max_hsv)  # бинаризация изображения.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # выделение контуров.
    if cnt:  # проверяем не пустой ли список контуров
        c = sorted(cnt, key=cv2.contourArea, reverse=True)[0]  # берём наибольшую маску
        if cv2.contourArea(c) > 100:  # проверяем не мусор ли это
            return True, c
    return False, False


def regul_figure(angle, angle_control=5, time_control=3):
    # проверяет совпадает ли направление робота со стрелкой
    # указанной количество времени, иначе перезапускает таймер
    global time_new
    if not(abs(auv.get_yaw() - angle) < angle_control):
        time_new = time.time()
    if time_control < time.time() - time_new:
        return True
    return False


def alignment_arrow(depth=0.5):
    # ищет стрелку, центруется и выравнивается по ней
    while True:
        keep_depth(depth)
        vales = defining_arrow()
        if vales:
            keep_yaw(vales[2] + auv.get_yaw())
            stab_on_(vales[0], vales[1])
            if regul_figure(vales[2] + auv.get_yaw()):
                break


def alignment_line(depth=0.5):
    # ищет стрелку, центруется и выравнивается по ней
    while True:
        keep_depth(depth)
        vales = defining_line()
        if vales:
            keep_yaw(vales[2] + auv.get_yaw())
            stab_on_(vales[0], vales[1])
            if regul_figure(vales[2] + auv.get_yaw()):
                break


def calculate_angle(x1, x2, y1, y2):
    # вычесляем угол
    angle = int(math.acos((x1 * x2 + y1 * y2) /
                          (((x1 ** 2 + y1 ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
    return angle


if __name__ == "__main__":
    while True:
        defining_line()
    print("Finish")
