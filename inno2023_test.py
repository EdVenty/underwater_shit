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
from sys import platform

def dertemine_robot():
    return platform == "linux" or platform == "linux2"

SIMULATOR = not dertemine_robot()

ROBOT = not SIMULATOR

auv = mur.mur_init()
time_new = 0

if SIMULATOR:
    DEPTH = 2.5
    GO_DEEPER_DEPTH = 3.3

    THRUSTER_DEPTH_LEFT = 2
    THRUSTER_DEPTH_RIGHT = 3
    THRUSTER_YAW_LEFT = 0
    THRUSTER_YAW_RIGHT = 1

    DIRECTION_THRUSTER_DEPTH_LEFT = +1
    DIRECTION_THRUSTER_DEPTH_RIGHT = +1
    DIRECTION_THRUSTER_YAW_LEFT = +1
    DIRECTION_THRUSTER_YAW_RIGHT = +1

    Kp_depth = 150  # кофецент пропорционального регулятора на глубину
    Kd_depth = 200  # кофецент дифференциального регулятора на глубину
    Kp_yaw = 1   # кофецент пропорционального регулятора на курс
    Kd_yaw = 0.001  # кофецент дифференциального регулятора на курс

else:
    DEPTH = 0.
    GO_DEEPER_DEPTH = 0.

    THRUSTER_DEPTH_LEFT = 0
    THRUSTER_DEPTH_RIGHT = 3
    THRUSTER_YAW_LEFT = 1
    THRUSTER_YAW_RIGHT = 2

    DIRECTION_THRUSTER_DEPTH_LEFT = 1
    DIRECTION_THRUSTER_DEPTH_RIGHT = 1
    DIRECTION_THRUSTER_YAW_LEFT = -1
    DIRECTION_THRUSTER_YAW_RIGHT = -1

    cap1 = cv2.VideoCapture(1)
    cap0 = cv2.VideoCapture(0)

    mur_view = auv.get_videoserver()

    Kp_depth = 150  # кофецент пропорционального регулятора на глубину
    Kd_depth = 100  # кофецент дифференциального регулятора на глубину
    Kp_yaw = 0.6  # кофецент пропорционального регулятора на курс
    Kd_yaw = 0.6 # кофецент дифференциального регулятора на курс

Low_hsv_black = (0, 0, 0)
Max_hsv_black = (180, 30, 255)
Low_hsv_orange = (0, 10, 10)
Max_hsv_orange = (70, 255, 255)
# Low_hsw_orange = (30, 10, 15)
# Max_hsv_orange = (119, 182, 255)

IMAGE_CENTER_X = 320 // 2
IMAGE_CENTER_Y = 240 // 2

colors_dict = {
    'red': ((160, 50, 50), (180, 255, 255)),
    'orange': ((8, 10, 10), (25, 220, 220)),
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

def clamp(v, min, max):
    if v < min: return min
    if v > max: return max
    return v


def get_depth_correction(k=1):
    return auv.get_depth()

def angle_correct(angle):
    return clamp_to180(angle + auv.get_yaw())


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
    # angle_ = calculate_angle_cos(x1, y1)
    # angle = calculate_angle_sin_vector(x1, y1)
    # try:
    #     angle = angle_ * (angle / abs(angle))
    # except ZeroDivisionError:
    #     pass
    # return int(angle)
    return math.degrees(math.atan2(IMAGE_CENTER_Y - y1, IMAGE_CENTER_X - x1))


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


def keep_yaw(yaw_to_set, speed_to_yaw=0, error=...):
    try:
        if error is ...:
            # вычесление ошибки, действительное значение - заданное значение
            error = clamp_to180(yaw_to_set - auv.get_yaw())
            # error = clamp_to180(error)  # проверяем ошибку на ограничение
        time.sleep(0.001)
        # забиваем ошибку и получаем выходное значение на моторы
        output = keep_yaw.regulator.process(error)
        # проверяем выходное значение на ограничение
        output = clamp(output, -30, 30)
#        print(output)
        auv.set_motor_power(THRUSTER_YAW_LEFT, DIRECTION_THRUSTER_YAW_LEFT * clamp((speed_to_yaw - output), -100,
                                                                                   100))  # передаём выходное значение на мотор 0
        auv.set_motor_power(THRUSTER_YAW_RIGHT, DIRECTION_THRUSTER_YAW_RIGHT * clamp((speed_to_yaw + output), -100,
                                                                                     100))  # передаём выходное значение на мотор 1
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(Kp_yaw)  # запись пк на курс
        keep_yaw.regulator.set_d_gain(Kd_yaw)  # запись дк на курс
        keep_yaw.timer = time.time()


# ПД регулятор по глубине, !!! без ожидания будет выдавать ошибку !!!
def keep_depth(depth_to_set):
    try:
        time.sleep(0.001)
        # вычесление ошибки, действительное значение - заданное значение
        error = get_depth_correction() - depth_to_set
        # забиваем ошибку и получаем выходное значение на моторы
        output = keep_depth.regulator.process(error)
#        output += 30
#        print(output)
        # проверяем выходное значение на
        output = clamp(output, -50, 50)
        auv.set_motor_power(THRUSTER_DEPTH_LEFT,
                            output * DIRECTION_THRUSTER_DEPTH_LEFT)  # передаём выходное значение на мотор 2
        auv.set_motor_power(THRUSTER_DEPTH_RIGHT,
                            output * DIRECTION_THRUSTER_DEPTH_RIGHT)  # передаём выходное значение на мотор 3
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)  # запись пк на глубину
        keep_depth.regulator.set_d_gain(Kd_depth)  # запись дк на глубину


def motor_control_regulator(time_control, yaw_to_set, depth_to_set, speed_to_yaw=0.0, show_vid = True):
    # Функция управления моторами, принимает: time_control - время,
    # по умолчанию равное нулю, то есть работает один раз,
    # yaw_to_set - заданное значение курса,
    # speed_to_yaw - заданное значение скорости по курсу,
    if time_control > 0:
        time_new = time.time()
        while time_new + time_control > time.time():
            keep_yaw(yaw_to_set, speed_to_yaw)
            keep_depth(depth_to_set)
            if show_vid:
                img = get_img(0)
                view_img(img)
    else:
        keep_yaw(yaw_to_set, speed_to_yaw)
        keep_depth(depth_to_set)


def centering_r(yaw, y, depth=DEPTH, Kof_smooth=0.3, accuracy=50.0):
    y_center = (480 / 2) - y
    try:
        time.sleep(0.001)
        output_forward = centering_r.regulator_forward.process(y_center)
        output_forward = clamp(output_forward * Kof_smooth, -30, 30)
        print(y_center)
        keep_yaw(yaw, output_forward)
        keep_depth(depth)
        if abs(y_center) < accuracy:
            return True
    except AttributeError:
        centering_r.regulator_forward = PD()
        centering_r.regulator_forward.set_p_gain(0.8)
        centering_r.regulator_forward.set_d_gain(7)

    return False


def regul_distance(distance, time_control=2):
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
    if not abs(auv.get_yaw() - angle) < angle_control:
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


def view_img(img1, Low_hsv = ..., Max_hsv = ...):
    # перевод изображения из RGB в HSV формат.
    imageHSV = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    # бинаризация изображения.
    bin_spec = Low_hsv is not ... and Max_hsv is not ...
    img_bin = cv2.inRange(imageHSV, *colors_dict['orange']) if bin_spec else ...
    if SIMULATOR:
        cv2.imshow('Image', img1)
        cv2.imshow('Bin', img_bin) if bin_spec else ...

        cv2.waitKey(1)
    else:
        mur_view.show(img1, 0)
        mur_view.show(img_bin, 1) if bin_spec else ...


def defining_arrow(image):  # вычесление угла у стрелки путём нахождения наибольшой стороны
    # перевод изображения из RGB в HSV формат.
    draw = image.copy()
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # бинаризация изображения.
    img_bin = cv2.inRange(imageHSV, *colors_dict["orange"])
    # выделение контуров.
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnt:  # проверяем не пустой ли список контуров
        c = max(cnt, key=cv2.contourArea)  # берём наибольшую маску
        if cv2.contourArea(c) > 200:  # проверяем не мусор ли это
            c = cv2.convexHull(c)  # сглаживаем фигуру
            # аппроксимируем фигуры, (!!!)
            s_triangle, angle_arrow = cv2.minEnclosingTriangle(c)
            
            cv2.line(draw, angle_arrow[0, 0].astype(int), angle_arrow[1, 0].astype(int), (255, 0, 0), 2)
            cv2.line(draw, angle_arrow[1, 0].astype(int), angle_arrow[2, 0].astype(int), (255, 0, 0), 2)
            cv2.line(draw, angle_arrow[2, 0].astype(int), angle_arrow[0, 0].astype(int), (255, 0, 0), 2)

            x, y, w, h = cv2.boundingRect(angle_arrow)

            crop_img = img_bin[y:y+h, x:x+w]

            # getting triangle edges (sides)
            side1 = [angle_arrow[0, 0], angle_arrow[1, 0]]
            side2 = [angle_arrow[1, 0], angle_arrow[2, 0]]
            side3 = [angle_arrow[2, 0], angle_arrow[0, 0]]
            
            sides_n_ratios = []

            # searching for the base edge
            for side in [side1, side2, side3]:
                angle_side = np.degrees(np.arctan2(side[1][1] - side[0][1], side[1][0] - side[0][0]))

                rows, cols = crop_img.shape[:2]

                # preparing image for the rotating. So, the max width/height can be sqrt(2*a**2) after rotating.
                # image must be placed on the background because after rotation w and h not changes automatically, image crops.
                max_dim_hyp = math.ceil(max(rows, cols) * math.sqrt(2))
                
                if max_dim_hyp <= 0:
                    continue

                canvas_width = max_dim_hyp
                canvas_height = max_dim_hyp

                # making background image
                canvas = np.zeros((canvas_height, canvas_width), dtype=np.uint8)

                # getting center coordinates
                ix = int((canvas_width - crop_img.shape[1]) / 2)
                iy = int((canvas_height - crop_img.shape[0]) / 2)

                # placing image on the background
                canvas[iy:iy+crop_img.shape[0], ix:ix+crop_img.shape[1]] = crop_img

                # getting rotation matrix
                M = cv2.getRotationMatrix2D((max_dim_hyp / 2, max_dim_hyp / 2), angle_side, 1.0)

                # rotating
                rotated = cv2.warpAffine(canvas, M, (max_dim_hyp, max_dim_hyp))

                # next, cropping contour image
                ax, ay, aw, ah = cv2.boundingRect(rotated)
                rotated_crop = rotated[ay:ay+ah, ax:ax+aw]

                # splitting image on two sides to get ratio between all white pixels and left side white pixels and left side ones difference.
                left_rotated_crop = rotated_crop[:, :rotated_crop.shape[1] // 2]
                right_rotated_crop = rotated_crop[:, rotated_crop.shape[1] // 2:]

                # show(left_rotated_crop, True)
                # show(right_rotated_crop, True)

                # counting white pixels
                left_rotated_crop_1pixs = np.count_nonzero(left_rotated_crop)
                right_rotated_crop_1pixs = np.count_nonzero(right_rotated_crop)

                # calculating left and right side difference
                delta_pixs = abs(left_rotated_crop_1pixs - right_rotated_crop_1pixs)

                if (left_rotated_crop_1pixs + right_rotated_crop_1pixs) != 0:
                    # calculating ratio
                    ratio = delta_pixs / (left_rotated_crop_1pixs + right_rotated_crop_1pixs)
                else:
                    ratio = 0

                sides_n_ratios.append((ratio, side))

            if len(sides_n_ratios) <= 0:
                return False
            
            # getting edge with minimum ratio
            ratio, real_base = min(sides_n_ratios, key=lambda x: x[0])
            
            cv2.circle(draw, real_base[0].astype(int), 5, (255, 255, 255), 2)
            cv2.circle(draw, real_base[1].astype(int), 5, (255, 255, 255), 2)

            moments = cv2.moments(c)
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            
            angle = math.degrees(math.atan2(real_base[1][1] - real_base[0][1], real_base[1][0] - real_base[0][0]))

            if ((real_base[1][0] - real_base[0][0]) * (cy - real_base[0][1]) - (real_base[1][1] - real_base[0][1]) * (cx - real_base[0][0])) > 0:
                angle += 180
                print("add 180")
                
            # if ((real_base[1][1]-real_base[0][1])/(real_base[1][0]-real_base[0][0])<((real_base[1][1]-real_base[0][1])/(real_base[1][0]-real_base[0][0]))
        
            # if (math.atan2(real_base[1][1]-real_base[0][1],real_base[1][0]-real_base[0][0]))(np.atanh2(real_base[1][1]-real_base[0][1],real_base[1][0]-real_base[0][0]))<:
            #     pass
            # print("Agl", angle)
            
            return cx, cy, angle, draw
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


def angle_score(yaw, score=1):
    angle_sum = 0
    angle_sum_c = 0
    while True:
        img0 = get_img()
        data = defining_arrow(img0)
        motor_control_regulator(0, yaw, DEPTH, 0, show_vid=False)
        if data:
            mur_view.show(data[3], 1)
            angle_sum += data[2]
            angle_sum_c += 1
            if angle_sum_c >= score:
                print(data)
                print("Now angle takoi:", auv.get_yaw())
                return data[2]


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
            # print(data_ang)
            motor_control_regulator(0, data_ang, 0.5, 0)
            yaw = data_ang
            if regul_angle(data_ang, 15, 2):
                return data_ang
        else:
            motor_control_regulator(0, yaw, 0.5, 0)

def regul_angle_arrow(yaw):
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        view_img(img, *colors_dict['orange'])
        data = defining_arrow(img)
        # print("a")
        if data:
            data_ang = clamp_to180(np.degrees(np.arctan2(160 - data[0], 120 - data[1]))) + auv.get_yaw()
            print(data_ang)
                # calculate_angle_sin_vector(
                #     160 - data[0], 120 - data[1]) + auv.get_yaw()
            # print(data_ang)
            motor_control_regulator(0, data_ang, DEPTH, 0)
            yaw = data_ang
            if regul_angle(data_ang, 30, 1):
                return data_ang
        else:
            print("No arrow found :(")
            motor_control_regulator(0, yaw, DEPTH, 0)


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
            # print(data[0], n_error)
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

def regul_angle_arrow_2(yaw):
    time_new = None
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        data = defining_arrow(img)
        if data:
            error = img.shape[1] - data[0]
            n_error = error * 0.1
            print(n_error)
            yaw = error
            keep_depth(DEPTH)
            keep_yaw(0, error=error)
            view_img(img, *colors_dict['orange'])
            if -40 < n_error < 40:
                if time_new is None:
                    time_new = time.time()
                elif time.time() - time_new >= 2:
                    return auv.get_yaw()
            else:
                time_new = None
        else:
            auv.set_rgb_color(255, 255, 0)
            motor_control_regulator(0, yaw, DEPTH, 0)
            time_new = None
#        mur_view.show(0, img)


def regul_r_figure(yaw, hsv_min=Low_hsv_orange, hsv_max=Max_hsv_orange):
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        view_img(img)
        data = defining_figure(img, hsv_min, hsv_max)
        # print(data)
        if data:
            if regul_distance(centering_r(yaw, data[1])):
                return True


def regul_r_arrow(yaw):
    while True:
        img = get_img()
        # img = cv2.resize(img, (320, 240))
        data = defining_arrow(img)
        view_img(img)
        if data:
            if regul_distance(centering_r(yaw, data[1], DEPTH)):
                return True


def figure_search(yaw):
    while True:
        img = get_img()
        img = cv2.resize(img, (320, 240))
        data = defining_figure(img, Low_hsv_orange, Max_hsv_orange)
        data2 = defining_figure(img, *colors_dict["red"])
        # cv2.imshow("img", img)
        # cv2.waitKey(1)
        motor_control_regulator(0, yaw, 0.5, 20)
        if data:
            return True
        if data2:
            return "red"
        
def try_find_figure():
    img = get_img()
    img = cv2.resize(img, (320, 240))
    data = defining_arrow(img)
    data2 = defining_figure(img, *colors_dict["red"])
    view_img(img)
    if data:
        return True, 0
    if data2:
        return True, 1
    return False, -1

from colorsys import hsv_to_rgb

def rainbow_test():
    print()
    print("rainbow test")

    # turn off flashing
    auv.set_on_delay(1.0)
    auv.set_off_delay(0.0)

    frame_time = 0.01   # duration of frame in seconds
    counter = 0         # current frame counter
    counter_max = 4     # total duration of animation in seconds
    transition = 1      # duration of soft transition (start & end)

    while True:
        for led in range(13):
            # sotf transition
            value = min(counter, transition) - max(counter - (counter_max - transition), 0)
            # rotate HSV circle to draw rainbow
            rgb = hsv_to_rgb((led + counter) * 0.9, 1.0, value * 50)
            auv.set_single_led_color(led, rgb[0], rgb[1], rgb[2])

        counter += frame_time
        time.sleep(frame_time)

from threading import Thread

# Thread(target=rainbow_test, name="a").start()


Yaw_const = auv.get_yaw()
yaw = auv.get_yaw()

if __name__ == '__main__':
    success = False
    try:
        auv.set_rgb_color(255, 0, 255) if ROBOT else ...
        auv.set_off_delay(0) if ROBOT else ...
        time.sleep(2)
        Yaw_const = auv.get_yaw()
        auv.set_off_delay(0.5) if ROBOT else ...

        motor_control_regulator(5, Yaw_const, DEPTH, 0)
        print("Starting main")
        auv.set_rgb_color(0, 255, 0) if ROBOT else ...

        while True:
            keep_depth(DEPTH)
            keep_yaw(yaw)
            time.sleep(0.01)

            f, obj = try_find_figure()

            if obj == 0:
                print("Found arrow")

                auv.set_rgb_color(0, 255, 0) if ROBOT else ...
                yaw = regul_angle_arrow(yaw)
                auv.set_rgb_color(0, 0, 255) if ROBOT else ...
                print("Centered yaw")

                yaw = auv.get_yaw()
                regul_r_arrow(yaw)
                auv.set_rgb_color(39, 237, 201) if ROBOT else ...
                print("Centered r")

                DEPTH = GO_DEEPER_DEPTH
                yaw = auv.get_yaw()
                yaw -= angle_score(yaw)
                yaw = clamp_to180(yaw)
                print("angle:", yaw)

                motor_control_regulator(3, yaw, DEPTH, 0)

                motor_control_regulator(10, yaw, DEPTH, 30)

                # go_forward = True

                success = True
                break

    except Exception as err:
        print("ACHTUNG:", err) 
    finally:
        if not success:
            auv.set_rgb_color(255, 0, 0) if ROBOT else ...
            auv.set_on_delay(0.1) if ROBOT else ...
            auv.set_off_delay(0.1) if ROBOT else ...
        else:
            auv.set_rgb_color(255, 255, 255) if ROBOT else ...
            auv.set_on_delay(0.1) if ROBOT else ...
            auv.set_off_delay(0.1) if ROBOT else ...
        print("Going up")
        keep_depth(-1)
        time.sleep(10)
