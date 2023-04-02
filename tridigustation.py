from typing import Union
import pymurapi as mur
import time
import math
from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt

Kp_depth = 20  # кофецент пропорционального регулятора на глубину
Kd_depth = 5  # кофецент дифференциального регулятора на глубину
Kp_yaw = 1   # кофецент пропорционального регулятора на курс
Kd_yaw = 0.001  # кофецент дифференциального регулятора на курс

DISTANCE = 0.24

THRUSTER_YAW_RIGHT = 0
THRUSTER_YAW_LEFT = 1
THRUSTER_DEPTH_LEFT = 2
THRUSTER_DEPTH_RIGHT = 3

DIRECTION_THRUSTER_YAW_LEFT = +1
DIRECTION_THRUSTER_YAW_RIGHT = +1
DIRECTION_THRUSTER_DEPTH_LEFT = +1
DIRECTION_THRUSTER_DEPTH_RIGHT = +1


auv = mur.mur_init()


class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self, kp=0, kd=0):
        self._kp = kp
        self._kd = kd

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


def keep_yaw(yaw_to_set, speed_to_yaw=0,
             error=...):  # ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
    try:
        if error is ...:
            # вычесление ошибки, действительное значение - заданное значение
            error = yaw_to_set - auv.get_yaw()
            # error = clamp_to180(error)  # проверяем ошибку на ограничение
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
        # error = get_depth_correction() - depth_to_set
        error = auv.get_depth() - depth_to_set
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


# def triangulate(d1, d2, d3):
#     m = math.sqrt((2 * d1 ** 2 + 2 * d2 ** 2 - d3 ** 2) / 4)
#     cos = (m ** 2 + d2 ** 2 - d1 ** 2) / (m * d3)
#     print("cos:", cos)
#     alpha = 90 - math.degrees(math.acos(cos))
#     return alpha


def euclidean_distance(x1, y1, x2, y2):
    p1 = np.array((x1, y1))
    p2 = np.array((x2, y2))
    return np.linalg.norm(p1 - p2)

# Mean Square Error
# locations: [ (lat1, long1), ... ]
# distances: [ distance1, ... ]


def mse(x, locations, distances):
    mse = 0.0
    for location, distance in zip(locations, distances):
        distance_calculated = euclidean_distance(
            x[0], x[1], location[0], location[1])
        mse += math.pow(distance_calculated - distance, 2.0)
    return mse / len(distances)


def tri(locations, distances):
    # Initial point: the point with the closest distance
    min_distance = float('inf')
    closest_location = None
    for l, d in zip(locations, distances):
        # A new closest point!
        if d < min_distance:
            min_distance = d
            closest_location = l
    initial_location = closest_location

    # initial_location: (lat, long)
    # locations: [ (lat1, long1), ... ]
    # distances: [ distance1,     ... ]
    result = minimize(
        mse,                         # The error function
        initial_location,            # The initial guess
        args=(locations, distances),  # Additional parameters for mse
        method='L-BFGS-B',           # The optimisation algorithm
        options={
            'ftol': 1e-5,         # Tolerance
            'maxiter': 1e+7      # Maximum iterations
        })
    location = result.x
    return location


def get_angle(x1, y1, x2, y2):
    if y1 == y2:
        if x1 < x2:
            return 90
        else:
            return -90
    elif y1 < y2:
        return math.degrees(math.atan((x2 - x1) / (y2 - y1)))
    else:
        return math.degrees(math.atan((x2 - x1) / (y2 - y1)) + 180)


def get_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def stabilizate_value_by_time(timer: float, value_to_set: float, current_value: float, accuracy: float, time_to_keep_value: float) -> Union[float, bool]:
    """Stabilizate value to be in accuracy range for any time.

    Args:
        timer (float): Timer.
        value_to_set (float): Value must to be that.
        current_value (float): Current value
        accuracy (float): Accuracy range.
        time_to_keep_value (float): For this time value will be kept.

    Returns:
        Union[float, bool]: float - timer. Must be reset for this func after. bool - if stabilizated.
    """
    # print(value_to_set - accuracy, value_to_set + accuracy)
    if value_to_set - accuracy <= current_value and current_value <= value_to_set + accuracy:
        if time.time() > timer + time_to_keep_value:
            return timer, True
    else:
        timer = time.time()
    return timer, False


class Point:
    def __init__(self, x, y, name) -> None:
        self.x = x
        self.y = y
        self.name = name

    def calculate_attributes(self, robot_x, robot_y):
        self.angle = get_angle(robot_x, robot_y, self.x, self.y) % 360
        self.distance = get_distance(self.x, self.y, robot_x, robot_y)

    def __str__(self) -> str:
        return "Point(name=" + self.name + ", x=" + str(self.x) + ", y=" + str(self.y) + ", angle=" + str(self.angle) + ", distance=" + str(self.distance) + ")"


checked = []

points = {}
now = 0
pointsX = [0, 0, 0, 0, 0]
pointsY = [0, 0, 0, 0, 0]
n = ["1", "2", "3", "4", "5"]
last_sig = 0
while True:
    # функция возвращает сигнал с гидрофонов
    # и расстояние каждого из них до пингера
    # сигнал (номер пингера): tr, tl, fr
    # расстояние: d1, d2, d3
    # tr, d1 - гидрофон на переднем правом моторе аппарата
    # tl, d2 - на переднем левом моторе
    # fr, d3 - на заднем правом моторе

    tr, tl, fr, d1, d2, d3 = auv.get_hydrophone_signal()

    print(tr, tl, fr, " ", d1, "  ", d2, "  ", d3)
    if tr != 0:
        last_sig = tr
        if tr != now:
            points.clear()
            now = tr
        points.update({"tr": d1})
    if tl != 0:
        last_sig = tl
        if tl != now:
            points.clear()
            now = tl
        points.update({"tl": d2})
    if fr != 0:
        last_sig = fr
        if fr != now:
            points.clear()
            now = fr
        points.update({"fr": d3})

    ltr = points.get("tr", None)
    ltl = points.get("tl", None)
    lfr = points.get("fr", None)

    if ltr is not None and ltl is not None and lfr is not None:
        # a = triangulate(ltl, ltr, DISTANCE)
        # print("angle:", a)
        c = tri(((0.24, 0), (0, 0), (0.24, -0.24)), (ltr, ltl, lfr))
        a = get_angle(0.24 / 2, 0, c[0], c[1])
        if last_sig != 0:
            print("aaaa", last_sig)
            pointsX[last_sig - 1] = c[0]
            pointsY[last_sig - 1] = c[1]
        if all(pointsX):
            print("Detected angle:", a)
            fig, ax = plt.subplots()
            ax.scatter([*pointsX, 0], [*pointsY, 0])
            for i, txt in enumerate([*n, "ME"]):
                ax.annotate(txt, ([*pointsX, 0.24 / 2][i], [*pointsY, 0][i]))
            plt.show()

            points_ = []
            for x, y, ni in zip(pointsX, pointsY, n):
                newp = Point(x, y, ni)
                newp.calculate_attributes(0.24 / 2, 0)
                points_.append(newp)

            sorted_points = sorted(points_, key=lambda x: x.distance)
            # nearestp = min(points_, key=lambda x: x.distance)
            for i in sorted_points:
                if i.name not in checked:
                    nearestp = i
                    break

            if nearestp.distance <= 1.5:
                print("Dropping!", nearestp.name)
                checked.append(nearestp.name)
                auv.drop()
                points.clear()
                pointsX = [0, 0, 0, 0, 0]
                pointsY = [0, 0, 0, 0, 0]
                now = 0
                last_sig = 0
                continue

            print("Nearest point:", nearestp)
            print("Going to yaw.")

            timer = time.time()
            stabilized = False
            yaw = clamp_to180(auv.get_yaw() + nearestp.angle)
            while not stabilized:
                timer, stabilized = stabilizate_value_by_time(
                    timer, yaw, auv.get_yaw(), 2, 3)
                keep_depth(2.5)
                keep_yaw(yaw)
            print("Going forward")
            timer = time.time()
            while time.time() - timer < 5:
                keep_depth(2.5)
                keep_yaw(yaw, 50)

            print("Stopping")
            timer = time.time()
            while time.time() - timer < 7:
                keep_depth(2.5)
                keep_yaw(yaw, 0)

            time.sleep(2)
            print("Redetecting points.")
            points.clear()
            pointsX = [0, 0, 0, 0, 0]
            pointsY = [0, 0, 0, 0, 0]
            now = 0
            last_sig = 0

        print("Cooridnate:", c, "angle:", a)

    time.sleep(0.02)
