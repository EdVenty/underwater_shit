# Данный пример предназначен для тестирования моторов на аппарате!

# В данном примере мы подаем тягу на 4 мотора продолжительностью в 5 секунд.

# 0 - донная камера
# 1 - передняя камера

# id - http://10.3.141.1/
from typing import Any, List, Tuple, Union
import cv2
import pymurapi as mur
import time
import math
from numpy import ndarray


CAMERA_FRONT = 1
CAMERA_BOTTOM = 0
RIGHT_DEPTH_MOTOR = 2
LEFT_DEPTH_MOTOR = 3
RIGHT_YAW_MOTOR = 1
LEFT_YAW_MOTOR = 0
SCREEN_HEIGHT = 640
SCREEN_WIDTH = 480

Kp_depth = 150  # кофецент пропорционального регулятора на глубину
Kd_depth = 50  # кофецент дифференциального регулятора на глубину
Kp_yaw = 10  # кофецент пропорционального регулятора на курс
Kd_yaw = 1  # кофецент дифференциального регулятора на курс

low_hsw = (0, 0, 0)
# рамки бинаризации для выделение оранжевого цвета.
max_hsv = (180, 20, 255)

# рамки бинаризации для красный, синий, желтый, зеленый
Low_hsw = (0, 50, 50)
Max_hsv = (0, 200, 200)

# Low_hsw_red = (15, 50, 50)  # рамки бинаризации для красный
# Max_hsv_red = (45, 255, 255)

Low_hsw_red = (160, 20, 50)  # рамки бинаризации для желтый
Max_hsv_red = (180, 255, 255)

Low_hsw_blue = (97, 91, 0)
Max_hsv_blue = (179, 255, 255)  # рамки бинаризации для голубого цвета.

Low_hsw_black = (0, 0, 0)
Max_hsv_black = (180, 40, 255)  # рамки бинаризации для чёрного цвета.


class PD:
    """Base PD regulator for mur robot"""
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self, p: float = 0, d: float = 0) -> None:
        """Creates base PD regulator.

        Args:
            p (float, optional): Proportional coefficient. Defaults to 0.
            d (float, optional): Differential coefficient. Defaults to 0.
        """
        self.set_p_gain(p)
        self.set_d_gain(d)

    def set_p_gain(self, value: float):
        """Set proportional coefficient (gain)

        Args:
            value (float): Proportional coefficient (gain)
        """
        self._kp = value

    def set_d_gain(self, value: float):
        """Set differectial coefficient (gain)

        Args:
            value (float): Differectial coefficient (gain)
        """
        self._kd = value

    def process(self, error: float) -> float:
        """Process regulator and return value

        Args:
            error (float): Regulator error

        Returns:
            float: Value to set
        """
        timestamp = int(round(time.time() * 1000))  # в timestamp записываем
        # время(выраженное в секундах) и домножаем до милисекунд, round отбрасывает знаки после запятой
        output = self._kp * error + self._kd / \
            (timestamp - self._timestamp) * (error - self._prev_error)
        # вычесляем выходное значение на моторы по ПД регулятору и записываем в output
        self._timestamp = timestamp  # перезаписываем время
        self._prev_error = error  # перезаписываем ошибку
        return output


class Clamper:
    """Base clamper (limiter)"""

    def clamp_to180(self, angle: float) -> float:
        """Limit value for yaw between -180 and 180 angle.

        Ограничитель максимального зачения для курса.

        Args:
            angle (float): Current robot yaw

        Returns:
            float: Limited robot yaw
        """
        if angle > 180:
            return angle - 360
        if angle < -180:
            return angle + 360

        return angle

    def clamp(self, v: float, max_v: float, min_v: float) -> float:
        """Limit value between min and max values.

        Ограничитель максимального зачения для моторов.

        Args:
            v (float): Value to clamp
            max_v (float): Max value
            min_v (float): Min value

        Returns:
            float: Clamped value
        """
        if v > max_v:
            return max_v
        if v < min_v:
            return min_v
        return v


class Robot:
    """Base program for mur robot."""
    _kp_depth = 0  # кофецент пропорционального регулятора на глубину
    _kd_depth = 0  # кофецент дифференциального регулятора на глубину
    _kp_yaw = 0  # кофецент пропорционального регулятора на курс
    _kd_yaw = 0  # кофецент дифференциального регулятора на курс

    def __init__(self, is_simulator_mode: bool = False) -> None:
        """Base program for mur robot.

        Args:
            is_simulator_mode (bool, optional): If simulator mode active. Defaults to False.
        """
        self.auv = mur.mur_init()
        self.clamper = Clamper()
        self.simulator_mode = is_simulator_mode
        self.yaw_reg = PD(self._kp_yaw, self._kd_yaw)
        self.depth_reg = PD(self._kp_depth, self._kd_depth)
        self.passed_contours = []

        if not is_simulator_mode:
            self.cap0 = cv2.VideoCapture(0)
            self.cap1 = cv2.VideoCapture(1)
            self.mur_view = self.auv.get_videoserver()

    def set_regulators_coefficients(self, depth_p: float = None, depth_d: float = None, yaw_p: float = None, yaw_d: float = None):
        """Sets regulators coefficients. If None then won't be set, saves old values.

        Args:
            depth_p (float, optional): Depth regulator proportional coefficient. Defaults to None.
            depth_d (float, optional): Depth regulator differetial coefficient. Defaults to None.
            yaw_p (float, optional): Yaw regulator proportional coefficient. Defaults to None.
            yaw_d (float, optional): Yaw regulator differential coefficient. Defaults to None.
        """
        if depth_p is not None:
            self._kp_depth = depth_p
        if depth_d is not None:
            self._kd_depth = depth_d
        if yaw_p is not None:
            self._kp_yaw = yaw_p
        if yaw_d is not None:
            self._kd_yaw = yaw_d

    def get_depth_correction(self, k: float = 1) -> float:
        """Get correction for current robot's depth

        Args:
            k (float, optional): Correction coefficient. Defaults to 1.

        Returns:
            float: Correction
        """
        return self.auv.get_depth() + 0.2 * k

    # ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
    def keep_yaw(self, yaw_to_set: float, speed_to_yaw: float = 0) -> float:
        """Auto yaw regulator. Keeps yaw!

        Args:
            yaw_to_set (float): Yaw to set
            speed_to_yaw (float, optional): Additional speed to yaw. Defaults to 0.

        Returns:
            float: Value after regulator's procession
        """
        #        try:
        #            error = yaw_to_set - auv.get_yaw()  # вычесление ошибки, действительное значение - заданное значение
        #            error = clamp_to180(error)  # проверяем ошибку на ограничение
        #            output = keep_yaw.regulator.process(error)  # забиваем ошибку и получаем выходное значение на моторы
        #            output = clamp(output, 100, -100)  # проверяем выходное значение на ограничение
        #            auv.set_motor_power(1, clamp((speed_to_yaw - output), 50, -50))  # передаём выходное значение на мотор 0
        #            auv.set_motor_power(2, clamp((speed_to_yaw + output), 50, -50))  # передаём выходное значение на мотор 1
        #        except ZeroDivisionError:
        time.sleep(0.001)
        # вычесление ошибки, действительное значение - заданное значение
        error = yaw_to_set - self.auv.get_yaw()
        # проверяем ошибку на ограничение
        error = self.clamper.clamp_to180(error)
        # забиваем ошибку и получаем выходное значение на моторы
        output = self.yaw_reg.process(error)
        # проверяем выходное значение на ограничение
        output = self.clamper.clamp(output, 100, -100)
        self.auv.set_motor_power(RIGHT_YAW_MOTOR, self.clamper.clamp(
            (speed_to_yaw - output), 50, -50))  # передаём выходное значение на мотор 0
        self.auv.set_motor_power(LEFT_YAW_MOTOR, self.clamper.clamp(
            (speed_to_yaw + output), 50, -50))  # передаём выходное значение на мотор 1
        return output

    # ПД регулятор по глубине, !!! без ожидания будет выдавать ошибку !!!
    def keep_depth(self, depth_to_set: float, speed_to_depth: float = 1.0):
        try:
            # вычесление ошибки, действительное значение - заданное значение
            error = self.get_depth_correction() - depth_to_set
            # забиваем ошибку и получаем выходное значение на моторы
            output = self.depth_reg.process(error)
            # проверяем выходное значение на ограничение
            output = self.clamper.clamp(output * speed_to_depth, 25, -25)
            # передаём выходное значение на мотор 2
            self.auv.set_motor_power(0, output)
            # передаём выходное значение на мотор 3
            self.auv.set_motor_power(3, output)
        except ZeroDivisionError:
            time.sleep(0.001)
            # вычесление ошибки, действительное значение - заданное значение
            error = self.get_depth_correction() - depth_to_set
            # забиваем ошибку и получаем выходное значение на моторы
            output = self.depth_reg.process(error)
            # проверяем выходное значение на ограничение
            output = self.clamper.clamp(output * speed_to_depth, 25, -25)
            # передаём выходное значение на мотор 2
            self.auv.set_motor_power(LEFT_DEPTH_MOTOR, output)
            # передаём выходное значение на мотор 3
            self.auv.set_motor_power(RIGHT_DEPTH_MOTOR, output)

    def defining_figure(self, image):
        imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

        # бинаризация изображения. по цветам
        img_bin = cv2.inRange(
            imageHSV, self.Low_hsw_yellow, self.Max_hsv_yellow)

        # выделение контуров.
        c, _ = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if c:  # проверяем не пустой ли список контуров
            # берём наибольшую маску
            c = sorted(c, key=cv2.contourArea, reverse=True)[0]
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

    def stab_to_coords(self, x: float, y: float, min_length: float = 10.0) -> bool:
        """Stabilizates robot to a specified coordinates on screen

        Args:
            auv (MUR auv): MUR robot object
            x_reg (PD): X PD regulator
            y_reg (PD): Y PD regulator
            x (float): X center
            y (float): Y center 
            min_length (float): Min length to a stab point

        Returns:
            bool: If robot has reached the Sstab point (length to the point lower than min_length)
        """
        length = math.sqrt((x - SCREEN_WIDTH/2) ** 2 +
                        (y - SCREEN_HEIGHT/2) ** 2)  # length to a screen center
        if length < min_length:
            return True  # stabilized
        output_forward = self.depth_reg.process(
            y - SCREEN_HEIGHT/2)  # for vertical movement
        output_side = self.yaw_reg.process(x - SCREEN_WIDTH/2)  # for horizontal movement

        # clamps value between -50 and 50
        output_forward = self.clamper.clamp(output_forward, 50, -50)
        # clamps value between -50 and 50
        output_side = self.clamper.clamp(output_side, 50, -50)

        self.auv.set_motor_power(LEFT_DEPTH_MOTOR, -output_forward)  # set vertical motor power
        self.auv.set_motor_power(RIGHT_DEPTH_MOTOR, -output_forward)  # set vertical motor power
        self.auv.set_motor_power(LEFT_YAW_MOTOR, output_side)  # set horizontal (size) motor power
        self.auv.set_motor_power(RIGHT_YAW_MOTOR, -output_side)  # set horizontal (size) motor power
        return False  # not stabilized

    def motor_control_regulator(self, yaw_to_set: float, depth_to_set: float, speed_to_yaw: float = 0.0, time_to_keep: float = 0):
        """Keeps yaw and depth of robot

        Args:
            yaw_to_set (float): Yaw to keep
            depth_to_set (float): Depth to keep
            speed_to_yaw (float, optional): Speed to yaw. Defaults to 0.0.
            time_to_keep (float, optional): How long will be kept. If equals 0 then keeps 1 iteration. Defaults to 0.
        """
        if time_to_keep > 0:
            time_new = time.time()
            while time_new + time_to_keep > time.time():
                self.keep_yaw(yaw_to_set, speed_to_yaw)
                self.keep_depth(depth_to_set)
        else:
            self.keep_yaw(yaw_to_set, speed_to_yaw)
            self.keep_depth(depth_to_set)

    def centering_h(self, y, Kof_smooth=1, accuracy=4.0):
        y_center = y - (240 / 2)
        try:
            time.sleep(0.001)
            output_forward = self.centering_h.regulator_forward.process(
                y_center)
            output_forward = self.clamper.clamp(
                output_forward * Kof_smooth, 70, -70)

            self.auv.set_motor_power(0, -output_forward)
            self.auv.set_motor_power(3, -output_forward)

            if abs(math.sqrt(y_center ** 2)) < accuracy:
                return True

        except AttributeError:
            self.centering_h.regulator_forward = PD()
            self.centering_h.regulator_forward.set_p_gain(1)
            self.centering_h.regulator_forward.set_d_gain(0)
        return False

    def imshow(self, camera_id: int, frame) -> Union[int, Any]:
        """Displays cv2 frame. If simulator mode is active will be displayed in cv2 window. Else in robot camera window.

        Args:
            camera_id (int): Camera id. If in simulator mode camera_id will be name of window.
            frame (cv2 frame): Frame to display

        Returns:
            int | Any: int - if in simulator mode. The key pressed in cv2 window number. Any - if not in simulator mode. Anything from murview.show function. Let me know wat is it.
        """
        if self.simulator_mode:
            cv2.imshow(str(camera_id), frame)
            return cv2.waitKey(1)
        else:
            assert camera_id in [
                CAMERA_FRONT, CAMERA_BOTTOM], "Wrong camera id for robot or simulator!"
            return self.mur_view.show(frame, camera_id)

    def imread(self, camera_id: int) -> Tuple[bool, Union[ndarray, None]]:
        """Captures frame from camera.

        Args:
            camera_id (int): Camera id

        Return:
            (bool, np.ndarray | None): bool - is frame got, np.ndarray - frame or None
        """
        assert camera_id in [
            CAMERA_FRONT, CAMERA_BOTTOM], "Wrong camera id for robot or simulator!"
        if self.simulator_mode:
            if camera_id == CAMERA_FRONT:
                return True, self.auv.get_image_front()
            elif camera_id == CAMERA_BOTTOM:
                return True, self.auv.get_image_bottom()
        else:
            if camera_id == CAMERA_FRONT:
                return self.cap1.read()
            elif camera_id == CAMERA_BOTTOM:
                return self.cap0.read()

    def set_motor_power(self, motor_id: int, power: float):
        """Sets motor power

        Args:
            motor_id (int): Motor id
            power (float): Power value to set
        """
        self.auv.set_motor_power(motor_id, power)

    def rotate(self, angle: float, speed: float, delay: float=0.001, stabilize_after_rotation: bool = True, depth_to_keep: float = None, depth_speed_to_keep: float = 1):
        """`Blocks program!` Rotates robot horizontally. Can keep depth while rotating. 

        Args:
            angle (float): Angle to rotate. Angle adds to a current yaw. If angle > 0 then rotates to the right, else to the left. If angle == 0 then only keeps depth for a 1 iteration.
            speed (float): Speed to rotate with.
            delay (float): Delay between yaw corrections.
            stabilize_after_rotation (bool): Stibilize after rotate with yaw regulator.
            depth_to_keep (float, optional): Depth to keep. If None then won't be kept. Defaults to None.
            depth_speed_to_keep (float, optional): Depth speed to keep. Will work only if depth_to_keep != None. Defaults to 1.
        """
        if depth_to_keep is not None:
            self.keep_depth(depth_to_keep, depth_speed_to_keep)

        start_yaw = self.auv.get_yaw()
        need_yaw = self.clamper.clamp_to180(start_yaw + angle)
        # print("need", need_yaw)
        if angle == 0:
            return
        while not(need_yaw - 5 < self.auv.get_yaw() < need_yaw + 5):
            self.set_motor_power(LEFT_YAW_MOTOR if angle > 0 else RIGHT_YAW_MOTOR, speed)
            # print(self.auv.get_yaw())
            time.sleep(delay)
        if stabilize_after_rotation: #! NOT WORKING!!!
            while self.keep_yaw(start_yaw + angle) != 0: pass
        
    def auto_rotate360(self, speed: float):
        """Rotates robot for 360 degrees
        """
        depth = robot.auv.get_depth()
        self.rotate(180, speed, depth_to_keep=depth)
        self.rotate(180, speed, depth_to_keep=depth)

    def off_motors(self):
        """Sets all motors power to 0
        """
        robot.set_motor_power(LEFT_YAW_MOTOR, 0)
        robot.set_motor_power(RIGHT_YAW_MOTOR, 0)
        robot.set_motor_power(LEFT_DEPTH_MOTOR, 0)
        robot.set_motor_power(RIGHT_DEPTH_MOTOR, 0)

    def forward_backward(self, time_to_go: float, power: float, back_power: float):
        """`Blocking program` Robot goes forward and then backeard for a specified time.

        Args:
            time_to_go (float): Time to go forward and backward.
            power (float): Power for motors.
            back_power (float): Power for motors for backward movement.
        """
        robot.set_motor_power(LEFT_YAW_MOTOR, power)
        robot.set_motor_power(RIGHT_YAW_MOTOR, power)
        time.sleep(time_to_go)
        robot.set_motor_power(LEFT_YAW_MOTOR, -back_power)
        robot.set_motor_power(RIGHT_YAW_MOTOR, -back_power)
        time.sleep(time_to_go)

    def detect_contours(self, frame, hsv_low: Tuple[int, int, int], hsv_high: Tuple[int, int, int]) -> List:
        """Detects all contours

        Args:
            frame (cv2 frame): CV2 frame
            hsv_low (Tuple[int, int, int]): HSV low values
            hsv_high (Tuple[int, int, int]): HSV high values

        Returns:
            List: cv2 Contours
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, hsv_low, hsv_high)
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        return contours

    def center_on_contour(self, contour):
        rect = cv2.boundingRect(contour)
        while SCREEN_WIDTH - 10 < rect.x < SCREEN_WIDTH + 10 and SCREEN_HEIGHT - 10 < rect.y < SCREEN_HEIGHT + 10:
            self.keep_depth()

    def detect_left_unused_figure_and_center(self, hsv_params: List[Tuple[Tuple[int, int, int], Tuple[int, int, int], str]]) -> Tuple[bool, Any, str]:
        """Detects figures and centers on it.

        Args:
            hsv_params (List[Tuple[Tuple[int, int, int], Tuple[int, int, int], str]]): HSV params

        Returns:
            bool, Contour, str: Detected figures. bool - detected? Contour - cv2 contour. str - type of contour
        """
        frame = self.imread(CAMERA_FRONT)
        unsorted_contours = [[self.detect_contours(frame, hsv_low, hsv_high), color] for (hsv_low, hsv_high, color) in hsv_params]
        all_contours = filter(lambda data: cv2.contourArea(data[0]) > 100, [[[c, color] for c in grouped_contours] for grouped_contours, color in unsorted_contours])
        if len(all_contours): return False
        filtered_contours = filter(lambda obj: obj not in self.passed_contours, all_contours)
        left_contour = min(filtered_contours, key=lambda data: cv2.boundingRect(data[0])[0])
        rect = cv2.boundingRect(left_contour[0])
        while self.stab_to_coords(rect.x, rect.y): pass
        return True, left_contour[0], left_contour[1]

    def emerge(self):
        while True:
            self.keep_depth(0, 50)

if __name__ == '__main__':
    robot = Robot(True)
    robot.set_regulators_coefficients(Kp_depth, Kd_depth, Kp_yaw, Kd_yaw)

    while robot.detect_left_unused_figure_and_center(
        (
            (Low_hsw_blue, Max_hsv_blue, "blue")
        )
    ): print("centering...")
    # robot.auto_rotate360(40)
    # print("rotated!")
    # robot.off_motors()
    # robot.forward_backward(3, 100, 120)
    # print("forward-backward")
    # robot.off_motors()
    # robot.emerge()
    # print("emergerd!")

    # print("done!")
    # while True:
    #     # robot.motor_control_regulator(0, 3)
    #     # robot.rotate(50, 10, depth_to_keep=2, delay=0.001)
        
    #     # print(robot.get_depth_correction())
    #     _, img0 = robot.imread(CAMERA_FRONT)
    #     #        _, img1 = cap1.read()

    #     # перевод изображения из RGB в HSV формат.
    #     imageHSV = cv2.cvtColor(img0, cv2.COLOR_BGR2HSV)
    #     # бинаризация изображения.
    #     img_bin2 = cv2.inRange(
    #         imageHSV, robot.Low_hsw_blue, robot.Max_hsv_blue)

    #     robot.imshow(CAMERA_BOTTOM, img0)
    #     robot.imshow(CAMERA_FRONT, img_bin2)

        # mur_view.show((img_bin2), 1)
#        print(defining_figure(img1))

#        keep_yaw(50, 0)
#        keep_depth(0.1, 3.5)
#    motor_control_regulator(5, Yaw_const - 180, 0.1)
#    motor_control_regulator(5, Yaw_const - 180, 0.1, 40)


#    while True:
#        _, img0 = cap0.read()
#        _, img1 = cap1.read()
#
#        imageHSV = cv2.cvtColor(img1, cv2.COLOR_BGR2HLS)  # перевод изображения из RGB в HSV формат.
#        img_bin2 = cv2.inRange(imageHSV, Low_hsw_red, Max_hsv_red)  # бинаризация изображения.
#
#        mur_view.show(img1, 0)
#        mur_view.show((img_bin2), 1)
#        print(defining_figure(img1))

#        keep_yaw(50, 0)
#        keep_depth(0.1, 3.5)
