from typing import Union
import numpy as np
from numpy.__config__ import show
import aboba2 as boba
import cv2
import time
import pymurapi as mur
import time as t

class PD:
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self, kp: float = ..., kd: float = ...):
        if kp is not ...:
            self.set_p_gain(kp)
        if kd is not ...:
            self.set_d_gain(kd)

    def set_p_gain(self, value):
        self._kp = value

    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(t.time() * 1000))  # в timestamp записываем
        try:
            # время(выраженное в секундах) и домножаем до милисекунд, round отбрасывает знаки после запятой
            output = self._kp * error + self._kd / \
                (timestamp - self._timestamp) * (error - self._prev_error)
            # вычесляем выходное значение на моторы по ПД регулятору и записываем в output
            self._timestamp = timestamp  # перезаписываем время
            self._prev_error = error  # перезаписываем ошибку
            return output
        except ZeroDivisionError:
            return 0

class Functions:
    @staticmethod
    def opencv_show_func(frame, window_name: str = "window"):
        cv2.imshow(window_name, frame)
        cv2.waitKey(1)

    @staticmethod
    def simulator_get_front_frame_func(auv: 'SmartAUV'):
        return auv.get_image_front()

    @staticmethod
    def simulator_get_bottom_frame_func(auv: 'SmartAUV'):
        return auv.get_image_bottom()

    @staticmethod
    def clamp(v: float, min_v: float, max_v: float) -> float: 
        """Clamp value between two ranges.

        Args:
            v (float): Value
            min_v (float): Min value
            max_v (float): Max value

        Returns:
            float: Clamped value
        """
        if v > max_v:
            return max_v
        if v < min_v:
            return min_v
        return v
    
    @staticmethod
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
            if t.time() > timer + time_to_keep_value:
                return timer, True
        else:
            timer = t.time() 
        return timer, False


class SmartAUV(mur.simulator.Simulator):
    def __init__(self, screen_size = (320, 240), 
                 get_frame_func = Functions.simulator_get_front_frame_func, 
                 show_frame_func = Functions.opencv_show_func,
                 yaw_regulator = PD(1, 0.001),
                 depth_regulator = PD(10, 5),
                 prepare: bool = True):
        super().__init__()
        self.screen_size = screen_size
        # self.get_frame_func = lambda x: np.zeros((screen_size[1], screen_size[0], 3), np.uint8)
        self.set_get_frame_func(get_frame_func)
        self.set_show_frame_func(show_frame_func)
        self.yaw_pd = yaw_regulator
        self.depth_pd = depth_regulator
        self.THRUSTER_YAW_LEFT = 0
        self.THRUSTER_YAW_RIGHT = 1
        self.THRUSTER_DEPTH_LEFT = 2
        self.THRUSTER_DEPTH_RIGHT = 3

        self.THRUSTER_YAW_LEFT_DIRECTION = +1
        self.THRUSTER_YAW_RIGHT_DIRECTION = +1
        self.THRUSTER_DEPTH_LEFT_DIRECTION = +1
        self.THRUSTER_DEPTH_RIGHT_DIRECTION = +1

        if prepare:
            self.prepare()
    # @property
    # def yaw(self):
    #     return self.get_yaw()
    # @property
    # def depth(self):
    #     return self.get_depth()
    def set_thrusters(self,
                      thruster_yaw_left_num: int = ...,
                      thruster_yaw_right_num: int = ...,
                      thruster_depth_left_num: int = ...,
                      thruster_depth_right_num: int = ...,
                      thruster_yaw_left_direction: int = ...,
                      thruster_yaw_right_direction: int = ...,
                      thruster_depth_left_direction: int = ...,
                      thruster_depth_right_direction: int = ...):
        if thruster_yaw_left_num is not ...:
            self.THRUSTER_YAW_LEFT = thruster_yaw_left_num
        if thruster_yaw_right_num is not ...:
            self.THRUSTER_YAW_RIGHT = thruster_yaw_right_num
        if thruster_depth_left_num is not ...:
            self.THRUSTER_DEPTH_LEFT = thruster_depth_left_num
        if thruster_depth_right_num is not ...:
            self.THRUSTER_DEPTH_RIGHT = thruster_depth_right_num

        if thruster_yaw_left_direction is not ...:
            self.THRUSTER_YAW_LEFT_DIRECTION = thruster_yaw_left_direction
        if thruster_yaw_right_direction is not ...:
            self.THRUSTER_YAW_RIGHT_DIRECTION = thruster_yaw_right_direction
        if thruster_depth_left_direction is not ...:
            self.THRUSTER_DEPTH_LEFT_DIRECTION = thruster_depth_left_direction
        if thruster_depth_right_direction is not ...:
            self.THRUSTER_DEPTH_RIGHT_DIRECTION = thruster_depth_right_direction
    def set_get_frame_func(self, func):
        self.get_frame_func = func
    def set_show_frame_func(self, func):
        self.show_frame_func = func
    def stab_by_arrow(self, arrow_color_range: boba.ColorRange, time_to_stab: float = 5, accuracy: float = 5, show_video: bool = True, draw_arrows: bool = True, depth_to_keep: float = 2) -> float:
        timer = time.time()
        while True:
            # frame = boba.auv.get_image_bottom()
            frame = self.get_frame_func(self)
            # print(frame.shape)
            draw = frame.copy()
            f = boba.find_figures(frame, boba.FiguresSearchParams([arrow_color_range])).figures
            triangle_p = f.where(boba.Figure.color_name == 'orange', boba.Figure.shape == boba.ShapeType.TRIANGLE).max(boba.Figure.cnt_area)
            if not triangle_p.is_bad_index:
                f_triangle = triangle_p.to_value()
                if draw_arrows:
                    f_triangle.draw_figure(draw, thickness=3)
                if f_triangle.define_arrow_angle():
                    print(f_triangle.arrow_angle)
                    timer, stabilizated = Functions.stabilizate_value_by_time(timer, self.get_yaw() + f_triangle.arrow_angle, self.get_yaw(), accuracy, time_to_stab)
                    if stabilizated:
                        # print("Stabbed!") 
                        return
                        # boba.keep(boba.auv.get_yaw() - f_triangle.arrow_angle, 3, time)
                    else:
                        self.keep(self.get_yaw() - f_triangle.arrow_angle, depth_to_keep, time=0.01)
            if show_video:
                self.show_frame_func(draw)
    def keep_yaw(self, yaw_to_set: float, move_speed: float = 0):
        """Keep robot yaw and move forward/backward.

        Args:
            yaw_to_set (float): Yaw to set
            move_speed (float, optional): Speed to move forward. Defaults to 0.
        """
        err = Functions.clamp(self.yaw_pd.process(yaw_to_set - self.get_yaw()), -100, 100)
        self.set_motor_power(self.THRUSTER_YAW_LEFT, move_speed+err*self.THRUSTER_YAW_LEFT_DIRECTION)
        self.set_motor_power(self.THRUSTER_YAW_RIGHT, move_speed-err*self.THRUSTER_YAW_RIGHT_DIRECTION)
        t.sleep(0.1)

    def keep_depth(self, depth_to_det: float):
        """Keep robot depth.

        Args:
            depth_to_det (float): Depth to set.
        """
        err = Functions.clamp(self.depth_pd.process(depth_to_det - self.get_depth()), -100, 100)
        self.set_motor_power(self.THRUSTER_DEPTH_LEFT, -err*self.THRUSTER_DEPTH_LEFT_DIRECTION)
        self.set_motor_power(self.THRUSTER_DEPTH_RIGHT, -err*self.THRUSTER_DEPTH_RIGHT_DIRECTION)
        t.sleep(0.1)

    def keep(self, yaw_to_set: float = ..., depth_to_set: float = ..., move_speed: float = 0, time: float = 0):
        """Keep depth, yaw and move robot forward/backward.

        Args:
            yaw_to_set (float, optional): Yaw to set. Defaults to ....
            depth_to_set (float, optional): Depth to set. Defaults to ....
            move_speed (float, optional): Move forward speed. Defaults to 0.
            time (float, optional): Time to keep. Defaults to 0.
        """
        timer = t.time() + time
        while t.time() < timer:
            if yaw_to_set is not ...:
                self.keep_yaw(yaw_to_set, move_speed)
            else:
                self.set_motor_power(self.THRUSTER_YAW_LEFT, move_speed*self.THRUSTER_YAW_LEFT_DIRECTION)
                self.set_motor_power(self.THRUSTER_YAW_RIGHT, -move_speed*self.THRUSTER_YAW_RIGHT_DIRECTION)
            if depth_to_set is not ...:
                self.keep_depth(depth_to_set)
            t.sleep(0.01)
    def move_to_color(self, color_range: boba.ColorRange, delay_time: int = 0, depth_to_keep: float = 3, yaw_to_keep: float = ..., forward_speed: float = 50, show_frame: bool = True):
        timer = time.time()
        if yaw_to_keep is ...:
            yaw_to_keep = self.get_yaw()
        while True:
            frame = self.get_frame_func(self)
            figures = boba.find_figures(frame, boba.FiguresSearchParams([color_range])).figures
            pointer = figures.where(boba.Figure.color_name == 'orange', boba.Figure.shape == boba.ShapeType.TRIANGLE).max(boba.Figure.cnt_area)
            if not pointer.is_bad_index:
                pointer.to_value().draw_figure(frame)
            timer, stabilizated = Functions.stabilizate_value_by_time(timer, 1, 0 if pointer.is_bad_index else 1, accuracy=0, time_to_keep_value=delay_time)
            if stabilizated:
                return
            else:
                self.keep(yaw_to_keep, depth_to_keep, time=0.01, move_speed=forward_speed)
            if show_frame:
                self.show_frame_func(frame)
    def forward(self, time: float, yaw_to_keep: float = ..., depth_to_keep: float = ..., speed: float = 50, show_frame: bool = True):
        timer = t.time() + time
        if yaw_to_keep is ...:
            yaw_to_keep = self.get_yaw()
        if depth_to_keep is ...:
            depth_to_keep = self.get_depth()
        while True:
            if show_frame:
                self.show_frame_func(self.get_frame_func(self))
            self.keep(yaw_to_keep, depth_to_keep, speed, 0.01)
            if t.time() > timer:
                return


orange = boba.ColorRange(boba.Color(10, 20, 20), boba.Color(25, 255, 255), "orange")
auv = SmartAUV(get_frame_func=Functions.simulator_get_bottom_frame_func)
auv.