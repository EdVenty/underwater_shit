import copy
import math
import time as t
from enum import Enum
from typing import Any, List, Set, Tuple, Union

import cv2
import numpy as np
import pymurapi as mur


class Convert:
    @staticmethod
    def list_elements_to(list: List, type: Any) -> List[Any]:
        return [type(i) for i in list]

class Maath:
    @staticmethod
    def vectors2_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return abs(math.sqrt(
            (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        ))
    @staticmethod
    def vectors2_triangle_center(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> Tuple[float, float]:
        return (
            (a[0] + b[0] + c[0]) / 3,
            (a[1] + b[1] + c[1]) / 3,
        )
    @staticmethod
    def vectors2_square_center(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float], d: Tuple[float, float]) -> Tuple[float, float]:
        return (
            int((a[0] + b[0] + c[0] + d[0]) / 4),
            int((a[1] + b[1] + c[1] + d[1]) / 4),
        )
    @staticmethod
    def vectors2_two_center(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
        return (
            int((a[0] + b[0]) / 2),
            int((a[1] + b[1]) / 2),
        )
    @staticmethod
    def vectors2_angle_sin(a, b):
        # вычесляем угол
        x2 = 0 - 0
        y2 = 0 - 240
        try:
            angle = int(math.asin((a * y2 - b * x2) /
                                (((a ** 2 + b ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
        except ZeroDivisionError:
            angle = int(math.asin((a * y2 - b * x2) /
                                (((a ** 2 + b ** 2) ** 0.5 + 0.000001) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
        return angle

    @staticmethod
    def vectors2_angle_cos(a, b):
        x2 = 0 - 0
        y2 = 0 - 240
        # вычесляем угол
        angle = int(math.acos((a * x2 + b * y2) /
                            (((a ** 2 + b ** 2) ** 0.5) * ((x2 ** 2 + y2 ** 2) ** 0.5))) * 180 / math.pi)
        return angle
    @staticmethod
    def vectors2_angle(a, b):
        angle_ = Maath.vectors2_angle_cos(a, b)
        angle = Maath.vectors2_angle_sin(a, b)
        try:
            angle = angle_ * (angle / abs(angle))
        except ZeroDivisionError:
            pass
        return angle

class ShapeType(Enum):
    CIRCLE = 0
    RECTANGLE = 1
    TRIANGLE = 2
    UNKNOWN = 3
    SQUARE = 4

class Color:
    def __init__(self, x: float, y: float, z: float, name: str = ...) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.z = z
    def to_tuple(self) -> Tuple:
        return (self.x, self.y, self.z)

class CompareType(Enum):
    EQUALS = 0
    GREATER = 1
    GREATER_EQUALS = 2
    LOWER = 3
    LOWER_EQUALS = 4
    NOT_EQUALS = 5

class PropertyCompare:
    def __init__(self, name: str, value: Any, compare_type: CompareType) -> None:
        self.name = name
        self.value = value
        self.compare_type = compare_type

class Property:
    def __init__(self, name: str, type: Any = ...) -> None:
        self.name = name
    def __eq__(self, o: Any) -> PropertyCompare: # ==
        return PropertyCompare(self.name, o, CompareType.EQUALS)
    def __gt__(self, o: Any) -> PropertyCompare: # >
        return PropertyCompare(self.name, o, CompareType.GREATER)
    def __ge__(self, o: Any) -> PropertyCompare: # >=
        return PropertyCompare(self.name, o, CompareType.GREATER_EQUALS)
    def __lt__(self, o: Any) -> PropertyCompare: # <
        return PropertyCompare(self.name, o, CompareType.LOWER)
    def __le__(self, o: Any) -> PropertyCompare: # <=
        return PropertyCompare(self.name, o, CompareType.LOWER_EQUALS)
    def __ne__(self, o: Any) -> PropertyCompare: # !=
        return PropertyCompare(self.name, o, CompareType.NOT_EQUALS)
    # def __or__(self, o: 'Property') -> PropertyCompare:


class Figure:
    shape = Property('shape')
    color_range = Property('color_range')
    color_name = Property('color_name')
    center = Property('center')
    center_x = Property('center_x')
    center_y = Property('center_y')
    rect_area = Property('rect_area')
    circle_area = Property('circle_area')
    triangle_area = Property('triangle_area')
    moments = Property('moments')
    rect_box = Property('rect_box')
    triangle_box = Property('triangle_box')
    circle_radius = Property('circle_radius')
    cnt = Property('cnt')
    cnt_area = Property('cnt_area')
    arrow_angle = Property('arrow_angle')

    def __init__(self, shape: ShapeType = ShapeType.UNKNOWN, 
                 color_range: 'ColorRange' = None, 
                 color_name: str = None,
                 center: Tuple[int, int] = None,
                 cnt: np.ndarray = None, 
                 cnt_area: float = None) -> None:
        self.shape = shape
        self.color_range = color_range
        self.color_name = color_name
        self.center = center
        
        self.center_x = None
        self.center_y = None
        if self.center is not None:
            self.center_x = center[0]
            self.center_y = center[1]
        
        self.rect_area = None
        self.circle_area = None
        self.triangle_area = None
        self.moments = None

        self.rect_box = None
        self.triangle_box = None
        self.circle_radius = None

        self.cnt = cnt
        self.cnt_area = cnt_area

        self.arrow_angle = None

    def define_moments(self, cnt: np.ndarray = ...):
        if cnt is not ...:
            self.cnt = cnt
        self.moments = cv2.moments(self.cnt)
       
    def define_center(self, cnt: np.ndarray = ...) -> bool:
        if cnt is not ...:
            self.cnt = cnt
            self.define_moments()
        try:
            x = int(self.moments["m10"] / self.moments["m00"])  
            y = int(self.moments["m01"] / self.moments["m00"])
            self.center = (x, y)
            return True
        except ZeroDivisionError:
            return False
        
    def define_shape(self, cnt: np.ndarray = ...):
        if cnt is not ...:
            self.cnt = cnt

        (cX, xY), cR = cv2.minEnclosingCircle(self.cnt)
        s_triangle, triangle = cv2.minEnclosingTriangle(self.cnt)
        rect = cv2.minAreaRect(self.cnt)
        ((_, _), (w, h), _) = rect

        rectangle_area = w * h
        circle_area = cR ** 2 * math.pi

        if rectangle_area > circle_area < s_triangle:
            self.shape = ShapeType.CIRCLE
            self.circle_radius = cR
        elif circle_area > rectangle_area < s_triangle:
            if 0.8 < w / h < 1.2:
                self.shape = ShapeType.SQUARE
            else:
                self.shape = ShapeType.RECTANGLE
            self.rect_box = rect
        elif rectangle_area > s_triangle < circle_area:
            self.shape = ShapeType.TRIANGLE
            self.triangle_area = s_triangle
            self.triangle_box = triangle

        self.rect_area = rectangle_area
        self.triangle_area = s_triangle
        self.circle_area = circle_area

        self.center_x = self.center[0]
        self.center_y = self.center[1]
        
        return True

    def draw_contour(self, rgb: np.ndarray, color: Tuple[int, int, int] = (0, 0, 0), thickness=1) -> np.ndarray:
        cv2.drawContours(rgb, [self.cnt], 0, color, thickness=thickness)
    def draw_rect(self, rgb: np.ndarray, color: Tuple[int, int, int] = (0, 0, 0), thickness=1) -> np.ndarray:
        box = cv2.boxPoints(self.rect_box)
        box = np.int0(box)
        cv2.drawContours(rgb, [box], 0, color, thickness)
    def draw_circle(self, rgb: np.ndarray, color: Tuple[int, int, int] = (0, 0, 0), thickness=1) -> np.ndarray:
        cv2.circle(rgb, (int(self.center[0]), int(self.center[1])), int(self.circle_radius), color, thickness)
    def draw_triangle(self, rgb: np.ndarray, color: Tuple[int, int, int] = (0, 0, 0), thickness=1) -> np.ndarray:
        cv2.line(rgb, tuple(self.triangle_box[0][0]), tuple(self.triangle_box[1][0]), color, thickness)
        cv2.line(rgb, tuple(self.triangle_box[1][0]), tuple(self.triangle_box[2][0]), color, thickness)
        cv2.line(rgb, tuple(self.triangle_box[0][0]), tuple(self.triangle_box[2][0]), color, thickness)
    def draw_figure(self, rgb: np.ndarray, color: Tuple[int, int, int] = (0, 0, 0), thickness=1) -> np.ndarray:
        if self.shape == ShapeType.CIRCLE:
            self.draw_circle(rgb, color, thickness)
        elif self.shape == ShapeType.RECTANGLE or self.shape == ShapeType.SQUARE:
            self.draw_rect(rgb, color, thickness)
        elif self.shape == ShapeType.TRIANGLE:
            self.draw_triangle(rgb, color, thickness)
    def define_arrow_angle(self):
        angle_arrow = self.triangle_box
        W1 = (((angle_arrow[1, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
                  + ((angle_arrow[1, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5

        W2 = (((angle_arrow[0, 0, 0] - angle_arrow[2, 0, 0]) ** 2)
                + ((angle_arrow[0, 0, 1] - angle_arrow[2, 0, 1]) ** 2)) ** 0.5

        W3 = (((angle_arrow[1, 0, 0] - angle_arrow[0, 0, 0]) ** 2)
                + ((angle_arrow[1, 0, 1] - angle_arrow[0, 0, 1]) ** 2)) ** 0.5

        if W3 < W1 > W2:
            coordinate = [[angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]],
                            [angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]],
                            [angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]]]
        elif W3 < W2 > W1:
            coordinate = [[angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]],
                            [angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]],
                            [angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]]]
        elif W1 < W3 > W2:
            coordinate = [[angle_arrow[2, 0, 0], angle_arrow[2, 0, 1]],
                            [angle_arrow[0, 0, 0], angle_arrow[0, 0, 1]],
                            [angle_arrow[1, 0, 0], angle_arrow[1, 0, 1]]]
        else:
            return False
        # вычисления кооринат середины наибольшой стороны треугольника
        x_centre_arrow = (coordinate[1][0] + coordinate[2][0]) // 2
        y_centre_arrow = (coordinate[1][1] + coordinate[2][1]) // 2
        # вычисление векторов для расчёта угла стрелки
        x1 = coordinate[0][0] - x_centre_arrow
        y1 = coordinate[0][1] - y_centre_arrow
        # вычисляем угол
        angle = Maath.vectors2_angle(x1, y1)
        self.arrow_angle = angle
        return True

class ColorRange:
    def __init__(self, min_color: Color = ...,
                 max_color: Color = ...,
                 name: str = ...,) -> None:
        self.name = name
        self.min_color = min_color
        self.max_color = max_color

        self.min_color.name = name
        self.max_color.name = name

class FiguresSearchParams:
    def __init__(self, color_ranges: List[ColorRange] = ...,
                 contours_find_mode: int = cv2.RETR_EXTERNAL,
                 contours_find_method: int = cv2.CHAIN_APPROX_SIMPLE,
                 min_contour_area: float = 0) -> None:
        self.color_ranges = color_ranges
        self.contours_mode = contours_find_mode
        self.min_contour_area = min_contour_area
        self.contours_find_method = contours_find_method

        
class FiguresSearchResult:
    def __init__(self, figures: List[Figure] = None) -> None:
        if figures is not None:
            self.raw_figures = figures
        self.figures = FiguresList(self.raw_figures)

    def has_any_figure(self) -> bool:
        return len(self.figures) > 0

class FiguresListPointer:
    def __init__(self, index: int, figures_list: 'FiguresList', got: bool = True) -> None:
        self.index = index
        self.figures_list = figures_list
        self.got = got
    def to_value(self) -> Figure:
        return self.figures_list._figures[self.index]
    def to_value_or_default(self, default: Any = None) -> Union[Figure, Any]:
        if not self.got:
            return default
        return self.to_value()
    @property
    def is_bad_index(self) -> bool:
        return True if self.index < 0 else False

class FunctionResult:
    def __init__(self, function, args = [], kwargs = {}, calc: bool = False) -> None:
        self.function = function
        self.args = args
        self.kwargs = kwargs
        if calc:
            self.calculate()
        else:
            self.result = None
            self.calculated = False
    def calculate(self):
        self.result = self.function(*self.args, **self.kwargs)
        self.calculated = True
    def get_args(self) -> List: 
        return self.args
    def __str__(self) -> str:
        return "<FunctionResult, calculated={}, result={}, function={}, args={}, kwargs={}>".format(self.calculated, self.result, self.function, self.args, self.kwargs)
    def __repr__(self) -> str:
        return self.__str__()

# class FunctionResultsList:
#     def __init__(self, function_results: List[FunctionResult]) -> None:
#         self._list = function_results
#     def max(self, func = ...) -> FunctionResult:
#         if func is None:
#             func = lambda x: x.result
#         return 

def unpoint(figures_list: 'FiguresList'):
    for p in figures_list:
        yield p.to_value()

def extract(figures_list: 'FiguresList'):
    for p in figures_list:
        yield (p, p.to_value())

class FiguresList:
    def __init__(self, figures_list: List[Figure], sub_lists: List['FiguresList'] = [], function_results: List[FunctionResult] = []) -> None:
        self._figures = figures_list
        # self._sub_lists = sub_lists
        # self.zipped = None
        # self.function_results = function_results
    def where(self, *args: PropertyCompare, **kwargs) -> 'FiguresList':
        return FiguresList([i for i in self._figures if (
            all([(
                (getattr(i, j.name) == j.value) if j.compare_type == CompareType.EQUALS else 
                (getattr(i, j.name) > j.value) if j.compare_type == CompareType.GREATER else 
                (getattr(i, j.name) >= j.value) if j.compare_type == CompareType.GREATER_EQUALS else 
                (getattr(i, j.name) < j.value) if j.compare_type == CompareType.LOWER else 
                (getattr(i, j.name) <= j.value) if j.compare_type == CompareType.LOWER_EQUALS else 
                (getattr(i, j.name) != j.value) if j.compare_type == CompareType.NOT_EQUALS else False
            ) for j in args])
        )])
    def max(self, key: Property) -> FiguresListPointer:
        if len(self._figures) <= 0: return FiguresListPointer(-1, self, False)
        return FiguresListPointer(self._figures.index(max(self._figures, key=lambda x: getattr(x, key.name))), self)
    def min(self, key: Property) -> FiguresListPointer:
        if len(self._figures) <= 0: return FiguresListPointer(-1, self, False)
        return FiguresListPointer(self._figures.index(min(self._figures, key=lambda x: getattr(x, key.name))), self)
    def sort(self, key: Property, reverse = False) -> 'FiguresList':
        return FiguresList(list(sorted(self._figures, key=lambda x: getattr(x, key.name), reverse=reverse)))
    def to_list(self) -> List[Figure]:
        return self._figures
    def __len__(self) -> int:
        return len(self._figures)
    def append(self, fig: Figure) -> 'FiguresList':
        self._figures.append(fig)
        return self
    def first_or_default(self, *args: PropertyCompare) -> FiguresListPointer:
        for i, el in enumerate(self._figures):
            if all([(
                (getattr(el, j.name) == j.value) if j.compare_type == CompareType.EQUALS else 
                (getattr(el, j.name) > j.value) if j.compare_type == CompareType.GREATER else 
                (getattr(el, j.name) >= j.value) if j.compare_type == CompareType.GREATER_EQUALS else 
                (getattr(el, j.name) < j.value) if j.compare_type == CompareType.LOWER else 
                (getattr(el, j.name) <= j.value) if j.compare_type == CompareType.LOWER_EQUALS else 
                (getattr(el, j.name) != j.value) if j.compare_type == CompareType.NOT_EQUALS else False
            ) for j in args]):
                return FiguresListPointer(i, self)
        return FiguresListPointer(-1, self, False)
    def pop(self, index: int) -> 'FiguresList':
        self._figures.pop(index)
        return self
    def remove(self, fig: Figure) -> 'FiguresList':
        self._figures.remove(fig)
        return self
    def __getitem__(self, index: int) -> FiguresListPointer:
        size = self.length
        return FiguresListPointer(index, self, index < size)
    # def __setitem__(self, index: int, fig: Figure) -> None:
    #     self._figures[index] = fig

    def to_tuple(self) -> Tuple[Figure]:
        return tuple(self._figures)

    def __iter__(self):
        for (i, el) in enumerate(self._figures):
            yield FiguresListPointer(i, self)

    @property
    def length(self):
        return self.__len__()

    # def with_other(self, other_list: 'FiguresList') -> 'FiguresList':
    #     self._sub_lists.append(other_list)
    #     return self

    # def zip(self, dims: Set[int] = ...):
    #     if dims is not ...:
    #         self.zipped = dims
    #     else:
    #         self.zipped = {i for i in range(len(self._sub_lists) + 1)}
    #     return self

    # def _zzip(self):
    #     zziped = []
    #     for el in self._figures:
    #         zziped.append([el])
    #     for i, el in enumerate(self._sub_lists):
    #         if i > 0:
    #             for el2 in unpoint(el):
    #                 zziped[i].append(el2)
    #         # else:
    #         #     for el2 in unpoint(self):
    #         #         zziped[i].append(el2)
    #     return zziped

    # def exec_func(self, func):
    #     z = self._zzip()
    #     for elements in z:
    #         self.function_results.append(FunctionResult(func, elements))
    #     return self

    # def calc_results(self):
    #     for f in self.function_results:
    #         f.calculate()
    #     return [f.result for f in self.function_results]

    def copy(self) -> 'FiguresList':
        return copy.deepcopy(self)

    def compose(self, lists: List['FiguresList']) -> 'FiguresListComposition':
        allllllll = [self, *lists]
        return FiguresListComposition(allllllll)

    # def self_unique_compose(self) -> 'FiguresListUniqueComposition':
    #     ...

class FiguresListComposition:
    def __init__(self, fig_lists: List[FiguresList]) -> None:
        self._lists = fig_lists
    def _recursive_list(self, lists: List[FiguresList]):
        if len(lists) == 1:
            return [[i.to_value()] for i in lists[0]]
        l = []
        for other in self._recursive_list(lists[1:]):
            for i in lists[0]:
                l.append((i.to_value(), *other))
        return l
    def exec_func(self, func, calc: bool = True, unpoint: bool = True) -> List[FunctionResult]:
        a = self._recursive_list(self._lists)
        # new = []
        # for elements in zip(*self._lists):
        #     if unpoint:
        #         elements = [i.to_value() for i in elements]
        #     new.append(FunctionResult(func, elements, calc=calc))
        # return new
        return [FunctionResult(func, i, calc=calc) for i in a]

# class FiguresListUniqueComposition:
#     def _recursive_list(self, lists: List[FiguresList]):
#         if len(lists) == 1:
#             return [[i.to_value()] for i in lists[0]]
#         l = []
#         for other in self._recursive_list(lists[1:]):
#             for i in lists[0]:
#                 l.append((i.to_value(), *other))
#         return l

def find_figures(rgb: np.ndarray, params: FiguresSearchParams) -> FiguresSearchResult:
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    figures = []
    for range in params.color_ranges:
        color_min = range.min_color.to_tuple()
        color_max = range.max_color.to_tuple()
        bin = cv2.inRange(hsv, color_min, color_max)
        contours, _ = cv2.findContours(bin, params.contours_mode, params.contours_find_method)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > params.min_contour_area:
                fig = Figure(color_range=range, cnt=cnt, cnt_area=area, color_name=range.name)
                if not fig.define_center(cnt): continue # if no center found. HOW? - yes!
                fig.define_shape(cnt)
                figures.append(fig)
    return FiguresSearchResult(figures)

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
    def __enter__(self):
        return self.get_frame_func(self)
    def show(self, frame):
        self.show_frame_func(frame)
    def stab_by_arrow(self, arrow_color_range: ColorRange, time_to_stab: float = 5, accuracy: float = 5, show_video: bool = True, draw_arrows: bool = True, depth_to_keep: float = 2) -> float:
        timer = t.time()
        while True:
            # frame = auv.get_image_bottom()
            frame = self.get_frame_func(self)
            # print(frame.shape)
            draw = frame.copy()
            f = find_figures(frame, FiguresSearchParams([arrow_color_range])).figures
            triangle_p = f.where(Figure.color_name == 'orange', Figure.shape == ShapeType.TRIANGLE).max(Figure.cnt_area)
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
                        # keep(auv.get_yaw() - f_triangle.arrow_angle, 3, time)
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
    def move_to_color(self, color_range: ColorRange, delay_time: int = 0, depth_to_keep: float = 3, yaw_to_keep: float = ..., forward_speed: float = 50, show_frame: bool = True):
        timer = t.time()
        if yaw_to_keep is ...:
            yaw_to_keep = self.get_yaw()
        while True:
            frame = self.get_frame_func(self)
            figures = find_figures(frame, FiguresSearchParams([color_range])).figures
            pointer = figures.where(Figure.color_name == 'orange', Figure.shape == ShapeType.TRIANGLE).max(Figure.cnt_area)
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

