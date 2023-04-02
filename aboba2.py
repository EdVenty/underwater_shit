from textwrap import indent
import time
from typing import Any, List, Set, Tuple, Union

# from numpy.core.fromnumeric import shape
# import pymurapi as mur
# import aboba
from enum import Enum
import cv2
import numpy as np
import math
import copy

# auv = mur.mur_init()

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


        

# CAMERA_WIDTH = 320
# CAMERA_HEIGHT = 240

colors = [
    # ColorRange(Color("white", 0, 0, 191.25), Color("white", 180, 25.5, 255)),
    # ColorRange(Color("lightgrey", 0, 0, 127.5), Color("lightgrey", 255, 255, 191.25)),
    # ColorRange(Color("darkgrey", 0, 0, 63.75), Color("darkgrey", 255, 255, 127.5)),
    # ColorRange(Color("black", 0, 0, 0), Color("black", 255, 255, 63.75)),
    ColorRange(Color(170, 20, 20), Color(15, 255, 255), "red"),
    ColorRange(Color(135, 20, 20), Color(170, 255, 255), "pink"),
    ColorRange(Color(115, 20, 20), Color(135, 255, 255), "purple"),
    ColorRange(Color(100, 20, 20), Color(115, 255, 255), "blue"),
    ColorRange(Color(92.5, 20, 20), Color(100, 255, 255), "lblue"),
    ColorRange(Color(60, 20, 20), Color(92.5, 255, 255), "green"),
    ColorRange(Color(25, 20, 20), Color(60, 255, 255), "yellow"),
    ColorRange(Color(10, 20, 20), Color(25, 255, 255), "orange")
]

# if __name__ == "__main__":
#     aboba.yaw_pd = aboba.PD(0.1, 0.001)
#     aboba.depth_pd = aboba.PD(0.1, 0.005)
#     while True:
#         frame = aboba.auv.get_image_bottom()
#         draw = frame.copy()
#         result = find_figures(frame, FiguresSearchParams(colors, min_contour_area=100))
#         for f in unpoint(result.figures):
#             # cv2.circle(draw, f.center, 5, (255, 0, 0))
#             # f.draw_figure(draw, thickness=3)
#             if f.center is not None:
#                 center = f.center
#                 cv2.putText(draw, str(f.shape).replace("ShapeType.", ''), (center[0], center[1] + 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255))
#                 cv2.putText(draw, f.color_range.min_color.name, f.center, cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0))
#         # orange_top = result.figures.where(Figure.color_name == 'orange').min(Figure.center_y).to_value_or_default()
#         # if orange_top is not None:
#         #     orange_top.draw_contour(draw, thickness=3)
#         # max_orange = result.figures.where(Figure.color_name == 'orange').min(Figure.center_y).to_value_or_default()
#         # vectors_and_distance = []
#         # for i in unpoint(result.figures.where(Figure.color_name == 'orange')):
#         #     for j in unpoint(result.figures.where(Figure.color_name == 'green')):
#         #         dist = abs(math.sqrt((j.center_x - i.center_x) ** 2 + (j.center_y - i.center_y) ** 2))
#         #         vectors_and_distance.append((i, j, dist))
#         # a = result.figures.copy()
#         # b = result.figures.copy().with_other(a)
#         # c = result.figures.copy().with_other(b)
#         # ...
#         # a = result.figures.copy().where(Figure.color_name == 'green')
#         # vectors_and_distance = result.figures.copy().where(Figure.color_name == 'orange').with_other(a).zip().exec_func(lambda i, j: abs(math.sqrt((j.center_x - i.center_x) ** 2 + (j.center_y - i.center_y) ** 2))).calc_results()
#         # if len(vectors_and_distance) > 0:
#         #     a, b, dist = min(vectors_and_distance, key=lambda x: x[2])
#         #     a.draw_figure(draw, thickness=3)
#         #     b.draw_figure(draw, thickness=3)
#         # a = result.figures.where(Figure.color_name == 'lblue')
#         # b = result.figures.where(Figure.color_name == 'pink')
#         # c = result.figures.where(Figure.color_name == 'yellow')
#         # comp = FiguresListComposition([a, b, c])
#         calculated = result.figures.where(Figure.color_name == 'lblue').compose([
#                 result.figures.where(Figure.color_name == 'pink'), 
#                 result.figures.where(Figure.color_name == 'yellow')
#                 ]).exec_func(lambda i, j, a:
#             Maath.vectors2_distance(i.center, j.center) + 
#             Maath.vectors2_distance(i.center, a.center) + 
#             Maath.vectors2_distance(a.center, j.center)
#         )
#         if len(calculated) > 0:
#             a, b, c = min(calculated, key=lambda x: x.result).get_args()
#             a.draw_figure(draw, thickness=3)
#             b.draw_figure(draw, thickness=3)
#             c.draw_figure(draw, thickness=3)
#             cv2.circle(draw, tuple(Convert.list_elements_to(Maath.vectors2_triangle_center(a.center, b.center, c.center), int)), 3, (255, 0, 0), 3)
#         # if max_orange is not None:
#         #     cv2.circle(draw, max_orange.center, 5, (255, 255, 255))
#         #     error = CAMERA_WIDTH / 2 - max_orange.center[0]
#         #     error_y = CAMERA_HEIGHT / 2 - max_orange.center[1]
#         #     aboba.keep(aboba.clamp(aboba.auv.get_yaw() - error, -180, 180), aboba.auv.get_depth() - error_y, 20, time=0.01)
#         #     if f.cnt_area > 40000:
#         #         aboba.auv.shoot()
            
#         cv2.imshow("draw", draw)
#         cv2.waitKey(1)