from enum import Enum
from typing import List, Tuple, Type


class Infix:
    def __init__(self, function, attr: str = None, compfunc=None) -> None:
        self.__function = function
        self.__attr = attr
        self.__compfunc = compfunc

    def __ror__(self, other):
        if self.__compfunc is not None:
            if self.__attr is not None:
                return Infix(lambda x, self=self, other=other: self.__function(self.__compfunc(getattr(other, self.__attr)), self.__compfunc(getattr(x, self.__attr))))
            else:
                return Infix(lambda x, self=self, other=other: self.__function(self.__compfunc(other), self.__compfunc(x)))
        else:
            if self.__attr is not None:
                return Infix(lambda x, self=self, other=other: self.__function(getattr(other, self.__attr), getattr(x, self.__attr)))
            else:
                return Infix(lambda x, self=self, other=other: self.__function(other, x))

    def __or__(self, other):
        if self.__compfunc is not None:
            if self.__attr is not None:
                return self.__function(self.__compfunc(getattr(other, self.__attr)))
            else:
                return self.__function(self.__compfunc(other))
        else:
            if self.__attr is not None:
                return self.__function(getattr(other, self.__attr))
            else:
                return self.__function(other)

    def __rlshift__(self, other):
        return self.__ror__(other)

    def __rshift__(self, other):
        return self.__or__(other)
    # def __call__(self, value1, value2):
    #     if self.__attr is not None:
    #         return self.__function(getattr(value1, self.__attr), getattr(value2, self.__attr))
    #     else:
    #         return self.__function(value1, value2)

    def __getattribute__(self, name: str):
        if name in ["_Infix__function", "_Infix__attr", "_Infix__compfunc"]:
            return super().__getattribute__(name)
        return Infix(self.__function, name, self.__compfunc)

    def __getitem__(self, compfunc):
        return Infix(self.__function, self.__attr, compfunc)


class Then:
    def __init__(self, function):
        self.__function = function

    def __rlshift__(self, other):
        return Then(lambda x, self=self, other=other: self.__function(other, x))

    def __rshift__(self, other):
        return self.__function(other)


class Compare(Enum):
    EQ = 0
    GT = 1
    LT = 2


class Default:
    def __matmul__(self, result) -> List:
        return [result]


def switch(value, *args):
    default = None
    for values in args:
        if len(values) == 1:
            default = values[0]
        else:
            if type(values[0]) == Default:
                default = values[1]
            elif value == values[0]:
                return values[1]
    return default


class Value:
    def __init__(self, value) -> None:
        self.value = value

    def __matmul__(self, result) -> Tuple:
        return (self.value, result)


class Person:
    def __init__(self, age, name) -> None:
        self.age = age
        self.name = name


class Sequence:
    def __init__(self, *functions) -> None:
        self.__functions = functions

    def __call__(self, *args):
        # return Sequence(*functions)
        return self.exec(*args)

    def exec(self, *args):
        for func in self.__functions:
            if type(args) not in [list, tuple]:
                args = func(args)
            else:
                args = func(*args)
        return args


class PatternMatch:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f"<PatternMatch {self.x}:{self.y}>"


class WrapArg:
    def __init__(self, name) -> None:
        self.name = name

    def __or__(self, other):
        # print(self.name, other.name)
        return PatternMatch(self, other)

    def __str__(self) -> str:
        return f"<WrapArg name: {self.name}>"


class ID:
    def __getattribute__(self, name: str):
        return WrapArg(name)


class WrapFunc:
    pass


class Do:
    def __init__(self, x=None) -> None:
        self.x = x

    def __rlshift__(self, other):
        # print(other)
        return Do(other)

    def __rshift__(self, other):
        # print(other, self.x)
        self.x == other
        return [other, self.x]


class Match:
    def __init__(self, x, values) -> None:
        self.x = x
        self.values = values

    def __str__(self) -> str:
        return f"<Match x: {self.x} values: {self.values}"


def match(x, *values):
    return Match(x, values)


class Function:
    def __init__(self, args=None, function=None) -> None:
        if type(args) not in [tuple, list]:
            self.args = [args]
        else:
            self.args = args
        self.function = function

    def __matmul__(self, args):
        # print(args)
        self.args = args
        return self

    def __call__(self, *args):
        # print(self.function)
        # print(arg.s)
        arg = args[0]
        if type(self.function) == Match:
            for [pattern], wrap in self.function.values:
                # print(pattern, wrap)
                if len(arg) >= 2 and type(pattern) == PatternMatch:
                    if type(wrap) == WrapArg:
                        return arg[0] if wrap.name == pattern.x.name else arg[1:]
                    else:
                        return wrap
                elif len(arg) == 1 and type(pattern) == WrapArg:
                    if type(wrap) == WrapArg:
                        return arg[0]
                    else:
                        return wrap
        else:
            return self.function

    def __eq__(self, o: object) -> bool:
        # print(self.function)
        self.function = o


_ = ID()
do = Do()
sequence = Sequence
default = Default()
divide = Infix(lambda x, y: x/y)
compare = Infix(lambda x, y: Compare.LT if x <
                y else Compare.GT if x > y else Compare.EQ)
space = Infix(lambda x, y: str(x) + ' ' + str(y))
then = Then(lambda x, y: (x, y))

# print("Длина имени первого человека" |space|
#     switch(Person(13, 'dвв') |compare [sequence(len, lambda x: 100 - x)] . name| Person(12, 'ddвв'),
#         Value(Compare.GT) @ "больше",
#         Value(Compare.LT) @ "меньше",
#         Value(Compare.EQ) @ "равена"
#     )
# )
# x, xs, foldr = PatternMatch(), PatternMatch(), Function()
# foldr = Function()
# foldr @ (_.lst, _.accum, _.func) = match(_.lst
#     [_.x] <<then>> _.func(_.accum, _.x),
#     [_.x|_.xs] <<then>> _.func(_.xs, foldr(_.xs, _.accum, _.func))
# )
myLen = Function()
myLen @ _.lst << do >> match(_.lst,
                             [_.x] << then >> "равна одному",
                             [_.x | _.xs] << then >> "более одного"
                             )
print("Длина списка" | space | myLen([1]))
