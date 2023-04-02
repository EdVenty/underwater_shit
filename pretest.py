# -*- coding: cpreprocessor -*-

import enum


# *place operators here*

def operator(name): 
    def __wrapper1__(func):
        _operators[name] = func
        def __wrapper2__(*args, **kwds):
            return func(*args, **kwds)
        return __wrapper2__
    return __wrapper1__

def run_operator(a, b, oper):
    return _operators[oper](a, b)

class Compare(enum.Enum):
    LT = 0
    EQ = 1
    GT = 2

@operator("<?>")
def operator_comparison(a, b):
    if a < b:
        return Compare.LT
    elif a > b:
        return Compare.GT
    return Compare.EQ

@operator("&&")
def operator_and(a, b):
    return bool(a) and bool(b)

@operator("??")
def operator_elseNone(a, b):
    if a is None:
        return b
    return a

def main():
    "amogus" = 5