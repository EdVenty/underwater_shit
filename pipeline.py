import abc
from collections import defaultdict
from dataclasses import dataclass, field
import re
import sys
from typing import Any, Callable, Dict, List
from inspect import stack
import inspect

@dataclass
class PipelineChainElement:
    plc: 'PipelineChain'
    function: Callable
    args: List = field(default_factory=list)
    kwargs: Dict = field(default_factory=dict)

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        self.args = args
        self.kwargs = kwds
        self.plc._PipelineChain__pces.append(self)
        return self.plc

    def run(self):
        return self.function(self.plc._PipelineChain__pipe, *self.args, **self.kwargs)

class PipelineChainRuner:
    def __init__(self, pc, loop: bool = False) -> None:
        self._pc = pc
        self._loop = loop

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        return self._pc._run(self._loop)

class PipelineChain:
    def __init__(self, pipeline: 'Pipeline', pces: List[PipelineChainElement] = []) -> None:
        self.__pipe = pipeline
        self.__pces = pces

    def _run(self, loop: bool = False):
        result = None
        while not result:
            for p in self.__pces:
                result = p.run()
        return result

    def __getattribute__(self, __name: str) -> Any:
        if __name == "run":
            return PipelineChainRuner(self)
        elif __name == "loop":
            return PipelineChainRuner(self, loop=True)
        if not any(map(__name.startswith, ['_PipelineFunctionWrapper_', '_PipelineChain', '__', '_run'])):
            return PipelineChainElement(self, getattr(self.__pipe, __name)._func)
        return super().__getattribute__(__name)

    def __str__(self) -> str:
        return f"<PipelineChain pipe={self.__pipe}, pces={self.__pces}"

class PipelineFunctionWrapper:
    def __init__(self, obj, func) -> None:
        self._obj = obj
        self._func = func

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        pc = PipelineChain(self._obj)
        pc._PipelineChain__pces = [PipelineChainElement(pc, self._func, args, kwds)]
        return pc

class Pipeline:
    def __init__(self) -> None:
        self._chain = []
        
    def __new__(cls, clsname, bases, attrs):
        obj = type(clsname, (bases), attrs)
        for key in attrs:
            if callable(attrs[key]):
                setattr(obj, key, PipelineFunctionWrapper(obj, attrs[key]))
        return obj

class Static:
    def __init__(self, value: Any) -> None:
        self._ = value

    def get(self):
        return self._value

    def set(self, value):
        self._value = value

def get_calling_function(fr):
    """finds the calling function in many decent cases."""
    # fr = sys._getframe(1)   # inspect.stack()[1][0]
    co = fr.f_code
    for get in (
        lambda:fr.f_globals[co.co_name],
        lambda:getattr(fr.f_locals['self'], co.co_name),
        lambda:getattr(fr.f_locals['cls'], co.co_name),
        lambda:fr.f_back.f_locals[co.co_name], # nested
        lambda:fr.f_back.f_locals['func'],  # decorators
        lambda:fr.f_back.f_locals['meth'],
        lambda:fr.f_back.f_locals['f'],
        ):
        try:
            func = get()
        except (KeyError, AttributeError):
            pass
        else:
            if type(func) == PipelineFunctionWrapper:
                return func._func
    raise AttributeError("func not found")

def static(value) -> Static:
    callerframerecord = inspect.stack()[1]    # 0 represents this line
                                            # 1 represents line at caller
    frame = callerframerecord[0]
    co_str = inspect.getframeinfo(frame)[3][0].strip()
    var_name = re.search(r'^(.+) = static.*$', co_str).group(1)

    func = get_calling_function(frame)

    if not hasattr(func, f"_static_{var_name}"):
        setattr(func, f"_static_{var_name}", Static(value))

    return getattr(func, f"_static_{var_name}")

# class StaticVariable:
#     def __init__(self) -> None:
#         pass

#     def 

class Robot(metaclass=Pipeline):
    def keep_depth(self, depth: float):
        print("keep depth", depth)
    def keep_yaw(self, yaw: float):
        print("keep yaw", yaw)
    def define_arrow(self):
        counter = static(1)
        if counter._ >= 5:
            print("end")
            return True
        else:
            print(counter._)
            counter._ += 1

print(Robot().keep_depth(1).keep_yaw(0).define_arrow().run())