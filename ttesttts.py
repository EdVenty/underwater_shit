from abc import abstractmethod
from typing import Any, NewType, Tuple, Type, Union
import cv2
import numpy as np

class FrameNotCapturedException(Exception): ...

Vector3 = NewType("Vector3", Tuple[int, int, int])
VideoCapturePath = NewType("VideoCapture Path", Tuple[int, str])
VideoCaptureAPI = NewType("VideoCapture API Reference", int)

ColorCode = NewType("Color Code", int)

Image = NewType("Image", np.ndarray)
BGRImage = NewType("BGR Image", Image)
HSVImage = NewType("HSV Image", Image)
BinaryImage = NewType("Binary Image", Image)

class Module:
    @abstractmethod
    def __init__(self) -> None:
        """Base class for cv2 module.
        """
        pass
    @abstractmethod
    def forward(self, src: Image) -> Image:
        raise NotImplementedError("Module forward method not implemented.")
    @abstractmethod
    def __call__(self, src: Image, *args: Any, **kwds: Any) -> Image:
        """Call cv2 module.

        Args:
            src (Image): Input image.

        Returns:
            Image: Output image.
        """
        return self.forward(src, *args, **kwds)

class InRange(Module):
    def __init__(self, lowerb: Vector3, upperb: Vector3) -> None:
        self.lowerb = lowerb
        self.upperb = upperb
    def forward(self, src: Image) -> Image:
        return cv2.inRange(src, self.lowerb, self.upperb)

class CvtColor(Module):
    def __init__(self, code: ColorCode) -> None:
        """Convert Image color. See https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab

        Args:
            code (ColorCode): Cv2 Color Code. 
        """
        self.code = code
    def forward(self, src: Image) -> Image:
        return cv2.cvtColor(src, self.code)

class Sequence:
    def __init__(self, *modules: Type[Module]) -> None:
        self._modules = list(modules)

    def add_action(self, module: Type[Module]) -> None:
        self._modules.append(module)

    def forward(self, src: Image) -> Image:
        copy = src.copy()
        for action in self._modules:
            copy = action(copy)
        return copy

    def __call__(self, src: Image, *args: Any, **kwds: Any) -> Image:
        return self.forward(src, *args, *kwds)

class VideoCapture:
    def __init__(self, path: VideoCapturePath, api: VideoCaptureAPI = ...) -> None:
        """Cv2 VideoCapture. See https://docs.opencv.org/3.4/d8/dfe/classcv_1_1VideoCapture.html

        Args:
            path (VideoCapture Path): VideoCapture path (camera index or filename). 
                Index can be:
                    id of the video capturing device to open. To open default camera using default backend just pass 0. (to backward compatibility usage of camera_id + domain_offset (CAP_*) is valid when apiPreference is CAP_ANY)
                Filename can be:
                    1) name of video file (eg. video.avi)
                    2) image sequence (eg. img_%02d.jpg, which will read samples like img_00.jpg, img_01.jpg, img_02.jpg, ...)
                    3) URL of video stream (eg. protocol://host:port/script_name?script_params|auth)
                    4) GStreamer pipeline string in gst-launch tool format in case if GStreamer is used as backend Note that each video stream or IP camera feed has its own URL scheme. Please refer to the documentation of source stream to know the right URL.
            api (VideoCapture API Reference): preferred Capture API backends to use. Can be used to enforce a specific reader implementation if multiple are available: e.g. cv::CAP_DSHOW or cv::CAP_MSMF or cv::CAP_V4L2.
        """
        self.path = path
        self.api = api

        if api is not ...:
            self._cam = cv2.VideoCapture(self.path, self.api)
        else:
            self._cam = cv2.VideoCapture(self.path)
    def read(self) -> BGRImage:
        """Read frame from VideoCapture.

        Raises:
            FrameNotCapturedException: Frame not captured.

        Returns:
            BGR Image: cv2 BGR frame.
        """
        r, frame = self._cam.read()
        if not r:
            raise FrameNotCapturedException("Frame from videocapture({id}) not captured.".format(id=self.path))
        return frame
    def save_read(self) -> Tuple[bool, BGRImage]:
        """Read frame from VideoCapture without exception raising.

        Returns:
            (bool, BGR Image): Tuple of 1)If frame read; 2)cv2 BGR frame.
        """
        return self._cam.read()
    @property
    def isOpened(self) -> bool:
        """Returns true if video capturing has been initialized already.
        """
        return self._cam.isOpened()
    def release(self) -> None:
        """Closes video file or capturing device.
        """
        self._cam.release()

class MyModule(Module):
    def __init__(self) -> None:
        super().__init__()
        self.lowerb = Vector3((0, 0, 0))
        self.upperb = Vector3((255, 255, 20))
        self.seq = Sequence(
            CvtColor(cv2.COLOR_BGR2HSV),
            InRange(self.lowerb, self.upperb)
        )
    def forward(self, src: HSVImage) -> BinaryImage:
        return self.seq(src)
    

def main():
    # m = MyModule()
    # camera = VideoCapture(1)

    # while True:
    #     res = m(camera.read())
    #     cv2.imshow("frame", res)
    #     cv2.waitKey(1)

    # a = [1, 2, 3, 2, 0] 
    # b = [5, 1, 2, 7, 3, 2]
    # intersections = [i for i in a if i in b]
    # print(intersections)

    # a = "AAAABBBCCXYZDDDDEEEFFFAAAAAABBBBBBBBBBBBBBBBBBBBBBBBBBBB"
    # out = ''
    # lastL = ''
    # counterL = 0
    # for l in a:
    #     if l != lastL:
    #         out += lastL + str(counterL)
    #         counterL = 0
    #         lastL = l
    #     counterL += 1
    # out += lastL + str(counterL)
    # print(out)

    a = [1,4,5,2,3,9,8,11,0]
    out = ''
    last = min(a)
    last_min = last
    for i in range(last, max(a) + 1):
        if i in a:
            last = i
            if last_min is None:
                last_min = last
        elif last is not None:
            if last == last_min:
                out += str(last_min) + ','
            else:
                out += str(last_min) + '-' + str(last) + ','
            last_min = None
            last = None
    if last == last_min:
        out += str(last_min)
    else:
        out += str(last_min) + '-' + str(last)
    print(out)




if __name__ == "__main__":
    main()