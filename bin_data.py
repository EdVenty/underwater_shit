import cv2
import numpy as np
import pymurapi as mur

if __name__ == '__main__':
    auv = mur.mur_init()
    def nothing(*arg):
        pass

cv2.namedWindow("result")  # создаем главное окно
cv2.namedWindow("settings")  # создаем окно настроек
#  (0, 63, 0) (255, 255, 99) чёрный
# (0, 144, 102) (70, 204, 255) жёлтый
# (121, 153, 67) (239, 255, 255) красный
# (33, 153, 67) (158, 255, 255) синий
# (67, 181, 102) (77, 255, 255) зелёный


# def constrain(value: float, min: float, max: float) -> float:
#     return min if value < min else max if value > max else value

constrain = lambda value, min, max: min if value < min else max if value > max else value

# создаем 6 бегунков для настройки начального и конечного цвета фильтра
cv2.createTrackbar('h', 'settings', 0, 360, nothing)
cv2.createTrackbar('s', 'settings', 0, 255, nothing)
cv2.createTrackbar('v', 'settings', 0, 255, nothing)
cv2.createTrackbar('hdev', 'settings', 255, 255, nothing)
cv2.createTrackbar('sdev', 'settings', 255, 255, nothing)
cv2.createTrackbar('vdev', 'settings', 255, 255, nothing)
crange = [0, 0, 0, 0, 0, 0]
# img = cv2.imread(r"scenes/20.png")
while True:
    img = auv.get_image_front()
    img = cv2.resize(img, (640, 480))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # считываем значения бегунков
    h = cv2.getTrackbarPos('h', 'settings')
    s = cv2.getTrackbarPos('s', 'settings')
    v = cv2.getTrackbarPos('v', 'settings')
    hdev = cv2.getTrackbarPos('hdev', 'settings')
    sdev = cv2.getTrackbarPos('sdev', 'settings')
    vdev = cv2.getTrackbarPos('vdev', 'settings')

    # формируем начальный и конечный цвет фильтра
    h_min = np.array((constrain(h - hdev, 0, 360), constrain(s - sdev, 0, 255), constrain(v - vdev, 0, 255)), np.uint8)
    h_max = np.array((constrain(h + hdev, 0, 360), constrain(s + sdev, 0, 255), constrain(v + vdev, 0, 255)), np.uint8)

    # накладываем фильтр на кадр в модели HSV
    thresh = cv2.inRange(hsv, h_min, h_max)

    cv2.imshow('result', thresh)
    cv2.imshow('w', img)
    print(h_min, ',', h_max)
    ch = cv2.waitKey(100)
    if ch == 0:
        break
