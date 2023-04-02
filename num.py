# import cv2
# import test

# HSV_LOW = (0, 0, 0)
# HSV_HIGH = (180, 60, 255)

# cap = cv2.VideoCapture("http://192.168.1.161:8080/video")

# if __name__ == "__main__":
#     while True:
#         # frame = cv2.imread("img.png")
#         r, frame = cap.read()
#         print(frame.shape)
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(frame, HSV_LOW, HSV_HIGH)
#         draw = mask.copy()
#         # cv2.imshow("mask", mask)

#         # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         # # cv2.drawContours(draw, contours, -1 ,(255, 0, 0))
#         # areas = []
#         # for (i, cnt) in enumerate(contours):
            
#         #     area = cv2.contourArea(cnt)
#         #     if area > 5000: 
#         #         #print(area)
#         #         (x, y, w, h) = cv2.boundingRect(cnt)
#         #         cv2.rectangle(draw, (x, y), (x + w, y + h), (255, 0, 0))
#         #         img = frame[y:y+h, x:x+h, :]
#         #         # img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
#         #         # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         #         # im_pil = Image.fromarray(img)
#         #         # imw, imh, _ = frame.shape
#         #         num = test.detect(frame)
#         #         print(num)
#         #         # areas.append(area)
#         #         # cv2.putText(draw, str(abs(int(num.data.max(1, keepdim=True)[1][0][0]))), (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0))
#         #         # cv2.imshow("a" + str(i) , img)
#         # assert len(areas) == 2, "Too many contours!"
#         # border = max(areas)
#         # num = min(areas)
#         # print(f"border: {border}, num: {num}")
#         # print(border / num)
#         # cv2.imshow("draw", draw)
#         num = test.detect(frame)
#         print(num)
#         cv2.imshow("fuck", frame)

#         while cv2.waitKey(1) != -1:
#             pass

def a(b, c): ...
def a(b): ...

a(1, 2)