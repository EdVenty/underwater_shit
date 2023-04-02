import cv2
# import imutils

# img = cv2.imread("21.png")




cv2.namedWindow("settings")
cv2.createTrackbar("block size", "settings", 1, 255, lambda x: None)
cv2.createTrackbar("C", "settings", 1, 255, lambda x: None)
cv2.createTrackbar("threshold1", "settings", 1, 255, lambda x: None)
cv2.createTrackbar("threshold2", "settings", 1, 255, lambda x: None)
def main(img):
    draw = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    b_s = cv2.getTrackbarPos("block size", "settings")
    C = cv2.getTrackbarPos("C", "settings")
    is_thresh_contours = False
    if not(b_s % 2 == 1 and b_s > 1): 
        # cv2.putText(draw, "Block size % 2 == 1 or == 1!", (10, 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0))
        pass
    else:
        is_thresh_contours = True
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        a_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, b_s, C)
        cv2.imshow("adaptive thresh", a_thresh)
        thresh = cv2.threshold(blurred, cv2.getTrackbarPos("threshold1", "settings"), 255, cv2.THRESH_BINARY)[1]
        cv2.imshow("thresh", thresh)
        cnts, _ = cv2.findContours(a_thresh.copy(), cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE)
        # cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        # cnts = imutils.grab_contours(cnts)
        cv2.drawContours(draw, cnts, -1, (0, 255, 0))
    cv2.bilateralFilter(gray, 11, 17, 17)
    canny = cv2.Canny(gray, cv2.getTrackbarPos("threshold1", "settings"), cv2.getTrackbarPos("threshold2", "settings"))
    canny_contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(draw, canny_contours, -1, (255, 0, 0))
    # if is_thresh_contours:
    #     _, thresh_contours = cv2.findContours(cv2.bitwise_not(thresh), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #     if thresh_contours is not None:
    #         for c in thresh_contours:
    #             try:
    #                 (x, y, w, h) = cv2.boundingRect(c)
    #                 cv2.rectangle(draw, (x, y), (x + w, y + h), (0, 255, 0))
    #             except: pass
        # if len(thresh_contours) > 0:
        #     cv2.drawContours(draw, thresh_contours, -1, (0, 255, 0))
    cv2.imshow("canny", canny)
    cv2.imshow("draw", draw)
        
if __name__ == "__main__":
    while True:
        cap = cv2.VideoCapture("video.mkv")
        while cv2.waitKey(1) == -1: 
            r, img = cap.read()
            if not r: break
            # assert r, "Video not captured"
            main(img)
            