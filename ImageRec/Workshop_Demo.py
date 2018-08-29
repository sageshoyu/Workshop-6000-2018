import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def getAspectRatio(cont):
    _, _, w, h = cv2.boundingRect(cont)
    return float(w)/h


while 1:

    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_bound = np.array([142, 162, 128])
    upper_bound = np.array([180, 255, 255])

    thresh = cv2.inRange(hsv, lower_bound, upper_bound)

    kernel = np.ones((19, 19), np.uint8)

    denoised = cv2.dilate(thresh, kernel, iterations=1)
    denoised = cv2.erode(denoised, kernel, iterations=1)

    denoised = cv2.dilate(denoised, kernel, iterations=1)
    denoised = cv2.erode(denoised, kernel, iterations=1)

    _, contours, hierarchy = cv2.findContours(denoised, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [cont for cont in contours if 10000 < cv2.contourArea(cont) < 60000]
    contours = [cont for cont in contours if 1.5 < getAspectRatio(cont) < 2.1]

    if len(contours) > 0:
        x, y, w, h = cv2.boundingRect(contours[0])
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print(getAspectRatio(contours[0]))

    cv2.imshow('frame', frame)
    cv2.imshow('thresh', thresh)
    cv2.imshow('denoised', denoised)


    k = cv2.waitKey(5) & 0xFF
    if k == ord('q'):
        cv2.destroyAllWindows()
        break
