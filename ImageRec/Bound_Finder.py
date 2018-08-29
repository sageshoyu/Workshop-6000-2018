import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('frame')

cv2.createTrackbar('H_lower', 'frame', 0, 180, nothing)
cv2.createTrackbar('S_lower', 'frame', 0, 255, nothing)
cv2.createTrackbar('V_lower', 'frame', 0, 255, nothing)

cv2.createTrackbar('H_upper', 'frame', 0, 180, nothing)
cv2.createTrackbar('S_upper', 'frame', 0, 255, nothing)
cv2.createTrackbar('V_upper', 'frame', 0, 255, nothing)

while (1):
    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hl = cv2.getTrackbarPos('H_lower', 'frame')
    sl = cv2.getTrackbarPos('S_lower', 'frame')
    vl = cv2.getTrackbarPos('V_lower', 'frame')

    hu = cv2.getTrackbarPos('H_upper', 'frame')
    su = cv2.getTrackbarPos('S_upper', 'frame')
    vu = cv2.getTrackbarPos('V_upper', 'frame')

    lower_bound = np.array([hl, sl, vl])
    upper_bound = np.array([hu, su, vu])

    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    res = cv2.bitwise_and(frame, frame, mask = mask)

    cv2.imshow('in', frame)
    cv2.imshow('out', res)
    cv2.imshow('mask', mask)

    k = cv2.waitKey(5) & 0xFF
    if k == ord('q'):
        cv2.destroyAllWindows()
        break