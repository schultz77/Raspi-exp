import cv2
import time
from Frame_Capture_thrd_stop import piVideoStream
# import mediapipe as mp
import math

from pynput.keyboard import Key, Controller


import hand_tracking_module as htm
import numpy as np

keyboard = Controller()

dispW = 640
dispH = 480
frameRate = 30
fps = 0
textPos = (30, 60)
font = cv2.FONT_HERSHEY_SIMPLEX
textHeight = 1
textWeight = 3
textColor = (0, 0, 180)

# instantiating the class for capturing frames
myCap = piVideoStream(resolution=(dispW, dispH), framerate=frameRate)
# starting the thread for capturing frames
myCap.start()
# instantiating the class for detecting hands
myDetector = htm.handDetector(detectionConf=0.7)

pTime = 0
last_angle = None
last_length = None



minAngle = 0
maxAngle = 180
angle    = 0
angleBar = 400
angleDeg = 0
minHand  = 30 #50
maxHand  = 350 #300


while True:
    img = myCap.getFrame()
    tStart = time.time()

    if len(img):
        img = myDetector.findHands(img=img)
        lmList = myDetector.findPosition(img, draw=False)
        if len(lmList):
            x1, y1 = lmList[4][1], lmList[4][2]
            x2, y2 = lmList[8][1], lmList[8][2]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 15, (255, 0, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
            length = math.hypot(x2 - x1, y2 - y1)

            print(length)
            # Hand range 30 - 350

            angle = np.interp(length, [minHand, maxHand], [minAngle, maxAngle])
            angleBar = np.interp(length, [minHand, maxHand], [400, 150])
            angleDeg = np.interp(length, [minHand, maxHand], [0, 180])  # degree angle 0 - 180

            if last_length:
                if length > last_length:
                    keyboard.press(Key.media_volume_up)
                    keyboard.release(Key.media_volume_up)
                    print("VOL UP")
                elif length < last_length:
                    keyboard.press(Key.media_volume_down)
                    keyboard.release(Key.media_volume_down)
                    print("VOL DOWN")

            last_angle = angle
            last_length = length

            # print(int(length), angle)

            if length < 50:
                cv2.circle(img, (cx, cy), 15, (0, 255, 0), cv2.FILLED)

        cv2.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
        cv2.rectangle(img, (50, int(angleBar)), (85, 400), (255, 0, 0), cv2.FILLED)
        cv2.putText(img, f'{int(angleDeg)} deg', (40,450), cv2.FONT_HERSHEY_COMPLEX,
                    1, (0, 9, 255), 1)



            # x1, y1 = lmList[8][1], lmList[8][2]
            # print('pt8: ', x1, y1)
            # x2, y2 = lmList[0][1], lmList[0][2]
            # print('pt1: ', x2, y2)
            # length = math.hypot(x2-x1, y2-y1)
            # # length = y2 - y1
            # cv2.line(img, (x1, y1), (x2, y2), textColor, textWeight-1)
            # cx, cy = abs(x1-x2) // 2, abs(y1-y2) // 2
            # center_text = 'dist: ' + str(length)
            # cv2.putText(img, center_text, (x1+cx, y1+cy), font, textHeight, textColor, textWeight-1)

            # print(abs(lmList[8][1] - lmList[5][1]))

        cv2.putText(img, str(int(fps)) + ' FPS', textPos, font, textHeight, textColor, textWeight)
        cv2.imshow('Preview', img)  # Display the Video
        if cv2.waitKey(1) == ord('q'):
            myCap.stop()  # When everything done, release the capture
            myCap.join()
            break

        tEnd = time.time()
        loopTime = tEnd - tStart
        fps = 0.8 * fps + 0.2 * (1 / loopTime)

# When everything done, release the capture
cv2.destroyAllWindows()

