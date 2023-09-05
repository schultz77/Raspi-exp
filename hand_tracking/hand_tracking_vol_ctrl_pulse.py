import cv2
import time
from Frame_Capture_thrd_stop import piVideoStream
import math

import hand_tracking_module as htm
import numpy as np
import pulsectl


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


minAngle = 0
maxAngle = 180
angle = 0
angleBar = 400
angleDeg = 0
minHand = 20  # 50
maxHand = 240  # 300
vol = None


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

            # print('len: ', length)
            # Hand range 30 - 350

            angle = np.interp(length, [minHand, maxHand], [minAngle, maxAngle])
            angleBar = np.interp(length, [minHand, maxHand], [400, 150])
            angleDeg = np.interp(length, [minHand, maxHand], [0, 180])  # degree angle 0 - 180

            vol = np.interp(length, [minHand, maxHand], [0.0, 1.0])
            print('vol: ', vol)

            with pulsectl.Pulse('volume') as pulse:
                for sink in pulse.sink_list():
                    # pulse.volume_change_all_chans(sink, vol)
                    pulse.volume_set_all_chans(sink, vol)

            if length < 50:
                cv2.circle(img, (cx, cy), 15, (0, 255, 0), cv2.FILLED)

        cv2.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
        cv2.rectangle(img, (50, int(angleBar)), (85, 400), (255, 0, 0), cv2.FILLED)
        cv2.putText(img, f'{int(angleDeg)} deg', (40, 450), cv2.FONT_HERSHEY_COMPLEX,
                    1, (0, 9, 255), 1)

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
