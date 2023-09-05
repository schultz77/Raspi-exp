import cv2
import time
from Frame_Capture_thrd_stop import piVideoStream
import mediapipe as mp

mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraws = mp.solutions.drawing_utils

dispW = int(1280 * 0.5)
dispH = int(720 * 0.5)
frameRate = 30
fps = 0
textPos = (30, 60)
font = cv2.FONT_HERSHEY_SIMPLEX
textHeight = 1
textWeight = 3
textColor = (0, 0, 180)

# instantiating the class objects
myCap = piVideoStream(resolution=(dispW, dispH), framerate=frameRate)
# starting the thread for capturing frames
myCap.start()

while True:
    img = myCap.getFrame()
    tStart = time.time()

    if len(img):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(imgRGB)

        if results.multi_hand_landmarks: # are there any hands detected
            for handLms in results.multi_hand_landmarks: # loop through hands
                for myID, lm in enumerate(handLms.landmark):  # get ids and landmarks
                    h, w, c = img.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    if myID == 4:
                        cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

                mpDraws.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

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
