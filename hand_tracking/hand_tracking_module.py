import cv2
import mediapipe as mp
import math


class handDetector():
    def __init__(self, mode=False, maxHands=2, model_complexity=1, detectionConf=0.5, trackConf=0.5):
        self.results = None
        self.mode = mode
        self.maxHands = maxHands
        self.model_complexity = model_complexity
        self.detectionConf = detectionConf
        self.trackConf = trackConf

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode,
                                        max_num_hands=self.maxHands,
                                        model_complexity=self.model_complexity,
                                        min_detection_confidence=self.detectionConf,
                                        min_tracking_confidence=self.trackConf)
        self.mpDraws = mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:  # are there any hands detected
            for handLms in self.results.multi_hand_landmarks:  # loop through hands
                if draw:
                    self.mpDraws.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):
        lmList = []
        if self.results.multi_hand_landmarks:  # are there any hands detected
            myHand = self.results.multi_hand_landmarks[handNo]
            for myID, lm in enumerate(myHand.landmark):  # get ids and landmarks
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)  # transforming hand coordinates to pixels
                lmList.append([myID, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 0), cv2.FILLED)

        return lmList

    @staticmethod
    def findLength(lmList, pt1, pt2):
        x1, y1 = lmList[pt1][1], lmList[pt1][2]  # x and y coordinates of landmarks
        x2, y2 = lmList[pt2][1], lmList[pt2][2]  # x and y coordinates of landmarks
        lmLen = math.hypot(x2 - x1, y2 - y1)
        lmCoord = [[x1, y1], [x2, y2]]

        return lmLen, lmCoord

    @staticmethod
    def showLen(img, lmCoord, center_text):

        cv2.line(img, (lmCoord[1][0], lmCoord[1][1]),
                 (lmCoord[0][0], lmCoord[0][1]),
                 (0, 0, 255),
                 2)
        cx, cy = ((lmCoord[1][0] + lmCoord[0][0]) // 2,
                  (lmCoord[1][1] + lmCoord[0][1]) // 2)

        cv2.putText(img,
                    center_text,
                    (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 0), 2)
