import cv2
from picamera2 import Picamera2
import time
import numpy as np


picam2 = Picamera2()
dispW=1280
dispH=720
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)

hueLow = 94
hueHigh = 96

satLow = 100
satHigh = 255

valLow= 100
valHigh= 255

lowerBound = np.array([hueLow, satLow, valLow])
upperBound = np.array([hueHigh, satHigh, valHigh])


while True:
    tStart=time.time()
    frame= picam2.capture_array()
    frameHSV = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
    
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)
    myMaskSmall=cv2.resize(myMask, (int(dispW/2), int(dispH/2)))
    
    objectOfInterest=cv2.bitwise_and(frame, frame,mask=myMask)
    objectOfInterestSmall = cv2.resize(objectOfInterest, (int(dispW/2), int(dispH/2)))

    print(frameHSV[int(dispH/2),int(dispW/2)])
    cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)
    cv2.imshow("Camera", frame)
    cv2.imshow("My Mask", myMaskSmall)
    cv2.imshow("OOI", objectOfInterestSmall)
    if cv2.waitKey(1)==ord('q'):
        break
    tEnd=time.time()
    loopTime=tEnd-tStart
    #low pass filter (trust the previous fps value 90% / current one 10%)
    #filtering high frequency changes
    fps=.9*fps + .1*(1/loopTime)  
cv2.destroyAllWindows()