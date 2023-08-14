import cv2
from picamera2 import Picamera2
import time
import numpy as np

picam2 = Picamera2()

def TrackX(val):
    global xPos
    xPos=val
    print('x Position', xPos)

def TrackY(val):
    global yPos
    yPos=val
    print('y Position', yPos)

def TrackW(val):
    global boxW
    boxW=val
    print('Box Width', boxW)

def TrackH(val):
    global boxH
    boxH=val
    print('Box Height', boxH)


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

boxColor=(255,0,0)

cv2.namedWindow('My Trackbars')
cv2.createTrackbar('X Pos', 'My Trackbars', 10,dispW-1,TrackX)
cv2.createTrackbar('Y Pos', 'My Trackbars', 10,dispH-1,TrackY)
cv2.createTrackbar('Box Width', 'My Trackbars', 10,dispW-1,TrackW)
cv2.createTrackbar('Box Height', 'My Trackbars', 10,dispH-1,TrackH)



while True:
    tStart=time.time()
    frame= picam2.capture_array()
    ROI = frame[yPos:yPos+boxH,xPos:xPos+boxW]
    
    cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)
    cv2.rectangle(frame, (xPos,yPos), (xPos+boxW,yPos+boxH),myColor,3)

    cv2.imshow("Camera", frame)
    cv2.imshow("ROI", ROI)
    
    if cv2.waitKey(1)==ord('q'):
        break
    tEnd=time.time()
    loopTime=tEnd-tStart
    #low pass filter (trust the previous fps value 90% / current one 10%)
    #filtering high frequency changes
    fps=.9*fps + .1*(1/loopTime)  
cv2.destroyAllWindows()