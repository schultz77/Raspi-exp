from picamera2 import Picamera2
import time
import numpy as np
import cv2
from servo import Servo
picam2 = Picamera2()
 
pan=Servo(pin=13)
tilt=Servo(pin=12)
 
panAngle=0
tiltAngle=0
 
pan.set_angle(panAngle)
tilt.set_angle(tiltAngle)
 
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
track=0
 
def onTrack1(val):
    global hueLow
    hueLow=val
    print('Hue Low',hueLow)
def onTrack2(val):
    global hueHigh
    hueHigh=val
    print('Hue High',hueHigh)
def onTrack3(val):
    global satLow
    satLow=val
    print('Sat Low',satLow)
def onTrack4(val):
    global satHigh
    satHigh=val
    print('Sat High',satHigh)
def onTrack5(val):
    global valLow
    valLow=val
    print('Val Low',valLow)
def onTrack6(val):
    global valHigh
    valHigh=val
    print('Val High',valHigh)
def onTrack7(val):
    global track
    track=val
    print('Track Value',track)
 
cv2.namedWindow('myTracker')
 
cv2.createTrackbar('Hue Low','myTracker',10,179,onTrack1)
cv2.createTrackbar('Hue High','myTracker',20,179,onTrack2)
cv2.createTrackbar('Sat Low','myTracker',100,255,onTrack3)
cv2.createTrackbar('Sat High','myTracker',255,255,onTrack4)
cv2.createTrackbar('Val Low','myTracker',100,255,onTrack5)
cv2.createTrackbar('Val High','myTracker',255,255,onTrack6)
cv2.createTrackbar('Train-0 Track-1','myTracker',0,1,onTrack7)
 
 
while True:
    tStart=time.time()
    frame= picam2.capture_array()
    frame=cv2.flip(frame,-1)
    frameHSV=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)
    lowerBound=np.array([hueLow,satLow,valLow])
    upperBound=np.array([hueHigh,satHigh,valHigh])
    myMask=cv2.inRange(frameHSV,lowerBound,upperBound)
    myMaskSmall=cv2.resize(myMask,(int(dispW/2),int(dispH/2)))
    myObject=cv2.bitwise_and(frame,frame, mask=myMask)
    myObjectSmall=cv2.resize(myObject,(int(dispW/2),int(dispH/2)))
    
    contours,junk=cv2.findContours(myMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)>0:
        contours=sorted(contours,key=lambda x:cv2.contourArea(x),reverse=True)
        #cv2.drawContours(frame,contours,-1,(255,0,0),3)
        contour=contours[0]
        x,y,w,h=cv2.boundingRect(contour)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),3)
        if track==1:
            error=(x+w/2)-dispW/2
            if error>35:
                panAngle=panAngle-1
                if panAngle<-90:
                    panAngle=-90
                pan.set_angle(panAngle)
            if error<-35:
                panAngle=panAngle+1
                if panAngle>90:
                    panAngle=90
                pan.set_angle(panAngle)
            tiltError=(y+h/2)-dispH/2
            if tiltError>35:
                tiltAngle=tiltAngle+1
                if tiltAngle>40:
                    tiltAngle=40
                tilt.set_angle(tiltAngle)
            if tiltError<-35:
                tiltAngle=tiltAngle-1
                if tiltAngle<-90:
                    tiltAngle=-90
                tilt.set_angle(tiltAngle)
        
    cv2.imshow('Camera',frame)
    cv2.imshow('Mask',myMaskSmall)
    cv2.imshow('My Object',myObjectSmall)
    if cv2.waitKey(1)==ord('q'):
        break
    tEnd=time.time()
    loopTime=tEnd-tStart
    fps=.9*fps + .1*(1/loopTime)
cv2.destroyAllWindows()