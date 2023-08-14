import cv2
from picamera2 import Picamera2
import time
import numpy as np
from buildhat import Motor

x_motor = Motor('A')
y_motor = Motor('B')

panAngle =0
tiltAngle=0
x_motor.run_to_position(panAngle)
y_motor.run_to_position(tiltAngle)
Track = 0

picam2 = Picamera2()

def TrackBars(val):
    global hueLow, hueHigh, satLow, satHigh, valLow, valHigh, Track

    hueLow=cv2.getTrackbarPos('Hue Low', 'My Trackbars')
    hueHigh= cv2.getTrackbarPos('Hue High', 'My Trackbars')
    satLow= cv2.getTrackbarPos('Sat Low', 'My Trackbars')
    satHigh= cv2.getTrackbarPos('Sat High', 'My Trackbars')
    valLow= cv2.getTrackbarPos('Value Low', 'My Trackbars')
    valHigh= cv2.getTrackbarPos('Value High', 'My Trackbars')
    Track= cv2.getTrackbarPos('Train-Track', 'My Trackbars')
    
    

dispW=int(1280*0.8)
dispH=int(720*0.8)
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

cv2.createTrackbar('Hue Low', 'My Trackbars',0,360,TrackBars)
cv2.createTrackbar('Hue High', 'My Trackbars', 360,360,TrackBars)
cv2.createTrackbar('Sat Low', 'My Trackbars', 127,255,TrackBars)
cv2.createTrackbar('Sat High', 'My Trackbars', 255,255,TrackBars)
cv2.createTrackbar('Value Low', 'My Trackbars', 248,255,TrackBars)
cv2.createTrackbar('Value High', 'My Trackbars', 255,255,TrackBars)
cv2.createTrackbar('Train-Track', 'My Trackbars', 0, 1,TrackBars)


while True:
    tStart=time.time()
    frame= picam2.capture_array()
    # ROI = frame[yPos:yPos+boxH,xPos:xPos+boxW]

    lowerBound = np.array([hueLow, satLow, valLow])
    upperBound = np.array([hueHigh, satHigh, valHigh])

    frameHSV = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
    
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)
    myMaskSmall=cv2.resize(myMask, (int(dispW/2), int(dispH/2)))

    objectOfInterest=cv2.bitwise_and(frame, frame,mask=myMask)
    objectOfInterestSmall = cv2.resize(objectOfInterest, (int(dispW/2), int(dispH/2)))
    
    contours, junk = cv2.findContours(myMask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours):
        contours=sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
        # cv2.drawContours(frame,contours, 0, (0,255,0),3) #-1 for all contours
        contour=contours[0]
        x,y,w,h=cv2.boundingRect(contour)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),3)
        
        
        if Track == 1:
            # calculating the center of gravity of contour
            cog_contour_x = x+w/2
            cog_contour_y = y+h/2

            # print('contour_x: ', cog_contour_x)

            # calculating distance between cog_contour and the center of the window
            error_x = cog_contour_x - dispW/2
            panAngle += error_x/100

            if panAngle > 35:
                panAngle = 35
            if panAngle < -35:
                panAngle = -35

            if abs(error_x) > 35:
                x_motor.run_to_position(panAngle) 

            error_y = cog_contour_y - dispH/2 
            tiltAngle -= error_y/100
            if tiltAngle > 35:
                tiltAngle = 35
            if tiltAngle < -35:
                tiltAngle=-35
            
            if abs(error_y) > 35:
                y_motor.run_to_position(tiltAngle) 
             
    # print('x_pos_motor: {} {}'.format(panAngle, x_motor.get_aposition()))    

    cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)
    # cv2.rectangle(frame, (xPos,yPos), (xPos+boxW,yPos+boxH),myColor,3)

    cv2.imshow("Camera", frame)
    # cv2.imshow("ROI", ROI)
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