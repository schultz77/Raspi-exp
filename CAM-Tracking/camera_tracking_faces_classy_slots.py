# import the necessary packages
from picamera2 import Picamera2
from threading import Thread
import cv2
import numpy as np
import time
from servo import Servo

# import pandas as pd
# import plotly.graph_objects as go
# from plotly.subplots import make_subplots
# from buildhat import Motor


class Tracking:

    __slots__ = "panAngle",'tiltAngle', 'motorPan','motorTilt','MAX_ANGLE_PAN','MAX_ANGLE_TILT','MIN_PIX_ERR','PIX2DEG_RATE','dispW','dispH','cog_contour_x', 'cog_contour_y','low_freq_factor', 'x_motor', 'y_motor', 'error_x', 'error_y'

   
    def __init__(self, panAngle=0, tiltAngle=0, motorPan=13,motorTilt=12,MAX_ANGLE_PAN=90,MAX_ANGLE_TILT=40,MIN_PIX_ERR=35, PIX2DEG_RATE=70,
                 dispW=1280,dispH=720,cog_contour_x=0,cog_contour_y=0,low_freq_factor=0.8):
        
        self.panAngle = panAngle
        self.tiltAngle = tiltAngle
        self.motorPan = motorPan
        self.motorTilt = motorTilt
        self.MAX_ANGLE_PAN = MAX_ANGLE_PAN
        self.MAX_ANGLE_TILT = MAX_ANGLE_TILT
        self.MIN_PIX_ERR = MIN_PIX_ERR
        self.PIX2DEG_RATE = PIX2DEG_RATE
        self.dispW = dispW
        self.dispH = dispH
        self.cog_contour_x =  cog_contour_x
        self.cog_contour_y = cog_contour_y
        self.low_freq_factor = low_freq_factor
        self.x_motor = Servo(motorPan)
        self.y_motor = Servo(motorTilt)
 
        self.x_motor.set_angle(self.panAngle) # - clockwise / + counterclockwise
        self.y_motor.set_angle(self.tiltAngle) # + values down / - values up

   

    def low_pass_filter(self, dim=(0, 0, 0, 0)): # dim={'x_box':0,'y_box':0,'width_box': 0, 'height_box': 0}:
                 
        if self.cog_contour_x:
           self.cog_contour_x = self.low_freq_factor * self.cog_contour_x + (1-self.low_freq_factor) * (dim[0]+dim[2]/2)
        else:
           self.cog_contour_x = dim[0]+dim[2]/2
        
        if self.cog_contour_y:
            self.cog_contour_y = self.low_freq_factor * self.cog_contour_y + (1-self.low_freq_factor) * (dim[1]+dim[3]/2)
        else:
            self.cog_contour_y = dim[1]+dim[3]/2
        

         
    
    def movement(self, dim=(0, 0, 0, 0)): # dim={'x_box':0,'y_box':0,'width_box': 0, 'height_box': 0}
        
        self.low_pass_filter(dim)

        # calculating distance between cog_contour and the center of the window
        self.error_x = self.cog_contour_x - self.dispW/2
        self.panAngle -= self.error_x/self.PIX2DEG_RATE
        
        if self.panAngle < -self.MAX_ANGLE_PAN:
            self.panAngle = -self.MAX_ANGLE_PAN
        if self.panAngle > self.MAX_ANGLE_PAN:
            self.panAngle = self.MAX_ANGLE_PAN
        
        if abs(self.error_x) > self.MIN_PIX_ERR:
            self.x_motor.set_angle(self.panAngle)
        

        self.error_y = self.cog_contour_y - self.dispH/2
        self.tiltAngle -= self.error_y/self.PIX2DEG_RATE
                                        
        if self.tiltAngle > self.MAX_ANGLE_TILT:
            self.tiltAngle = self.MAX_ANGLE_TILT
        if self.tiltAngle < -(self.MAX_ANGLE_TILT+50):
            self.tiltAngle=-(self.MAX_ANGLE_TILT+50)
                        
        if abs(self.error_y) > self.MIN_PIX_ERR:
            self.y_motor.set_angle(self.tiltAngle)
                



class PiVideoStream:
	
    def __init__(self, resolution=(1280, 720), framerate=30):
                # initialize the camera and stream
                self.picam2 = Picamera2()
                self.picam2.preview_configuration.main.size = resolution
                self.picam2.preview_configuration.main.format = "RGB888"
                self.picam2.preview_configuration.controls.FrameRate=framerate
                self.picam2.preview_configuration.align()
                self.picam2.configure("preview")
                self.picam2.start()
            
                # if the thread should be stopped
                self.frame = []
                self.stopped = False
		
    def start(self):
        # start the thread to read frames from the video stream
        self.thread=Thread(target=self.update, args=())
        self.thread.daemon=True
        self.thread.start()

        return self
    
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            self.frame=self.picam2.capture_array()
            if self.stopped:
                    break
            
    def getFrame(self):
        return self.frame
    

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True



# dispW=int(640)
# dispH=int(360)
dispW=int(1280)
dispH=int(720)
frameRate = 30
fps=0
textPos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
textHeight=1.5
textWeight=3
textColor=(0,0,180)



# instatiating the class objects 
myCam = PiVideoStream((dispW,dispH), frameRate)
# starting the thread for capturing frames
myCam.start()

myTrack = Tracking(dispW=dispW, dispH=dispH,tiltAngle=-40)




# training models for face and eyes
faceCascade = cv2.CascadeClassifier('/home/gecko/Dokumente/camera_1_0/haar/haarcascade_frontalface_default.xml')
# eyesCascade = cv2.CascadeClassifier('/home/gecko/Dokumente/camera_1_0/haar/haarcascade_eye.xml')

while True:
    tStart=time.time()
    frame= myCam.getFrame()
    
    
 
    if len(frame):
        frame=cv2.flip(frame,-1) # flipping frame by 180 degrees
        frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(frameGray,1.3,5)
        # for face in faces:
        if len(faces):
            x_face, y_face, w_face, h_face = faces[0]
            cv2.rectangle(frame, (x_face,y_face),(x_face+w_face,y_face+h_face),(0,0,255),3)
            # roiGray=frameGray[y_face:y_face+h_face,x_face:x_face+h_face] # rows first than columns
            # roiColor=frame[y_face:y_face+h_face,x_face:x_face+h_face]   
            # eyes = eyesCascade.detectMultiScale(roiGray,1.3,5)
            # for eye in eyes:
            #     x, y, w, h = eye
            #     cv2.rectangle(roiColor, (x,y),(x+w,y+h),(255,0,0),3)
            
            dim=(x_face,y_face,w_face,h_face)
            myTrack.movement(dim)
            
            
        

        
        cv2.putText(frame,str(int(fps))+' FPS',textPos,font,textHeight,textColor,textWeight)
        cv2.imshow("Camera", frame)
        
        # cv2.imshow("My Mask", myMaskSmall)
        # cv2.imshow("OOI", objectOfInterestSmall)

    
    if cv2.waitKey(1)==ord('q'):
        myCam.stop()
        break
    
    tEnd=time.time()
    loopTime=tEnd-tStart
    # low pass filter (trust the previous fps value 90% / current one 10%)
    fps=.99*fps + .01*(1/loopTime)  # filtering high frequency changes

cv2.destroyAllWindows()

