# import the necessary packages
from picamera2 import Picamera2
from threading import Thread
import cv2
import numpy as np
import time
from servo import Servo
import os

# import pandas as pd
# import plotly.graph_objects as go
# from plotly.subplots import make_subplots
# from buildhat import Motor


class Tracking:

    __slots__ = "panAngle",'tiltAngle', 'motorPan','motorTilt','MAX_ANGLE','MIN_PIX_ERR','PIX2DEG_RATE','disp','filtering_box','panAngle_pre','tiltAngle_pre','cog_contour','low_freq_factor', 'x_motor', 'y_motor', 'error_x', 'error_y'


    def __init__(self, 
                 panAngle=0, 
                 tiltAngle=0, 
                 motorPan=13, 
                 motorTilt=12,
                 MAX_ANGLE={'PAN': 90, 'TILT': 40}, 
                 MIN_PIX_ERR=35, 
                 PIX2DEG_RATE=70,
                 disp={'width': 1280, 'height':720},
                 filtering_box = True,
                 panAngle_pre = 0,
                 tiltAngle_pre = 0,
                 cog_contour = {'x': 0, 'y': 0},
                 low_freq_factor = 0.8,
                 x_motor= None, y_motor= None, error_x = 0, error_y = 0) -> None:
         
         self.x_motor = Servo(motorPan)
         self.y_motor = Servo(motorTilt)
         self.panAngle = panAngle
         self.tiltAngle = tiltAngle
         self.MAX_ANGLE = MAX_ANGLE
         self.MIN_PIX_ERR = MIN_PIX_ERR
         self.PIX2DEG_RATE = PIX2DEG_RATE
         self.cog_contour = cog_contour
         self.low_freq_factor = low_freq_factor
         self.disp = disp
         self.filtering_box = filtering_box
         self.panAngle_pre = panAngle_pre
         self.tiltAngle_pre = tiltAngle_pre
         self.error_x = error_x
         self.error_y = error_y

        #  self.error_x_prev = 0
        #  self.error_x_sum = 0
         

         self.x_motor.set_angle(self.panAngle)
         self.y_motor.set_angle(self.tiltAngle)

    

    def low_pass_filter(self, dim={'x_box':0,'y_box':0,'width_box': 0, 'height_box': 0}):
                 
        if self.cog_contour['x'] and self.filtering_box:
           self.cog_contour['x'] = self.low_freq_factor * self.cog_contour['x'] + (1-self.low_freq_factor) * (dim['x_box']+dim['width_box']/2)
        else:
           self.cog_contour['x'] = dim['x_box']+dim['width_box']/2
        
        if self.cog_contour['y'] and self.filtering_box:
            self.cog_contour['y'] = self.low_freq_factor * self.cog_contour['y'] + (1-self.low_freq_factor) * (dim['y_box']+dim['height_box']/2)
        else:
            self.cog_contour['y'] = dim['y_box']+dim['height_box']/2
        

    # def start(self):
    #     # start the thread for moving the motors
    #     self.mov_thread=Thread(target=self.movement, args=())
    #     self.mov_thread.daemon=True
    #     self.mov_thread.start()

    
    def movement(self, dim={'x_box':0,'y_box':0,'width_box': 0, 'height_box': 0}):
    
        
        self.low_pass_filter(dim)

        # calculating distance between cog_contour and the center of the window - PID control
        KP_x = 1/self.PIX2DEG_RATE
        KD_x = KP_x/2
        KI_x = KD_x/2

        # calculating distance between cog_contour and the center of the window
        self.error_x = -(self.cog_contour['x'] - self.disp['width']/2)
        self.panAngle += self.error_x/self.PIX2DEG_RATE
        
        # PID control
        # self.panAngle += (self.error_x*KP_x) + (self.error_x_prev*KD_x) + (self.error_x_sum*KI_x)
        # self.error_x_prev = self.error_x
        # self.error_x_sum += self.error_x

        # print('cog_contour[x]: {} KP: {}'.format(self.cog_contour['x'], self.error_x/self.PIX2DEG_RATE ))
        # print('error_x: {} panAngle: {} pan_angle_pre {}'.format(self.error_x, self.panAngle, self.panAngle_pre ))
        
        if self.panAngle < -self.MAX_ANGLE['PAN']:
            self.panAngle = -self.MAX_ANGLE['PAN']
        if self.panAngle > self.MAX_ANGLE['PAN']:
            self.panAngle = self.MAX_ANGLE['PAN']
        
        if abs(self.error_x) > self.MIN_PIX_ERR and abs(abs(self.panAngle) - abs(self.panAngle_pre)) > 5:
            # print('error_x_in:: {} panAngle_cmd: {}'.format(self.error_x, self.panAngle ))
            self.x_motor.set_angle(self.panAngle) # (+) conterclockwise --> right / (-) clockwise --> left
            self.panAngle_pre = self.panAngle
            
        # print('panAngle_actual: ', self.x_motor.get_angle())

        self.error_y = self.cog_contour['y'] - self.disp['height']/2
        self.tiltAngle += self.error_y/self.PIX2DEG_RATE
                                        
        if self.tiltAngle > self.MAX_ANGLE['TILT']:
            self.tiltAngle = self.MAX_ANGLE['TILT']
        if self.tiltAngle < -self.MAX_ANGLE['TILT']:
            self.tiltAngle=-self.MAX_ANGLE['TILT']
                        
        if abs(self.error_y) > self.MIN_PIX_ERR and abs(abs(self.tiltAngle) - abs(self.tiltAngle_pre)) > 5:
            self.y_motor.set_angle(self.tiltAngle) # (+) down / (-) up
            self.tiltAngle_pre = self.tiltAngle
            

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


dispW=int(1280)
dispH=int(720)
frameRate = 30
fps=0
textPos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
textHeight=1.5
textWeight=3
textColor=(0,0,180)
full_path = os.path.realpath(__file__)
WD = os.path.dirname(full_path)


# instatiating the class objects 
myCam = PiVideoStream((dispW,dispH), frameRate)
# starting the thread for capturing frames
myCam.start()

myTrack = Tracking(tiltAngle=-30, disp={'width': dispW, 'height':dispH}, filtering_box=True)


# training models for face and eyes
faceCascade = cv2.CascadeClassifier(WD + '/haar/haarcascade_frontalface_default.xml')
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
            
            dim={'x_box': x_face,'y_box':y_face,'width_box':w_face, 'height_box': h_face}
            myTrack.movement(dim)
            
         
        cv2.putText(frame,str(int(fps))+' FPS',textPos,font,textHeight,textColor,textWeight)
        cv2.imshow("Camera", frame)
        
          
    if cv2.waitKey(1)==ord('q'):
        myCam.stop()
        myTrack.x_motor.set_angle(0)
        myTrack.y_motor.set_angle(-30)
        break
    
    tEnd=time.time()
    loopTime=tEnd-tStart
    # low pass filter (trust the previous fps value 90% / current one 10%)
    fps=.99*fps + .01*(1/loopTime)  # filtering high frequency changes

cv2.destroyAllWindows()

