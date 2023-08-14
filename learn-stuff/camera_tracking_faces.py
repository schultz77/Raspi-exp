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

    def __init__(self, 
                 panAngle=0, 
                 tiltAngle=0, 
                 motorPan=13, 
                 motorTilt=12,
                 MAX_ANGLE={'PAN': 90, 'TILT': 40}, 
                 MIN_PIX_ERR=35, 
                 PIX2DEG_RATE=70,
                 disp={'width': 1280, 'height':720},
                 filtering_box = True) -> None:
         
         self.x_motor = Servo(motorPan)
         self.y_motor = Servo(motorTilt)
         self.panAngle = panAngle
         self.tiltAngle = tiltAngle
         self.MAX_ANGLE = MAX_ANGLE
         self.MIN_PIX_ERR = MIN_PIX_ERR
         self.PIX2DEG_RATE = PIX2DEG_RATE
         self.cog_contour = {'x': 0, 'y': 0}
         self.low_freq_factor = 0.8
         self.disp = disp
         self.filtering_box = filtering_box

         self.x_motor.set_angle(self.panAngle)
         self.y_motor.set_angle(self.tiltAngle)

    def start(self, dim):
        # start the thread to read frames from the video stream
        self.secthread=Thread(target=self.movement, args=(dim,))
        self.secthread.daemon=True
        self.secthread.start()

    def low_pass_filter(self, dim={'x_box':0,'y_box':0,'width_box': 0, 'height_box': 0}):
                 
        if self.cog_contour['x'] and self.filtering_box:
           self.cog_contour['x'] = self.low_freq_factor * self.cog_contour['x'] + (1-self.low_freq_factor) * (dim['x_box']+dim['width_box']/2)
        else:
           self.cog_contour['x'] = dim['x_box']+dim['width_box']/2
        
        if self.cog_contour['y'] and self.filtering_box:
            self.cog_contour['y'] = self.low_freq_factor * self.cog_contour['y'] + (1-self.low_freq_factor) * (dim['y_box']+dim['height_box']/2)
        else:
            self.cog_contour['y'] = dim['y_box']+dim['height_box']/2
        

         
    
    def movement(self, dim={'x_box':0,'y_box':0,'width_box': 0, 'height_box': 0}):
        
        self.low_pass_filter(dim)

        # calculating distance between cog_contour and the center of the window
        self.error_x = self.cog_contour['x'] - self.disp['width']/2
        self.panAngle -= self.error_x/self.PIX2DEG_RATE
        
        if self.panAngle > self.MAX_ANGLE['PAN']:
            self.panAngle = self.MAX_ANGLE['PAN']
        if self.panAngle < -self.MAX_ANGLE['PAN']:
            self.panAngle = -self.MAX_ANGLE['PAN']
        
        if abs(self.error_x) > self.MIN_PIX_ERR:
            self.x_motor.set_angle(self.panAngle)
        

        self.error_y = self.cog_contour['y'] - self.disp['height']/2
        self.tiltAngle += self.error_y/self.PIX2DEG_RATE
                                        
        if self.tiltAngle > self.MAX_ANGLE['TILT']:
            self.tiltAngle = self.MAX_ANGLE['TILT']
        if self.tiltAngle < -self.MAX_ANGLE['TILT']:
            self.tiltAngle=-self.MAX_ANGLE['TILT']
                        
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



dispW=int(1280)
dispH=int(720)
frameRate = 30
fps=0
textPos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
textHeight=1.5
textWeight=3
textColor=(0,0,180)


x_motor = Servo(13)
y_motor = Servo(12)



panAngle =0
tiltAngle=-40
x_motor.set_angle(panAngle)
y_motor.set_angle(tiltAngle)

MAX_PAN_ANGLE = 90
MAX_TILT_ANGLE = 40
MIN_PIXEL_ERROR = 35
PIXEL2DEGREE_RATE = 70

cog_contour_x = 0
cog_contour_y = 0
error_x = 0
error_x_prev = 0
error_x_sum = 0
error_y = 0



# instatiating the class objects 
myCam = PiVideoStream((dispW,dispH), frameRate)
# starting the thread for capturing frames
myCam.start()

myTrack = Tracking(tiltAngle=-40, disp={'width': dispW, 'height':dispH}, filtering_box=False)




# training models for face and eyes
faceCascade = cv2.CascadeClassifier('/home/gecko/Dokumente/camera_1_0/haar/haarcascade_frontalface_default.xml')
eyesCascade = cv2.CascadeClassifier('/home/gecko/Dokumente/camera_1_0/haar/haarcascade_eye.xml')

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
            #myTrack.start(dim)

            # # calculating the center of gravity of contour
            # # if cog_contour_x:
            # #     cog_contour_x =  0.8*cog_contour_x + 0.2*(x_face+w_face/2) # low-pass filter
            # # else:
            # #         cog_contour_x = x_face+w_face/2
                    
            # # if cog_contour_y:
            # #         cog_contour_y = 0.8*cog_contour_y + 0.2*(y_face+h_face/2) # low-pass filter
            # # else:
            # #         cog_contour_y = y_face+h_face/2

            # cog_contour_x = x_face+w_face/2
            # cog_contour_y = y_face+h_face/2
            
            
            # # calculating distance between cog_contour and the center of the window - PID control
            # KP_x = 1/PIXEL2DEGREE_RATE
            # KD_x = KP_x/2
            # KI_x = KD_x/2
            
            
            # error_x = cog_contour_x - dispW/2
            # #panAngle -= (error_x*KP_x) + (error_x_prev*KD_x) + (error_x_sum*KI_x)
            # panAngle -= (error_x*KP_x)
            # #print('panAngle: {} / error_x: {}'.format(panAngle, error_x))
            # error_x_prev = error_x
            # error_x_sum += error_x

            # if panAngle > MAX_PAN_ANGLE:
            #     panAngle = MAX_PAN_ANGLE
            # if panAngle < -MAX_PAN_ANGLE:
            #     panAngle = -MAX_PAN_ANGLE
            
            # if abs(error_x) > MIN_PIXEL_ERROR:
            #     print('panAngle: {} / error_x: {}'.format(panAngle, error_x))
            #     x_motor.set_angle(panAngle)
                
           
            # error_y = cog_contour_y - dispH/2
            # tiltAngle += error_y/PIXEL2DEGREE_RATE
            # print('tiltAngle: {} / error_y: {}'.format(tiltAngle, error_y))
                                            
            # if tiltAngle > MAX_TILT_ANGLE:
            #     tiltAngle = MAX_TILT_ANGLE
            # if tiltAngle < -MAX_TILT_ANGLE:
            #     tiltAngle=-MAX_TILT_ANGLE
                            
            # if abs(error_y) > MIN_PIXEL_ERROR:
            #     y_motor.set_angle(tiltAngle)
           
            
        

        
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

