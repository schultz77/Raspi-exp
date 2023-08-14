# import the necessary packages
from picamera2 import Picamera2
from threading import Thread
import cv2
import numpy as np
import time
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from buildhat import Motor

x_motor = Motor('A')
y_motor = Motor('B')



panAngle =0
tiltAngle=0
x_motor.run_to_position(panAngle)
y_motor.run_to_position(tiltAngle)
Track = 0

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
frameRate = 30
fps=0
textPos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
textHeight=1.5
textWeight=3
textColor=(0,0,180)


cv2.namedWindow('My Trackbars')
cv2.createTrackbar('Hue Low', 'My Trackbars',0,360,TrackBars)
cv2.createTrackbar('Hue High', 'My Trackbars', 42,360,TrackBars)
cv2.createTrackbar('Sat Low', 'My Trackbars', 88,255,TrackBars)
cv2.createTrackbar('Sat High', 'My Trackbars', 255,255,TrackBars)
cv2.createTrackbar('Value Low', 'My Trackbars', 194,255,TrackBars)
cv2.createTrackbar('Value High', 'My Trackbars', 255,255,TrackBars)
cv2.createTrackbar('Train-Track', 'My Trackbars', 0, 1,TrackBars)

MAX_PAN_ANGLE = 35
MAX_TILT_ANGLE = 35
MIN_PIXEL_ERROR = 40
PIXEL2DEGREE_RATE = 150

cog_contour_x = 0
cog_contour_y = 0
error_x = 0
error_y = 0
cnt = 0

error_x_array = []
error_y_array = []
panAngle_array_calc = []
tiltAngle_array_calc = []
panAngle_array_pos = []
tiltAngle_array_pos = []
time_array = []
pos_motor_x_array = []
pos_motor_y_array = []
active_x_array = []
active_y_array = []
tx_active_array = []
ty_active_array = []

myCam =PiVideoStream((dispW,dispH), frameRate)
myCam.start()

while True:
    tStart=time.time()
    frame= myCam.getFrame()
    
 
    if len(frame):

        lowerBound = np.array([hueLow, satLow, valLow])
        upperBound = np.array([hueHigh, satHigh, valHigh])

        frameHSV = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
        
        myMask = cv2.inRange(frameHSV, lowerBound, upperBound)
        # myMaskSmall=cv2.resize(myMask, (int(dispW/2), int(dispH/2)))

        # objectOfInterest=cv2.bitwise_and(frame, frame,mask=myMask)
        # objectOfInterestSmall = cv2.resize(objectOfInterest, (int(dispW/2), int(dispH/2)))
        
        contours, junk = cv2.findContours(myMask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)       
        if len(contours):
            contours=sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
            # cv2.drawContours(frame,contours, 0, (0,255,0),3) #-1 for all contours
            contour=contours[0]
            x,y,w,h=cv2.boundingRect(contour)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),3)

            if Track == 1:
                # calculating the center of gravity of contour
                if cog_contour_x:
                    cog_contour_x =  0.8*cog_contour_x + 0.2*(x+w/2) # low-pass filter
                else:
                     cog_contour_x = x+w/2
                     
                if cog_contour_y:
                     cog_contour_y = 0.8*cog_contour_y + 0.2*(y+h/2) # low-pass filter
                else:
                     cog_contour_y = y+h/2

                # calculating distance between cog_contour and the center of the window
                error_x = cog_contour_x - dispW/2
                panAngle += error_x/PIXEL2DEGREE_RATE
               
                if panAngle > MAX_PAN_ANGLE:
                    panAngle = MAX_PAN_ANGLE
                if panAngle < -MAX_PAN_ANGLE:
                    panAngle = -MAX_PAN_ANGLE
                
                if abs(error_x) > MIN_PIXEL_ERROR:
                    active_x_array.append(50)
                    tx_active_array.append(time.time())
                    x_motor.run_to_position(panAngle)
                    print('panAngle', panAngle)
                else:
                    active_x_array.append(0)
                    tx_active_array.append(time.time())


                error_y = cog_contour_y - dispH/2
                tiltAngle -= error_y/PIXEL2DEGREE_RATE
                                                
                if tiltAngle > MAX_TILT_ANGLE:
                    tiltAngle = MAX_TILT_ANGLE
                if tiltAngle < -MAX_TILT_ANGLE:
                    tiltAngle=-MAX_TILT_ANGLE
                                
                if abs(error_y) > MIN_PIXEL_ERROR:
                    active_y_array.append(50)
                    ty_active_array.append(time.time())
                    print('tiltAngle', tiltAngle)
                    y_motor.run_to_position(tiltAngle)
                else:
                    active_y_array.append(0)
                    ty_active_array.append(time.time())
                     

                

        error_x_array.append(error_x)
        panAngle_array_calc.append(panAngle)
        error_y_array.append(error_y)
        tiltAngle_array_calc.append(tiltAngle)
        pos_motor_x_array.append(x_motor.get_aposition())
        pos_motor_y_array.append(y_motor.get_aposition())
        time_array.append(time.time())
         
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

df1 = pd.DataFrame({'error_x': error_x_array, 'error_y': error_y_array, 
                    'panAngle_calc': panAngle_array_calc, 'tiltAngle_calc':tiltAngle_array_calc})
df2 = pd.DataFrame({'time': time_array, 'pos_motor_x_array': pos_motor_x_array, 'pos_motor_y_array': pos_motor_y_array}) 
                    

df = pd.concat([df1,df2], axis=1)

df_active_x = pd.DataFrame({'active_x_array': active_x_array,'tx_active_array': tx_active_array })
df_active_y = pd.DataFrame({'active_y_array': active_y_array,'ty_active_array': ty_active_array })

df_active = pd.concat([df_active_x, df_active_y], axis=1)


# Create traces
fig = make_subplots(
     rows=2, cols=1,
     shared_xaxes=True,
     vertical_spacing=0.15,
     subplot_titles=('pan','tilt')

)

fig.add_trace(go.Scatter(y=df['error_x'], x=df['time'],
                    mode='lines',
                    name='error_x', legendgroup=1), row=1, col=1)
fig.add_trace(go.Scatter(y=df['panAngle_calc'], x=df['time'],
                    mode='lines',
                    name='x.run_to_position()', legendgroup=1), row=1, col=1)
fig.add_trace(go.Scatter(y=df['pos_motor_x_array'], x=df['time'],
                    mode='lines',
                    name='x.get_aposition()', legendgroup=1), row=1, col=1)
fig.add_trace(go.Scatter(y=df_active['active_x_array'], x=df_active['tx_active_array'],
                    mode='lines',
                    name='active_x_array', legendgroup=1), row=1, col=1)


fig.add_trace(go.Scatter(y=df['error_y'], x=df['time'],
                    mode='lines',
                    name='error_y', legendgroup=2), row=2, col=1)
fig.add_trace(go.Scatter(y=df['tiltAngle_calc'], x=df['time'],
                    mode='lines',
                    name='y.run_to_position()', legendgroup=2), row=2, col=1)
fig.add_trace(go.Scatter(y=df['pos_motor_y_array'], x=df['time'],
                    mode='lines',
                    name='y.get_aposition()', legendgroup=2), row=2, col=1)
fig.add_trace(go.Scatter(y=df_active['active_y_array'], x=df_active['ty_active_array'],
                    mode='lines',
                    name='active_y_array', legendgroup=2), row=2, col=1)


fig['layout']['yaxis']['title']='Angle [deg] / Pixels'
fig['layout']['yaxis2']['title']='Angle [deg] / Pixels'

# fig['layout']['xaxis']['title']='Angle [deg] / Pixels'
fig['layout']['xaxis2']['title']='time'

fig.update_layout(legend_tracegroupgap=350, 
                  legend_groupclick='toggleitem',
                  title={
                        'text': "Camera Pan/Tilt tracking object",
                        'xanchor': 'left',
                        'yanchor': 'top'})
fig.show()
