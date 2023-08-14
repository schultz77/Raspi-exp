import cv2
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO

# setting GPIO
GPIO.setmode(GPIO.BCM)
PBUTTON=14
GPIO.setup(PBUTTON, GPIO.IN,pull_up_down=GPIO.PUD_UP)

RED = 25
YELLOW = 26
ports = [RED, YELLOW]
for port in ports:
    GPIO.setup(port,GPIO.OUT)

# setting camera
picam2 = Picamera2()
dispW=1280
dispH=720 
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

class cam_v2():
    def __init__(self, fps, pos, font, height, weight, myColor, cam_flag_on):
        self.fps=fps
        self.pos=pos
        self.font=font
        self.height=height
        self.weight=weight
        self.myColor=myColor
        self.cam_flag_on=cam_flag_on
        
   
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)
cam_flag_on=False

def switch_cam(channel, my_cam):
    if my_cam.cam_flag_on:
        my_cam.cam_flag_on=False
        GPIO.output(ports[ports.index(YELLOW)],1)
        GPIO.output(ports[ports.index(RED)],0)
        
        
    else:
        my_cam.cam_flag_on=True
        GPIO.output(ports[ports.index(YELLOW)],0)
        GPIO.output(ports[ports.index(RED)],1)
        
        
my_cam = cam_v2(fps, pos, font, height, weight, myColor, cam_flag_on)
GPIO.add_event_detect(PBUTTON, GPIO.RISING,callback=lambda PBUTTON: switch_cam(PBUTTON, my_cam))
GPIO.output(ports[ports.index(YELLOW)],1)
GPIO.output(ports[ports.index(RED)],0)

while True:
    time.sleep(0.01)
    
    if my_cam.cam_flag_on:
        tStart=time.time()
        im= picam2.capture_array()
        cv2.putText(im,str(int(my_cam.fps))+' FPS',my_cam.pos,my_cam.font,my_cam.height,my_cam.myColor,my_cam.weight)
        cv2.imshow("Camera", im)
        if cv2.waitKey(1)==ord('q') or my_cam.cam_flag_on is False:
        #if my_cam.cam_flag_on is False:
            break
        tEnd=time.time()
        loopTime=tEnd-tStart
        #low pass filter (trust the previous fps value 90% / current one 10%)
        #filtering high frequency changes
        my_cam.fps=.9*my_cam.fps + .1*(1/loopTime)
cv2.destroyAllWindows()
GPIO.cleanup()
