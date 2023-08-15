import cv2
import time
from picamera2 import Picamera2
from threading import Thread

from tflite_support.task import core, processor, vision

import utils


class PiVideoStream:
	
    def __init__(self, resolution=(1280, 720), framerate=30, webCam_Flag=False):
                
                self.model='efficientdet_lite0.tflite'
                self.num_threads = 4
                
                self.frame = []
                
                # initialize the camera and stream     
                self.picam2 = Picamera2()
                self.picam2.preview_configuration.main.size = resolution
                self.picam2.preview_configuration.main.format = "RGB888"
                self.picam2.preview_configuration.controls.FrameRate=framerate
                self.picam2.preview_configuration.align()
                self.picam2.configure("preview")
                self.picam2.start()
                
                # checking webcam --> bash: v4l2-ctl --list-devices
                self.webCam='/dev/video2'
                self.cam=cv2.VideoCapture(self.webCam)
                # self.cam.release()
                self.webCam_Flag = webCam_Flag
                self.ret = None
                
                self.base_options=core.BaseOptions(file_name=self.model,use_coral=False,num_threads=self.num_threads)
                self.detection_options=processor.DetectionOptions(max_results=5, score_threshold=.3)
                self.options=vision.ObjectDetectorOptions(base_options=self.base_options,detection_options=self.detection_options)
                self.detector=vision.ObjectDetector.create_from_options(self.options)
            
                # if the thread should be stopped
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
            if self.webCam_Flag:
                self.ret, self.frame = self.cam.read()
                
            else:
                self.frame=self.picam2.capture_array()
            
            if self.stopped:
                self.cam.release()
                break
            
    def getFrame(self):
        return self.frame
    

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True



# model='efficientdet_lite0.tflite'
# num_threads = 4



#picam2=Picamera2()
#picam2.preview_configuration.main.size=(dispW,dispH)
#picam2.preview_configuration.main.format='RGB888'
#picam2.preview_configuration.align()
#picam2.configure("preview")

#picam2.start()

# checking webcam --> bash: v4l2-ctl --list-devices
# webCam='/dev/video2'

# cam=cv2.VideoCapture(webCam)
# cam.release()
#cam.set(cv2.CAP_PROP_FRAME_WIDTH, int(640))
#cam.set(cv2.CAP_PROP_FRAME_HEIGHT, int(480))
#cam.set(cv2.CAP_PROP_FPS,15)


dispW=int(1280)
dispH=int(720)
frameRate = 30
fps=0
textPos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
textHeight=1.5
textWeight=3
textColor=(0,0,180)

webCam_Flag= False
#base_options=core.BaseOptions(file_name=model,use_coral=False,num_threads=num_threads)
#detection_options=processor.DetectionOptions(max_results=8, score_threshold=.3)
#options=vision.ObjectDetectorOptions(base_options=base_options,detection_options=detection_options)
#detector=vision.ObjectDetector.create_from_options(options)

# instatiating the class objects 
myCam = PiVideoStream(resolution=(dispW,dispH), framerate=frameRate, webCam_Flag=webCam_Flag)

# starting the thread for capturing frames
myCam.start()

# tStart=time.time()

while True:
    frame= myCam.getFrame()
    tStart=time.time()
    
    if len(frame):
        if not webCam_Flag:
            frame=cv2.flip(frame,-1) # flipping frame by 180 degrees
        # ret, im = cam.read()
        # im = picam2.capture_array()
        # im=cv2.flip(im,-1)
        imRGB=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        imTensor=vision.TensorImage.create_from_array(imRGB)
        detections=myCam.detector.detect(imTensor)
        image=utils.visualize(frame,detections)
        
        cv2.putText(frame,str(int(fps))+' FPS',textPos,font,textHeight,textColor,textWeight)
        cv2.imshow('Camera',frame)
        if cv2.waitKey(1)==ord('q'):
            myCam.stop()
            break
        
        tEnd=time.time()
        loopTime = tEnd - tStart
        fps = 0.8*fps + 1/loopTime
    

cv2.destroyAllWindows()
