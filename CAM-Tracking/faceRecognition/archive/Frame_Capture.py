import cv2 #For Image processing
from threading import Thread 

class piVideoStream:
	
    def __init__(self, resolution=(1280, 720), framerate=30):
                               
        self.frame = []
        
        self.cam = cv2.VideoCapture(0) #Get vidoe feed from the Camera
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0]) # set video widht
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1]) # set video height
        self.cam.set(cv2.CAP_PROP_FPS,framerate)
        
        # Define min window size to be recognized as a face
        self.minW = 0.1*self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.minH = 0.1*self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.ret = None             
    
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
            self.ret, self.frame = self.cam.read()
            if self.stopped:
                self.cam.release()
                break
            
    def getFrame(self):
        return self.frame
    

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
