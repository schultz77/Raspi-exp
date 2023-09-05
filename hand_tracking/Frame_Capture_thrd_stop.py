import cv2  # For Image processing
import threading


class piVideoStream(threading.Thread):

    def __init__(self, resolution=(1280, 720), framerate=30):
        super().__init__()
        self.stop_flag = threading.Event()

        self.frame = []

        self.cam = cv2.VideoCapture(0)  # Get video feed from the Camera
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])  # set video widht
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])  # set video height
        self.cam.set(cv2.CAP_PROP_FPS, framerate)

        # Define min window size to be recognized as a face
        self.minW = 0.1 * self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.minH = 0.1 * self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.ret = None

    # def start(self):
    #     # start the thread to read frames from the video stream
    #     self.thread = Thread(target=self.update, args=())
    #     self.thread.daemon = True
    #     self.thread.start()
    #
    #     return self

    def run(self):
        # keep looping infinitely until the thread is stopped
        while not self.stop_flag.is_set():
            self.ret, self.frame = self.cam.read()

    def getFrame(self):
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.cam.release()
        self.stop_flag.set()
