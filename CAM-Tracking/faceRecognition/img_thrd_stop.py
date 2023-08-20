import cv2
import time
import os  # To handle directories
from Frame_Capture_thrd_stop import piVideoStream

full_path = os.path.realpath(__file__)
WD = os.path.dirname(full_path)

cpt = 0
maxFrames = 40  # if you want 5 frames only.

dispW = int(1280 * 0.8)
dispH = int(720 * 0.8)
frameRate = 30

# instantiating the class objects
myCap = piVideoStream(resolution=(dispW, dispH), framerate=frameRate)
# starting the thread for capturing frames
myCap.start()

while cpt < maxFrames:
    frame = myCap.getFrame()

    if len(frame):
        # frame = cv2.flip(frame, -1)  # Flip vertically
        frame = cv2.resize(frame, (640, 480))
        cv2.imshow("test window", frame)  # show image in window
        cv2.imwrite(WD + "/Face_images/LELE/Leandro_%d.jpg" % cpt, frame)
        time.sleep(0.5)
        cpt += 1
        if cv2.waitKey(1) == ord('q'):
            myCap.stop()  # When everything done, release the capture
            myCap.join()
            break

myCap.stop()  # When everything done, release the capture
myCap.join()
cv2.destroyAllWindows()
