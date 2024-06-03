import cv2
import time
import os #To handle directories

full_path = os.path.realpath(__file__)
WD = os.path.dirname(full_path)

cpt = 0
maxFrames = 40 # if you want 5 frames only.

cap=cv2.VideoCapture(0)

while cpt < maxFrames:
    ret, frame = cap.read()
    frame = cv2.flip(frame, -1) # Flip vertically
    frame=cv2.resize(frame,(640,480))
    cv2.imshow("test window", frame) # show image in window
    cv2.imwrite(WD + "/Face_images/LELE/Leandro_%d.jpg" %cpt, frame)
    time.sleep(0.5)
    cpt += 1
    if cv2.waitKey(1)==ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
