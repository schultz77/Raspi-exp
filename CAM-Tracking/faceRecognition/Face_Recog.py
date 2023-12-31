#Program to Detect the Face and Recognise the Person based on the data from face-trainner.yml

import cv2 #For Image processing 
import numpy as np #For converting Images to Numerical array 
import os #To handle directories 
from PIL import Image #Pillow lib for handling images
import time

from Frame_Capture import piVideoStream

from gtts import gTTS
from playsound import playsound


full_path = os.path.realpath(__file__)
WD = os.path.dirname(full_path)

labels = ["cle", "LELE", "Sonja", "Luke"]
# labels = ["cle", "LELE"]  

face_cascade = cv2.CascadeClassifier(WD + '/haarcascade_frontalface_default.xml')
recognizer = cv2.face.LBPHFaceRecognizer_create()
trainer = WD + "/face-trainner.yml"
recognizer.read(trainer)

dispW = int(1280*0.8)
dispH = int(720*0.8)
frameRate = 30
fps=0
textPos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
textHeight=1
textWeight=3
textColor=(0,0,180)


# instatiating the class objects 
myCap = piVideoStream(resolution=(dispW,dispH), framerate=frameRate)
# starting the thread for capturing frames
myCap.start()

first_time = True

while(True):
  img= myCap.getFrame()
  tStart=time.time()
  
  if len(img):
    img = cv2.flip(img, -1) # Flip vertically 
    gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert Video frame to Greyscale
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, 
                                          minSize = (int(myCap.minW), int(myCap.minH))) #Recog. faces
    for (x, y, w, h) in faces:
        roi_gray = gray[y:y+h, x:x+w] #Convert Face to greyscale 

        id_, conf = recognizer.predict(roi_gray) #recognize the Face
    
        if conf>=60:
          name = labels[id_] #Get the name from the List using ID number 
          cv2.putText(img, name, (x,y-5), font, textHeight, textColor, textWeight-1)

          # if name == 'cle' and first_time:
          #   language = 'pt-br'
          #   tts = gTTS(text="Ola Clemente, tudo bem? O que voce vai fazer hoje?",
          #             lang=language,
          #             slow=False)

          #   tts.save('./tts.mp3')
          #   playsound('./tts.mp3')
          #   first_time = False
    	
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

    cv2.putText(img,str(int(fps))+' FPS',textPos,font,textHeight,textColor,textWeight)
    cv2.imshow('Preview',img) #Display the Video
    if cv2.waitKey(1)==ord('q'):
        myCap.stop() # When everything done, release the capture
        break

    tEnd=time.time()
    loopTime = tEnd - tStart
    fps = 0.8*fps + 1/loopTime
# When everything done, release the capture
cv2.destroyAllWindows()
