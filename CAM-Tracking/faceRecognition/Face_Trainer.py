#Program to train with the faces and create a YAML file

import cv2 #For Image processing 
import numpy as np #For converting Images to Numerical array 
import os #To handle directories 
from PIL import Image #Pillow lib for handling images

full_path = os.path.realpath(__file__)
WD = os.path.dirname(full_path)

face_cascade = cv2.CascadeClassifier(WD + '/haarcascade_frontalface_default.xml')
recognizer = cv2.face.LBPHFaceRecognizer_create()

Face_ID = -1 
pev_person_name = ""
y_ID = []
x_train = []

Face_Images = os.path.join(WD, "Face_images") #Tell the program where we have saved the face images 
print (Face_Images)

for root, dirs, files in os.walk(Face_Images): #go to the face image directory 
    for file in files: #check every directory in it 
            if file.endswith("jpeg") or file.endswith("jpg") or file.endswith("png"): #for image files ending with jpeg,jpg or png
                path = os.path.join(root, file)
                person_name = os.path.basename(root)
                print(path, person_name)

            
            if pev_person_name!=person_name: #Check if the name of person has changed 
                Face_ID=Face_ID+1 #If yes increment the ID count 
                pev_person_name = person_name

            
            Grey_Image = Image.open(path).convert("L") # convert the image to greysclae using Pillow
            #Crop_Image = Gery_Image.resize( (800,800) , Image.ANTIALIAS) #Crop the Grey Image to 550*550 (Make sure your face is in the center in all image)
            Crop_Image = Grey_Image.resize( (800,800) , Image.Resampling.LANCZOS) #Crop the Grey Image to 550*550 (Make sure your face is in the center in all image)
            Final_Image = np.array(Crop_Image, "uint8")
            #print(Numpy_Image)
            faces = face_cascade.detectMultiScale(Final_Image, scaleFactor=1.2, minNeighbors=5) #Detect The face in all sample image 
            print (Face_ID,faces)

            for (x,y,w,h) in faces:
                roi = Final_Image[y:y+h, x:x+w] #crop the Region of Interest (ROI)
                x_train.append(roi)
                y_ID.append(Face_ID)

recognizer.train(x_train, np.array(y_ID)) #Create a Matrix of Training data 
recognizer.save(WD + "/face-trainner.yml") #Save the matrix as YML file 


