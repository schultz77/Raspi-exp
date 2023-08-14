import cv2
from picamera2 import Picamera2
import time
picam2 = Picamera2()
dispW=1280
dispH=720
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)
mythickness = -1

top_left_col=25
top_left_row=25
box_width=200
box_hight=100
botton_right_col=top_left_col + box_width
botton_right_row=top_left_row + box_hight


delta_column= 5
delta_row= 5

while True:
    tStart=time.time()
    im= picam2.capture_array()
    cv2.putText(im,str(int(fps))+' FPS',pos,font,height,myColor,weight)
    if top_left_col + delta_column < 0 or botton_right_col + delta_column > dispW -1:
        delta_column *= -1
        
    if top_left_row + delta_row < 0 or botton_right_row + delta_row > dispH -1:
        delta_row *= -1
    
    top_left_col += delta_column
    top_left_row += delta_row
    botton_right_col += delta_column
    botton_right_row += delta_row
    
    
    upperLeft = (top_left_col,top_left_row)
    lowerRight = (botton_right_col, botton_right_row)
    cv2.rectangle(im, upperLeft, lowerRight, myColor, mythickness)
    
    cv2.imshow("Camera", im)
    if cv2.waitKey(1)==ord('q'):
        break
    tEnd=time.time()
    loopTime=tEnd-tStart
    #low pass filter (trust the previous fps value 90% / current one 10%)
    #filtering high frequency changes
    fps=.9*fps + .1*(1/loopTime)  
cv2.destroyAllWindows()
