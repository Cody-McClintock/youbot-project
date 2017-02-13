#!/usr/bin/env python

#importing modules
 
import cv2   
import numpy as np

import rospy
from youbot_ros_hello_world.msg import Num 

from std_msgs.msg import String

Px = 0
Py = 0
Mx = 0
My = 0
Cx = 0
Cy = 0
x = 0
    
#capturing video through webcam. If using an external webcame, change the "0" to "1"
cap=cv2.VideoCapture(0)

#talker() is how we send info to the cpp file
def talker():
    pub = rospy.Publisher('centroids', Num)
    rospy.init_node('centroids_talker', anonymous=True)
    msg= Num()
    msg.Pepsi_x = Px
    msg.Pepsi_y = Py
    msg.DietMTD_x = Mx
    msg.DietMTD_y = My
    msg.Coke_x = Cx
    msg.Coke_y = Cy
    rospy.loginfo(msg)
    pub.publish(msg)


#This is the main program.
def camera():

    _, original = cap.read()
    _, img = cap.read()

	#crops the imag. Original is 480x640
    img = img[200:400, 0:640]

	#blurs the image to eliminate noise
    img = cv2.medianBlur(img,15)
         
    #converting to HSV (hue-saturation-value)
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
 
    #definig the range of Coke color
    red_lower=np.array([136,87,111],np.uint8)
    red_upper=np.array([180,255,255],np.uint8)
 
    #defining the Range of Pepsi lid
    Pepsi_lower=np.array([105,110,50],np.uint8)
    Pepsi_upper=np.array([119,200,200],np.uint8)
     
    #defining the Range of Diet Mt. Dew lid (DietMTD/white)
    DietMTD_lower=np.array([75,0,228],np.uint8)
    DietMTD_upper=np.array([103,52,255],np.uint8)

    #finding the range of red,Pepsi and DietMTD color in the image
    red=cv2.inRange(hsv, red_lower, red_upper)
    Pepsi=cv2.inRange(hsv,Pepsi_lower,Pepsi_upper)
    DietMTD=cv2.inRange(hsv,DietMTD_lower,DietMTD_upper)
    
 
    #Morphological transformation, Dilation     
    kernal = np.ones((5 ,5), "uint8")
 
    red=cv2.dilate(red, kernal)
    res=cv2.bitwise_and(img, img, mask = red)

    Pepsi=cv2.dilate(Pepsi,kernal)
    res1=cv2.bitwise_and(img, img, mask = Pepsi)
 
    DietMTD=cv2.dilate(DietMTD,kernal)
    res2=cv2.bitwise_and(img, img, mask = DietMTD)    
 
 

    #Tracking the Coke Color
    (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
 
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area>300):
             
            x,y,w,h = cv2.boundingRect(contour) 
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(img,"RED color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

           	# compute the center of the contour
            M = cv2.moments(contour)
            global Cx
            global Cy
            Cx = int(M["m10"] / M["m00"])
            Cy = int(M["m01"] / M["m00"])
         
	        # draw the contour and center of the shape on the image
            cv2.circle(img, (Cx, Cy), 3, (255, 255, 255), -1)

             
    #Tracking the Pepsi Color
    (_,contours,hierarchy)=cv2.findContours(Pepsi,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area>500):
            x,y,w,h = cv2.boundingRect(contour) 
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.putText(img,"Pepsi",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0))

           	# compute the center of the contour
            M = cv2.moments(contour)
            global Px
            global Py
            Px = int(M["m10"] / M["m00"])
            Py = int(M["m01"] / M["m00"])
         
	        # draw the contour and center of the shape on the image
            cv2.circle(img, (Px, Py), 3, (255, 255, 255), -1)


    #Tracking the DietMTD Color
    (_,contours,hierarchy)=cv2.findContours(DietMTD,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area>500):
            x,y,w,h = cv2.boundingRect(contour) 
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(img,"Diet Mountain Dew",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))  

           	# compute the center of the contour
            M = cv2.moments(contour)
            global Mx
            global My
            Mx = int(M["m10"] / M["m00"])
            My = int(M["m01"] / M["m00"])
         
	        # draw the contour and center of the shape on the image
            cv2.circle(img, (Mx, My), 3, (255, 255, 255), -1)
             
     
       

    cv2.imshow("Color Tracking",img)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()


while not rospy.is_shutdown():
    camera()
    talker()

	#Lets me see the approx frames/second the webcam is getting
    x = x+1
    print(x)






