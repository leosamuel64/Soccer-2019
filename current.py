import math
import time
import sys
from holobot import Holobot
from collections import deque
import numpy as np
import imutils
import cv2
import os
import RPi.GPIO as GPIO

"""-----------------------------------------------------------------------------------
Préconditions  -- robot : la variable du robot
                  Le robot affiche des couleurs
Postconditions -- Le robot n'affiche plus de couleurs
-----------------------------------------------------------------------------------"""
def ledOFF(robot) :
    robot.set_leds(0,0,0)
    
    """--------------------------------------------------------------------------------------------
Préconditions  -- robot     : la variable du robot

Postconditions -- Renvoie une information sur le robot
-----------------------------------------------------------------------------------------------"""
def faux(robot):
    robot.beep(150, 200)
    #robot.set_leds(255,0,0)
    #time.sleep(1)
    #robot.set_leds(0,0,0)

def juste(robot):
    robot.beep(880, 200)
    #robot.set_leds(0,255,0)
    #time.sleep(1)
    #robot.set_leds(0,0,0)



"""-----------------------------------------------------------------------------------
Préconditions  -- robot : la variable du robot
Postconditions -- initialise le robot
-----------------------------------------------------------------------------------"""
def initHolo(robot):
    robot.reset_yaw()

# Set up obligatoire des pins GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(5, GPIO.OUT, initial=GPIO.HIGH)
		
def frapper():
	distance = holo.get_dist()
	if distance <= 11:
		tpsfrapper = 0.5
		GPIO.output(3,GPIO.LOW)
		GPIO.output(5,GPIO.LOW)
		time.sleep(tpsfrapper)
		GPIO.output(3,GPIO.HIGH)
		GPIO.output(5,GPIO.HIGH)
		time.sleep(tpsfrapper)
		GPIO.output(3,GPIO.HIGH)
		GPIO.output(5,GPIO.HIGH)
		
holo = Holobot(sys.argv[1], 115200)
initHolo(holo)

"""--------------------------------- Début du code Principal --------------------------------- """

greenLower = (0, 0, 200)
greenUpper = (80, 90, 255)

camera = cv2.VideoCapture(0)


while True:
    
    frapper()
	
    BALLE = False
	#lit le flux video
    (grabbed, frame) = camera.read()
	#resize la frame
    frame = imutils.resize(frame, width=600)
	# convertie en HSV
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	# applique le masque 
    mask = cv2.inRange(frame, greenLower, greenUpper)
    #mask = cv2.erode(mask, None, iterations=2)
    #mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)


        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            u=os.system('clear')
            #print("x = "+str(int(x)))
            #print("y = "+str(int(y)))
            print("Radius  = "+str(radius))
            pos = (300-int(x))*-1    
            print("Pos = "+str(pos))
            BALLE = True
            juste(holo)

        else:
            u=os.system('clear')
            print("PAS DE BALLE")
            radius = -1
            x = -1
            y = -1
            holo.stop_all()
            faux(holo)
            
            
    else:
        u=os.system('clear')
        print("PAS DE BALLE")
        radius = -1
        x = -1
        y = -1
        holo.stop_all()
        faux(holo)
        
        
    #cv2.imshow("Frame", frame)
    #cv2.imshow("Mask", mask)
    

    if BALLE == True :
        if (x < 270) and (x !=-1) :
            #gauche
            #tournerg(holo, 1)
            holo.turn(3)
            #holo.move_toward(30, 300)
            time.sleep(0.3)
            holo.stop_all()

        elif (x > 330) :
            #droite
            #tournerd(holo,1)
            holo.turn(-3)
            #holo.move_toward(30, 300)
            time.sleep(0.3)
            holo.stop_all()
        else : 
            holo.move_toward(30,300)
            time.sleep(1.76)
    else :
        
        holo.turn(4)
        time.sleep(0.3)
        holo.stop_all()
            
    #cv2.imshow("frame", frame)		

    key = cv2.waitKey(1) & 0xFF
camera.release()
cv2.destroyAllWindows()
