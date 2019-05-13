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



"""-----------------------------------------------------------------------------------
Préconditions  -- diametre : type entier ; valeur du diametre de la balle en pixels
Postconditions -- renvoie la distance au sol entre le robot et la balle (en cm)
-----------------------------------------------------------------------------------"""
def calcDistance(diametre):
    distance = 3315.2*(diametre**-1.014)
    return distance



"""-----------------------------------------------------------------------------------
Préconditions  -- xBalle : type entier ; coordonnée x de la balle sur l'image
Postconditions -- renvoie l'angle au sol entre le robot et la balle (en degrés)
-----------------------------------------------------------------------------------"""
def calcAngle(xBalle):
    xCentral = 320
    AB = math.sqrt((xBalle-xCentral)**2) #Calcul de la distance en x de la balle par rapport au point central.
    angle = 0.113970*AB #Calcul de l'angle en fonction de la distance en x.
    if(xBalle < xCentral):
        angle = angle-29 #Gauche --> angle < 29 || Droite --> angle > 29.
    return angle



"""-----------------------------------------------------------------------------------
Préconditions  -- robot : la variable du robot
Postconditions -- initialise le robot
-----------------------------------------------------------------------------------"""
def initHolo(robot):
    robot.reset_yaw()



"""-----------------------------------------------------------------------------------
Préconditions  -- robot : la variable du robot
                  angle : type entier ; l'angle voulu en degrés
Postconditions -- déplace le robot vers la droite
-----------------------------------------------------------------------------------"""
def tournerd(robot, angle):
    #Constantes :
    dt = 0.1
    vitesse = 50
    t = 0.0

    #Calculs :
    angle = (angle*40)/180
    temps = angle/vitesse



    #Tourner :
    while(t < temps):
        robot.turn(-vitesse)
        time.sleep(dt)
        t += dt
    robot.stop_all()

"""-----------------------------------------------------------------------------------
Préconditions  -- robot : la variable du robot
                  angle : type entier ; l'angle voulu en degrés
Postconditions -- déplace le robot vers la gauche
-----------------------------------------------------------------------------------"""
def tournerg(robot, angle):
    #Constantes :
    dt = 0.1
    vitesse = 50
    t = 0.0

    #Calculs :
    angle = (angle*40)/180
    temps = angle/vitesse



    #Tourner :
    while(t < temps):
        robot.turn(vitesse)
        time.sleep(dt)
        t += dt
    robot.stop_all()




"""-----------------------------------------------------------------------------------
Préconditions  -- robot    : la variable du robot
                  distance : type entier ; la distance souhaitée pour le déplacement en cm
                  angle    : type entier ; l'angle voulu en degrés
Postconditions -- déplace le robot
-----------------------------------------------------------------------------------"""
def bouger(robot, distance, angle):
    #Constantes :
    dt = 0.1
    vitesse = 100
    t = 0.0

    #Calculs :
    distance = distance*10
    temps = distance/vitesse

    #Bouger :
    tourner(robot, angle)
    time.sleep(dt)
    while(t < temps):
        robot.move_toward(vitesse, 300)
        time.sleep(dt)
        t += dt
    robot.stop_all()

holo = Holobot(sys.argv[1], 115200)
initHolo(holo)

"""-----------------------------------------------------------------------------------------------
Préconditions   -- robot     : la variable du robot
                   inconnu   : doit toujours être 1


Postconditions  -- renvoie la distance du robot a un obstacle
-----------------------------------------------------------------------------------------------"""
def dist(robot, inconnu):
    distance = robot.get_dist(inconnu)
    return distance



"""-----------------------------------------------------------------------------------------------
Préconditions   -- duree    : temps durant lequel le robot continue de frapper
                   cadence  : Nombre de coups par seconde


Postconditions  -- Ordonne au robot de frapper
-----------------------------------------------------------------------------------------------"""

# Set up obligatoire des pins GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3, GPIO.OUT)
GPIO.setup(3, GPIO.OUT, initial=GPIO.HIGH)

def frapper(duree, cadence):
	t = 0
	while t < duree :
		GPIO.output(3, not GPIO.input(3))
		pause = 1/cadence
		time.sleep(pause)
		t += pause




"""--------------------------------- Début du code Principal --------------------------------- """

greenLower = (5, 140, 185)
greenUpper = (30, 215, 255)


camera = cv2.VideoCapture(0)


while True:

    BALLE = False

    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
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
           # print("x = "+str(int(x)))
            #print("y = "+str(int(y)))
            print("Radius  = "+str(radius))
            pos = (300-int(x))*-1    # position par rapport au centre de l'image x=300
            print("Pos = "+str(pos))
            BALLE = True

        else:
            u=os.system('clear')
            print("PAS DE BALLE")
            radius = -1
            x = -1
            y = -1
            BALLE = False
    else:
        u=os.system('clear')
        print("PAS DE BALLE")
        radius = -1
        x = -1
        y = -1
        BALLE = False

    cv2.imshow("Frame", frame)
    #cv2.imshow("Mask", mask)

    if BALLE == True :
        if (x < 250) and (x !=-1) :
            #gauche
            #tournerg(holo, 1)
            holo.turn(5)
            holo.move_toward(30, 300)

        elif (x > 350) :
            #droite
            #tournerd(holo,1)
            holo.turn(-5)
            holo.move_toward(30, 300)

        elif (x ==-1) :
            holo.stop_all()
        else :
            holo.move_toward(30, 300)

    else : holo.stop_all()
        holo.turn(20)

    key = cv2.waitKey(1) & 0xFF
camera.release()
cv2.destroyAllWindows()
