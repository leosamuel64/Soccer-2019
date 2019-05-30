# -*- coding: utf-8 -   *-

""" Partie INDISPENSABLE"""

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
import matplotlib as plt

# Indispensable au demmarage.
GPIO.setwarnings(False)
holo = Holobot(sys.argv[1], 115200)
if len(sys.argv) != 2:
    print ("error: I need the serial port (bluetooth or wired)")
    sys.exit(0)

# Set up obligatoire des pins GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3, GPIO.OUT)
GPIO.setup(3, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(26, GPIO.IN)

# Calibrage du gyroscope
print("Calibrage en cours...")
holo.calibrate_magneto()
time.sleep(5)
print("Calibrage terminé")



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
def init(robot):
    robot.reset_yaw()




"""-----------------------------------------------------------------------------------
Préconditions  -- robot    : la variable du robot
                  distance : type entier ; la distance souhaitée pour le déplacement en cm
                  angle    : type entier ; l'angle voulu en degrés
Postconditions -- déplace le robot
-----------------------------------------------------------------------------------"""
def avancer(robot, distance, angle, vitesse):
    #Constantes :
    dt = 0.1
    t = 0.0

    #Calculs :
    distance = distance*10
    temps = distance/vitesse
    angle = angle - 45

    #Bouger :
    time.sleep(dt)
    while(t < temps):
        robot.move_toward(vitesse, angle)
        time.sleep(dt)
        t += dt



"""-----------------------------------------------------------------------------------------------
Préconditions   -- robot     : la variable du robot


Postconditions  -- renvoie la distance du robot a un obstacle
-----------------------------------------------------------------------------------------------"""
def dist(robot):
    distance = robot.get_dist()
    return distance



"""-----------------------------------------------------------------------------------------------
Préconditions   -- duree    : temps durant lequel le robot continue de frapper
                   cadence  : Nombre de coups par seconde


Postconditions  -- Ordonne au robot de frapper
-----------------------------------------------------------------------------------------------"""

def frapper(duree, cadence):
    t = 0
    while t < duree :
        GPIO.output(3, not GPIO.input(3))
        pause = 1/cadence
        time.sleep(pause)
        t += pause

"""--------------------------------------------------------------------------------------------
Préconditions  -- robot     : la variable du robot

Postconditions -- Renvoie une information sur le robot
-----------------------------------------------------------------------------------------------"""
def faux(robot):
    robot.beep(150, 200)
    robot.set_leds(255,0,0)
    time.sleep(1)
    robot.set_leds(0,0,0)

def juste(robot):
    robot.beep(880, 200)
    robot.set_leds(0,255,0)
    time.sleep(1)
    robot.set_leds(0,0,0)

"""--------------------------------------------------------------------------------------------
Préconditions  -- vitesse     : la vitesse de rotation (toujours positive) en degrés par seconde
                  angle       : l'angle en degré à atteindre

Postconditions -- Ordonne au robot de pivoter de "angle" degrés
-----------------------------------------------------------------------------------------------"""
def tourner(vitesse, angleCible) :

    holo.reset_yaw()


    if (abs(angleCible) < 90) :
        correction = 6
    elif abs(angleCible) < 180 :
        correction = 10
    elif angleCible < 270 :
        correction = 6
    elif angleCible < 360 :
        correction = 10
    if angleCible == 180:
        angleCible += 1

    if angleCible < 0 :
        angleCible += 360

    angle = holo.get_yaw()

    while(angle > (angleCible+correction) or angle < (angleCible-correction)) :
        angle = holo.get_yaw()

        if angle < 0 :
            angle += 360


        if angleCible < 180 :
            holo.turn(vitesse)

        elif angleCible > 180 :
            holo.turn(-vitesse)


    holo.stop_all()


"""--------------------------------- Début du code Principal ----------------------------------"""
juste(holo)
