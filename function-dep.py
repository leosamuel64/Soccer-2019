# -*- coding: utf-8 -*-
"""
Created on Fri Apr 19 11:19:27 2019

@author: L.Draescher (draescherl@eisti.eu)
"""
import math
import time
import sys
from holobot import Holobot



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
Postconditions -- déplace le robot
-----------------------------------------------------------------------------------"""
def tourner(robot, angle):
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
    

    
    
    
"""----------------------------------------------------------------------------------
---------------------------------Programme principal---------------------------------
----------------------------------------------------------------------------------"""
holo = Holobot(sys.argv[1], 115200)
initHolo(holo)
bouger(holo, 0, 45)


"""
TODO:
1) Convertir rayon en distance
2) Convertir distance au pt central en angle
3) Interfacer avec la détection openCV
4) Tests
"""
