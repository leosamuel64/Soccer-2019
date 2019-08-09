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

#TODO : retourner quel(s) capteur(s) detecte(nt) opts
#TODO : optimiser le deplacement 
#TODO  : definir les limites pour les couleurs buts

"""----------------------------------------------------------------------------------- 
Postconditions -- initialise les systemes du robot et le robot
-----------------------------------------------------------------------------------"""

def init():

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(3, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(8, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    global holo = Holobot(sys.argv[1], 115200)
    global camera = cv2.VideoCapture(0)
    print("Camera ON")
    holo.reset_yaw()
    print("Calibrage Magneto")
    holo.calibrate_magneto()
    time.sleep(5)
    print("Fin Setup")
    u=os.system('clear')

"""-----------------------------------------------------------------------------------
Préconditions  -- distancemin : distance entre le capteur IR et la balle pour frapper 
                    
Postconditions -- Active de mode de debug
-----------------------------------------------------------------------------------"""

def debug(dimimage):
    holo.reset_yaw()

    While True:
        
            #suiveur de ligne
        
        line = " | "
        for i in range (0,7):
            line = line + str(holo.get_opt(i)) + " | "
        print("Suiveur de ligne : ")
        print(line)

            #Magneto
        
        rotation = holo.get_yaw()
        print("Gyroscope : ")
        print(rotation)

            #distance IR
        distance = holo.get_dist()
        print("Capteur IR : ")
        print(distance)

            #Ball detection

        BALLE,x,y,radius,FPS = checkball(dimimage)
            print("Balle : "+str(BALLE))
            print("x : "+str(x))
            print("y : "+str(y))
            print("radius : "+str(radius))
            print("FPS : "+str(FPS))


            #But detection

        BUT,x,y,radius,FPS = checkbut("jaune")
            print("But jaune : "+str(BALLE))
            print("x : "+str(x))
            print("y : "+str(y))
            print("radius : "+str(radius))
            print("FPS : "+str(FPS))

        BUT,x,y,radius,FPS = checkbut("bleu")
            print("But jaune : "+str(BALLE))
            print("x : "+str(x))
            print("y : "+str(y))
            print("radius : "+str(radius))
            print("FPS : "+str(FPS))    

            # Wheels speed

        wheel = " | "
        for j in range (0,3):
            wheel = wheel + str(holo.get_wheel_speeds()) + " | "
        print(wheel)

            # Test kicker

        tpsfrapper = 1
        GPIO.output(3,GPIO.LOW)
        GPIO.output(8,GPIO.HIGH)
        time.sleep(tpsfrapper)
        GPIO.output(3,GPIO.HIGH)
        GPIO.output(8,GPIO.LOW)
        time.sleep(tpsfrapper)
        GPIO.output(3,GPIO.HIGH)
        GPIO.output(8,GPIO.HIGH)

        time.sleep(0.1)
        u=os.system('clear')

"""-----------------------------------------------------------------------------------
Préconditions  -- distancemin : distance entre le capteur IR et la balle pour frapper 
                    
Postconditions -- Le robot frappe
-----------------------------------------------------------------------------------"""

def frapper(distancemin):
    distance = holo.get_dist()
    if distance <= distancemin:
        tpsfrapper = 1
        GPIO.output(3,GPIO.LOW)
        GPIO.output(8,GPIO.HIGH)
        time.sleep(tpsfrapper)
        GPIO.output(3,GPIO.HIGH)
        GPIO.output(8,GPIO.LOW)
        time.sleep(tpsfrapper)
        GPIO.output(3,GPIO.HIGH)
        GPIO.output(8,GPIO.HIGH)

## Definition des variables

greenLower = (0, 0, 200)
greenUpper = (80, 90, 255)
posballe = 2

"""-----------------------------------------------------------------------------------
Preconditions  -- dimimage : dimension de l'image apres reduction 

Postconditions -- BALLE : Presence de la balle
                  x : Position "x" de la balle sur l'image (en pixel)
                  y : Position "y" de la balle sur l'image (en pixel)
                  radius : Rayon de la balle sur l'image (en pixel)
                  FPS : Nombre d'images par seconde
-----------------------------------------------------------------------------------"""

def checkball(dimimage): #250 pour Sydney

    timedebut = time.time()

    (grabbed, frame) = camera.read()
    frame = imutils.resize(frame, width=dimimage)
    mask = cv2.inRange(frame, greenLower, greenUpper)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 3:
            BALLE = True
            pos = (300-int(x))*-1    
            holo.beep(150,100)            

        else:
            BALLE = False
            radius = -1
            x = -1
            y = -1
    
    else:
        BALLE = False
        radius = -1
        x = -1
        y = -1
        
    FPS = (time.time() - timedebut)**-1  
    
    return (BALLE,x,y,radius,FPS)

"""-----------------------------------------------------------------------------------
Preconditions  -- x : position sur l'image en x (en pixel)
                  y : position sur l'image en y (en pixel)
                  dimimage : dimension max en x de l'image

Postconditions -- renvoit la position par rapport au centre (x,y)
-----------------------------------------------------------------------------------"""

def calculmvt(x,y,dimimage):
    
    xcentre = dimimage/2
    ycentre = ((dimimage*9)/16)/2

    x = x - xcentre
    y = y - ycentre

    return (x,y)

"""-----------------------------------------------------------------------------------
Preconditions  -- mode  - 0 : detection simple
                        - 1 : detection et deplacement arriere
  
Postconditions -- mode : - 0 : renvoit si il y a une ligne ou non (boolean)
                         - 1 : renvoit si il y a une ligne ou non (boolean) et recule si il y en a une
-----------------------------------------------------------------------------------"""

#TODO : retourner quel(s) capteur(s) detecte(nt) opts

def linecheck(mode):
    for i in range (0,7):
        line = holo.get_opt(i)
        
        if (line < 0.40):
            linecheck = True
        else:
            linecheck = False
    if ((linecheck == True) and (mode == 1)):
        holo.move_toward(255,120)
        time.sleep(0.3)
    return linecheck

"""-----------------------------------------------------------------------------------
Preconditions  -- mode  - 0 : detection simple
                        - 1 : detection et deplacement arriere
  
Postconditions -- mode : - 0 : renvoit si il y a une ligne ou non (boolean)
                         - 1 : renvoit si il y a une ligne ou non (boolean) et recule si il y en a une
-----------------------------------------------------------------------------------"""    
# TODO : optimiser le deplacement 
def linecheck():
    # zone 1
    if ((y>0) and (x<0)) : 
        holo.control(0,-speed,0)

    #zone2
    elif ((y>0) and (x>0)) : 
        holo.control(0,speed,0)
    #zone3
    else :
        holo.control(-speed,0,0)

"""-----------------------------------------------------------------------------------
Postconditions --  couleur : couleur du but selection
-----------------------------------------------------------------------------------"""
# avec un levier
def selectbut():
    if GPIO.input(10):
        couleur = "jaune"
    else:
        couleur = "bleu" 

    return couleur
  
"""-----------------------------------------------------------------------------------
Preconditions  -- couleur : couleur du but : "jaune" ou "bleu"

Postconditions -- angle : angle du but
-----------------------------------------------------------------------------------"""

def checkbut(couleur):
    #TODO  : definir les limites pour les couleurs buts

    if (couleur == "jaune"):
        greenLower = (0, 0, 200)
        greenUpper = (80, 90, 255)
    elif (couleur == "bleu"):
        greenLower = (0, 0, 200)
        greenUpper = (80, 90, 255)

    timedebut = time.time()

    (grabbed, frame) = camera.read()
    frame = imutils.resize(frame, width=dimimage)
    mask = cv2.inRange(frame, greenLower, greenUpper)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 3:
            BUT = True
            pos = (300-int(x))*-1    
            holo.beep(150,100)            

        else:
            BUT = False
            radius = -1
            x = -1
            y = -1
    
    else:
        BUT = False
        radius = -1
        x = -1
        y = -1
        
    FPS = (time.time() - timedebut)**-1  
    
    return (BUT,x,y,radius,FPS)

"""-----------------------------------------------------------------------------------
Preconditions  -- angle : angle du but
  
Postconditions -- vise le but
-----------------------------------------------------------------------------------"""
def motionbut(angle):
    holo.calibrate_magneto()
    rotation = 0
    if angle < 0:
        while abs(rotation) <abs(angle):
            holo.turn(15)
            rotation = holo.get_yaw()
    else:
        while abs(rotation) <abs(angle):
            holo.turn(15)
            rotation = holo.get_yaw()

"""-----------------------------------------------------------------------------------
Préconditions  -- distance : distance de capture
  
Postconditions -- capture : informe sur la possesion de la balle
-----------------------------------------------------------------------------------"""

def capture(distance):
    distancecapteur = holo.get_dist()
    if distancecapteur < distance:
        capture = True
    else:
        capture = False
    return capture


               
           


### CODE PRINCIPAL

init()

























































    
while True:

    frapper()
    
    BALLE = False
    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=250)
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


        if radius > 3:

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            u=os.system('clear')
            print("x = "+str(int(x)))
            print("y = "+str(int(y)))
            #print("Radius  = "+str(radius))
            pos = (300-int(x))*-1    
            #print("Pos = "+str(pos))
            BALLE = True
            holo.beep(150,100)

        else:
            u=os.system('clear')
            #print("PAS DE BALLE")
            radius = -1
            x = -1
            y = -1
            holo.stop_all()
            #faux(holo)
            
            
    else:
        #u=os.system('clear')
        #print("PAS DE BALLE")
        radius = -1
        x = -1
        y = -1
        holo.stop_all()
        #faux(holo)
        
        
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    linecheck = 0
    for i in range (0,7):
        line = holo.get_opt(i)
        
        if (line < 0.40):
            linecheck = 1
            
      
            
    if (linecheck == 0):
        #linecheck tech
        if BALLE == True :
            
           
            if (x <= 70) and (x !=-1) :
                #gauche
                #tournerg(holo, 1)
                holo.turn(15)
                #holo.move_toward(30, 300)
                #time.sleep(0.1)
                holo.stop_all()
                posballe = 0

            elif (x >= 180) :
                #droite
                #tournerd(holo,1)
                holo.turn(-15)
                #holo.move_toward(30, 300)
                #time.sleep(0.1)
                holo.stop_all()
                posballe = 1
                
            elif (y < 160):
                holo.control(300,0,0)
                holo.beep(880,100)
                #time.sleep(0.1)
                
        else :
            holo.stop_all()
            if (posballe==0):
                holo.turn(35)
            else:
                holo.turn(-35)
    else:#Ligne donc go en arriere 
        holo.move_toward(255,120)
        time.sleep(0.3)
            
    key = cv2.waitKey(1) & 0xFF
camera.release()
cv2.destroyAllWindows()
                                        
