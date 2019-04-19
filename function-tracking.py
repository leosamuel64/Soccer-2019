import cv2

########---VARIABLES---########

    #   x : position du centre en x
    #   y : position du centre en y
    #   radius : rayon de la balle
    #   pos : position relative

        # Si x, y, radius, pos = -1 la balle est hors cadre

########---------------########

def cam():
    greenLower = (5, 140, 185)
    greenUpper = (30, 215, 255)
    camera = cv2.VideoCapture(0)
    for i in range(0,2):
        (grabbed, frame) = camera.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0 and i ==1:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            x=int(x)
            y=int(y)
            if radius > 10:
                pos = (300-int(x))*-1    # position par rapport au centre de l'image x=300
            else:
                radius = -1
                x = -1
                y = -1  
                pos = -1
        elif i ==1 :
            radius = -1
            x = -1
            y = -1
            pos = -1
    camera.release()

########---OUTPUTS---########

    print("x = "+str(x))
    print("y = "+str(y))
    print("radius = "+str(radius))
    print("Pos = "+str(pos))

########---------------########
