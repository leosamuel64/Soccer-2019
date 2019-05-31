# -*- coding: utf-8 -   *-

angleDecalage = holo.get_yaw()

if (angleDecalage <= 0) :
    angleTir = ((180 - angleDecalage)/2)

elif (angleDecalage >= 0) :
    angleTir = ((180 + angleDecalage)/2)

#robot qui va se déplacer selon un cercle
def cercle(angleTir) :
    t = 0.0
    dt = 0.050
    while t < 4.0 :
        x = math.sin(2*math.pi * (t/4) - math.pi/2)
        y = math.sin(2*math.pi * (t/4))
        holo.control(50*x, 50*y, angleTir)
        t += dt
        time.sleep(dt)
    holo.stop_all()

cercle(angleTir)
