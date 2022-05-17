"""
Created on Wed Jun  2 20:18:56 2021
"""

import numpy as np
import math as m
import sys
import time
import sim as vrep # access all the VREP elements
import scipy.interpolate as spi
import matplotlib.pyplot as plt


def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)


def sigPunto(angulo,carpos):
    flg = False
    
    if carrot[2] < m.pi and carrot[2] > 8*m.pi/9:
        flg = True
    elif carrot[2] > -m.pi and carrot[2] < -8*m.pi/9:
        flg = True
        
        
    if carrot[2] < 11*m.pi/18 and carrot[2] > 7*m.pi/18:
        xp= carpos[0]
        yp=carpos[1]+.5
    elif carrot[2] > -11*m.pi/18 and carrot[2] < -7*m.pi/18:
        xp= carpos[0]
        yp=carpos[1]-.5
    elif flg == True:
        xp= carpos[0]-.5
        yp=carpos[1]
    elif carrot[2] < m.pi/9 and carrot[2] > -m.pi/9:
        xp= carpos[0]+.5
        yp=carpos[1]
    else: 
        xp=carpos[0]
        yp=carpos[1]
    return xp, yp


if __name__ == "__main__":
    
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print('Not connected to remote API server')
        sys.exit("No connection")

    

    # Getting handles for the motors and robot
    err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
    err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
    err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

    

    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

    

    usensor = []
    for i in range(1,17): #se inicializan los sensores del 3-6 que son los necesarios para evitar colisiones 
        err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
        usensor.append(s)

    

    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)


    # Controller gains (linear and heading)
    Kv = 0.5
    Kh = 2.5

    errp=0
    r = 0.5*0.195
    L = 0.311
    i = 0
    t = time.time()
    trot = 1.35 #tiempo aproximado en que el robot gira 90°
    giro = False
    #delay1 = 1
    #delay2 = 10
    #delay = delay1
    #tdes = 7.6125/3


    while (time.time() - t) < 0.1: #inicializa el robot con velocidad 0 los primeros 0.1 segundos
        errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
        
    while (time.time() - t ) < 600: #se formulo el ciclo while para que este se realice hasta que el robot termine la trayectoria, es decir cruce los 600 puntos en un tiempo de 185 segundos
        sstate = []
        ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)
        ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
        for k in range(16):
            err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[k], vrep.simx_opmode_streaming)
            sstate.append(state)
        
        
        if giro == False:
            t1 = time.time()
            if sstate[7] == False and sstate[8] == False:
                print("Girar derecha")
                time.sleep(0.5)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
                time.sleep(1)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 2, vrep.simx_opmode_streaming)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorR, -2, vrep.simx_opmode_streaming)
                time.sleep(trot)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
                time.sleep(1)
                giro = True
                ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
                ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)
                xp, yp = sigPunto(carrot[2], carpos)
                errp = m.sqrt((xp-carpos[0])**2 + (yp-carpos[1])**2)
                t1 = time.time()
                    
                
            elif sstate[3] == False or sstate[4] == False or sstate[2] == False or sstate[5] == False or sstate[6] == False or sstate[7] == False:
                print("Avanzar")
                    
            elif sstate[0] == False and sstate[15] == False:
                print("Girar izquierda")
                time.sleep(0.5)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
                time.sleep(1)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorL, -2, vrep.simx_opmode_streaming)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 2, vrep.simx_opmode_streaming)
                time.sleep(trot)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
                errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
                time.sleep(1)
                ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)
                ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
                xp, yp = sigPunto(carrot[2], carpos)
                giro = True
            
            else:
                flg = True
                for w in range (9):
                    if sstate[w] == False:
                        flg = False
                if sstate[15] == True and flg == True:
                    print("Girar 180°")
                    time.sleep(0.5)

                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
                    time.sleep(1)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 2, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, -2, vrep.simx_opmode_streaming)
                    time.sleep(trot*2)
                    rrf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
                    time.sleep(1)
                    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)
                    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
                    xp, yp = sigPunto(carrot[2], carpos)
                    giro = True
                        

        #CALCULAR LA SIGUIENTE POSICIO
        xp, yp = sigPunto(carrot[2], carpos)
            
        #CALCULAR ERROR DE POSICION
        errp = m.sqrt((xp-carpos[0])**2 + (yp-carpos[1])**2)     
        angd = m.atan2(yp-carpos[1], xp-carpos[0])
        errh = angdiff(carrot[2], angd)
        

        #AVANZAR
        v = Kv*errp
        omega = Kh*errh
        ul = v/r - L*omega/(2*r)
        ur = v/r + L*omega/(2*r)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
        if time.time() - t1 > 3.7:
            giro = False
        #time.sleep(delay)
        print(giro)   
        

    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)