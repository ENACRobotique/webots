import math
import socket
import os
import time
from controller import Supervisor

supervisor = Supervisor()
joy = supervisor.getJoystick()
TIME_STEP = int(supervisor.getBasicTimeStep())

mot_frontleft = supervisor.getDevice("frontleft-omni-motor")
mot_frontright = supervisor.getDevice("frontright-omni-motor")
mot_backleft = supervisor.getDevice("backleft-omni-motor")
mot_backright = supervisor.getDevice("backright-omni-motor")
mot_frontleft.setPosition(float('inf'))
mot_frontright.setPosition(float('inf'))
mot_backleft.setPosition(float('inf'))
mot_backright.setPosition(float('inf'))
mot_frontleft.setVelocity(0.0)
mot_frontright.setVelocity(0.0)
mot_backleft.setVelocity(0.0)
mot_backright.setVelocity(0.0)

camera = supervisor.getDevice("camera")
camera.enable(TIME_STEP)

MAX_SPEED = 20 # in rad/s

def main():
    while supervisor.step(TIME_STEP) != -1:
        vx = 0.0
        vy = 0.0
        omega = 0.0

        if not joy.isConnected():
            joy.enable(10)
        else:
            a0 = joy.getAxisValue(0)
            a1 = joy.getAxisValue(1)
            a2 = joy.getAxisValue(2)
            a3 = joy.getAxisValue(3)
            a4 = joy.getAxisValue(4)
            a5 = joy.getAxisValue(5)
            # print(a0, a1, a2, a3, a4, a5)

            vx = -a1/32767
            vy = -a0/32767
            omega = -a3/32767

            # print(f"vx:{vx} vy:{vy} omega:{omega}")

        frontleft = vx - vy - omega
        frontright = -vx - vy - omega
        backleft = vx + vy - omega
        backright = -vx + vy - omega

        mot_frontleft.setVelocity(frontleft*MAX_SPEED)
        mot_frontright.setVelocity(frontright*MAX_SPEED)
        mot_backleft.setVelocity(backleft*MAX_SPEED)
        mot_backright.setVelocity(backright*MAX_SPEED)

        camera.getImage()

if __name__ == "__main__":
    main()
