from controller import Supervisor
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import sys
from math import pi, cos, sin
import numpy as np


sys.path.append("../..")

from generated import robot_state_pb2 as rpb
from generated import lidar_data_pb2 as lpb

LIDAR_PREDIV = 3

ROBOT_RADIUS = 121

WHEEL_RADIUS = 33   # 66mm diameter

class Robot(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        TIME_STEP = int(self.getBasicTimeStep())
        self.kbd = self.getKeyboard()
        self.kbd.enable(TIME_STEP)
        self.init_motors()
        self.camera = self.getDevice("camera")
        self.camera.enable(TIME_STEP)
        self.pos_pub = ProtoPublisher("odom_pos", rpb.Position)
        self.lidar_pub = ProtoPublisher("lidar_data", lpb.Lidar)
        self.lidar = self.getDevice("LD06")
        self.lidar.enable(TIME_STEP)
        gyro = self.getDevice("gyro")
        gyro.enable(TIME_STEP)
        self.lidar_count = 0

    def init_motors(self):
        self.mot_left = self.getDevice("left-wheel-motor")
        self.mot_left.setPosition(float('inf'))
        self.mot_left.setVelocity(0.0)
        self.mot_right = self.getDevice("right-wheel-motor")
        self.mot_right.setPosition(float('inf'))
        self.mot_right.setVelocity(0.0)
        self.servo = self.getDevice("servo")
        self.pompe = self.getDevice("vacuum-gripper")
    
    def send_lidar_data(self):
        dist_quality = [(1000*d, 200) if d!=float('inf') else (12500, 10) for d in self.lidar.getRangeImage()]
        distances, quality = list(zip(*dist_quality))
        angles = [2*pi-2*pi*i/len(distances)+pi for i in range(len(distances))]
        lidar_msg = lpb.Lidar(angles=angles, distances=distances, quality=quality)
        self.lidar_pub.send(lidar_msg)
    
    def send_position(self):
        robot = self.getSelf()
        robot_pos = robot.getField('translation')
        robot_rot = robot.getField('rotation')
        pos = robot_pos.getSFVec3f()
        rot = robot_rot.getSFRotation()
        x = (pos[0]+1.5)*1000
        y = (pos[1]+1)*1000
        theta = -rot[3] if rot[2] < 0 else rot[3]
        pos_msg = rpb.Position(x=x,y=y,theta=theta)
        self.pos_pub.send(pos_msg)
    
    def set_speed(self, vx, _vy, vtheta):
        v_left =  (vx - ROBOT_RADIUS*vtheta) / WHEEL_RADIUS
        v_right = (vx + ROBOT_RADIUS*vtheta) / WHEEL_RADIUS
        self.mot_left.setVelocity(v_left)
        self.mot_right.setVelocity(v_right)
        #print(f"{vx:.2f}  {vy:.2f}  {omega:.2f}")
    
    def run(self):
        while self.step() != -1:
            self.send_position()

            if self.lidar_count % LIDAR_PREDIV == 0:
                self.send_lidar_data()
            self.lidar_count += 1


            vx = 0.0
            omega = 0.0

            key = self.kbd.getKey()
            if key == ord('Z'):
                vx = 300
            elif key == ord('S'):
                vx = -300
            elif key == ord('Q'):
                omega = 2.0
            elif key == ord('D'):
                omega = -2.0
            elif key == ord('G'):
                self.servo.setPosition(1.7)
            elif key == ord('R'):
                self.servo.setPosition(0)
            elif key == ord('P'):
                self.pompe.turnOn()
            elif key == ord('O'):
                self.pompe.turnOff()
                
            
            self.set_speed(vx, 0, omega)

            

            self.camera.getImage()


if __name__ == "__main__":
    ecal_core.initialize([], "webots bridge")
    r = Robot()
    r.run()

