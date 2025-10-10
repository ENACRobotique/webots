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

MAX_SPEED = 20 # in rad/s

LIDAR_PREDIV = 1

THETA1 = pi/4
THETA2 = -pi/4.0
THETA3 = 3*pi/4.0
THETA4 = -3*pi/4
ROBOT_RADIUS = 121


D = np.array([[-sin(THETA1), cos(THETA1), ROBOT_RADIUS],
          [-sin(THETA2), cos(THETA2), ROBOT_RADIUS],
          [-sin(THETA3), cos(THETA3), ROBOT_RADIUS],
          [-sin(THETA4), cos(THETA4), ROBOT_RADIUS]])





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
        self.mot_front = self.getDevice("frontleft-omni-motor")
        self.mot_front.setPosition(float('inf'))
        self.mot_front.setVelocity(0.0)
        self.mot_right = self.getDevice("frontright-omni-motor")
        self.mot_right.setPosition(float('inf'))
        self.mot_right.setVelocity(0.0)
        self.mot_left = self.getDevice("backleft-omni-motor")
        self.mot_left.setPosition(float('inf'))
        self.mot_left.setVelocity(0.0)
        self.mot_back = self.getDevice("backright-omni-motor")
        self.mot_back.setPosition(float('inf'))
        self.mot_back.setVelocity(0.0)
    
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
    
    def set_speed(self, vx, vy, vtheta):
        m = D @ np.array([vx, vy, vtheta])
        self.mot_front.setVelocity(-m[0]*MAX_SPEED)
        self.mot_right.setVelocity(-m[1]*MAX_SPEED)
        self.mot_left.setVelocity(-m[2]*MAX_SPEED)
        self.mot_back.setVelocity(-m[3]*MAX_SPEED)
        #print(f"{vx:.2f}  {vy:.2f}  {omega:.2f}")
    
    def run(self):
        while self.step() != -1:
            self.send_position()

            if self.lidar_count % LIDAR_PREDIV == 0:
                self.send_lidar_data()
            self.lidar_count += 1


            vx = 0.0
            vy = 0.0
            omega = 0.0

            key = self.kbd.getKey()
            if key == ord('Z'):
                vx = 1.0
            elif key == ord('S'):
                vx = -1.0
            elif key == ord('Q'):
                vy = 1.0
            elif key == ord('D'):
                vy = -1.0
            elif key == ord('A'):
                omega = 1.0/ROBOT_RADIUS
            elif key == ord('E'):
                omega = -1.0/ROBOT_RADIUS
            
            self.set_speed(vx, vy, omega)

            

            self.camera.getImage()


if __name__ == "__main__":
    ecal_core.initialize([], "webots bridge")
    r = Robot()
    r.run()

