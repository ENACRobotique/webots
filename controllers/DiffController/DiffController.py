from controller import Supervisor
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import sys
from math import pi, cos, sin
import numpy as np
import cv2


sys.path.append("../..")

from generated import robot_state_pb2 as rpb
from generated import lidar_data_pb2 as lpb
from generated import common_pb2 as cpb
from generated import CompressedImage_pb2 as cipb
from google.protobuf.timestamp_pb2 import Timestamp

LIDAR_PREDIV = 3
CAM_PREDIV = 8

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
        self.cam_count = 0
        self.speed_cons_sub = ProtoSubscriber("speed_cons", cpb.Speed)
        self.speed_cons_sub.set_callback(self.receive_speed_cons)
        self.cam_pub = ProtoPublisher("images", cipb.CompressedImage)

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
    
    def receive_speed_cons(self, topic_name, hlm, time):
        self.set_speed(hlm.vx, hlm.vy, hlm.vtheta)
    
    def send_camera_img(self):
        img = np.frombuffer(self.camera.getImage(), dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
        img_encode = cv2.imencode(".jpg", img)[1]
        ci = cipb.CompressedImage(timestamp=Timestamp(), data=img_encode.tobytes(), format='jpeg')
        self.cam_pub.send(ci)

    def run(self):
        while self.step() != -1:
            self.send_position()

            if self.lidar_count % LIDAR_PREDIV == 0:
                self.send_lidar_data()
            self.lidar_count += 1

            if self.cam_count % CAM_PREDIV == 0:
                self.send_camera_img()
            self.cam_count += 1

            


            key = self.kbd.getKey()
            if key == ord('G'):
                self.servo.setPosition(1.7)
            elif key == ord('R'):
                self.servo.setPosition(0)
            elif key == ord('P'):
                self.pompe.turnOn()
            elif key == ord('O'):
                self.pompe.turnOff()
                



if __name__ == "__main__":
    ecal_core.initialize([], "webots bridge")
    r = Robot()
    r.run()

