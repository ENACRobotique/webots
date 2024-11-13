from enum import auto, Enum
import math
import os
import random
from controller import Supervisor

THETA = 0.01
RADIUS = 0.3

POSITIONS_1 = [(-1.0, 0.437, 2), (-1.12, 0.706, 1), (-0.761, 0.786, 2),
               (-0.13, 0.792, 1), (-0.764, 0.53, 1), (-0.848, 0.297, 3),
               (-0.713, 0.063, 3), (-0.603, -0.127, 2), (0.11, 0.23, 2),
               (0.8, 0.5, 2), (0.05, -0.07, 2)]

POSITIONS_2 = [(-1.0, 0.0238, 2), (-1.22, -0.257, 1), (-1.13, -0.646, 2),
               (-0.738, -0.904, 3), (-0.565, -0.666, 2), (-0.06, -0.47, 3),
               (-0.827, -0.412, 3), (0.82, -0.44, 2), (0.23, -0.42, 1)]


class States(Enum):
    ROTATE = auto()
    GO_TO = auto()


def is_destination_achieved(current_position, destination_position, epsilon=0.05):
    return abs(current_position[0] -
               destination_position[0]) < epsilon and abs(
                   current_position[1] - destination_position[1]) < epsilon


def wait_at_destination(supervisor, timestep, time_period):
    current_time = supervisor.getTime()
    while supervisor.getTime() - current_time < time_period:
        supervisor.step(timestep)


def is_rotation_achieved(current_rotation_angle, required_angle, epsilon=0.02):
    return abs(current_rotation_angle - required_angle) < epsilon


def set_angle(current_angle, required_angle, theta):
    if current_angle < required_angle:
        current_angle += theta
    else:
        current_angle -= theta
    return current_angle


def get_target_angle(supervisor, destination_x, destination_y):
    robot = supervisor.getSelf()
    field = robot.getField('translation')
    position = field.getSFVec3f()
    return math.atan2(destination_y - position[1], destination_x - position[0])


def are_colliding(translation, translation_memristor):
    position = translation.getSFVec3f()
    position_memristor = translation_memristor.getSFVec3f()

    dx = position_memristor[0] - position[0]
    dy = position_memristor[1] - position[1]

    return math.sqrt(dx ** 2 + dy ** 2) < RADIUS


def main():
    supervisor = Supervisor()

    timestep = int(supervisor.getBasicTimeStep())

    dummy_robot = supervisor.getSelf()
    dummy_translation = dummy_robot.getField('translation')
    dummy_rotation = dummy_robot.getField('rotation')
    robot = supervisor.getFromDef('ROBOT')
    robot_translation = robot.getField('translation')

    positions = POSITIONS_1

    destination = positions[random.randint(0, len(positions) - 1)]

    next_state = States.ROTATE

    while supervisor.step(timestep) != -1:
        pos = dummy_translation.getSFVec3f()
        rot = dummy_rotation.getSFRotation()

        if next_state == States.ROTATE:
            target_angle = get_target_angle(supervisor, destination[0], destination[1])
            
            if not is_rotation_achieved(rot[3], target_angle):
                rot[3] = set_angle(rot[3], target_angle, THETA)

            if not is_rotation_achieved(rot[3], target_angle):
                dummy_rotation.setSFRotation(rot)
            else:
                next_state = States.GO_TO

        if next_state == States.GO_TO:
            if is_destination_achieved(pos, destination):
                wait_at_destination(supervisor, timestep, destination[2])
                destination = random.choice(positions)
                next_state = States.ROTATE

            else:
                velocity_factor = random.uniform(0.0009, 0.002)

                delta_x = math.cos(target_angle) * velocity_factor
                delta_y = math.sin(target_angle) * velocity_factor
                

                if not are_colliding(dummy_translation, robot_translation):
                    pos[0] += delta_x
                    pos[1] += delta_y

                if pos[0] != destination[0] and pos[1] != destination[1]:
                    dummy_translation.setSFVec3f(pos)
                else:
                    pos[0] = destination[0]
                    pos[1] = destination[1]


if __name__ == '__main__':
    main()
