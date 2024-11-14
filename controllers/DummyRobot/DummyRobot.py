import enum
import math
import random
from controller import Supervisor

# a random speed is chosen in this interval for lines
MIN_VELOCITY = 0.002
MAX_VELOCITY = 0.004

# theta incremented each simulation step when rotating
THETA = 0.02

# collision radius of the dummy robot
RADIUS = 0.3

# positions (x, y, wait_seconds)
POSITIONS = [
    (-1.2, 0.5, 1), # backstage
    (-0.7, 0.5, 1), # backstage stack
    (-1.3, 0.3, 1), # stack
    (-0.4, 0.0, 1), # stack center
    (0.4, 0.0, 1), # stack center
    (-0.8, -0.8, 1), # small zone bottom center
    (-0.2, -0.6, 1), # zone bottom center
    (1.3, -0.8, 1), # small zone bottom right
    (1.3, -0.1, 1) # zone center right
]

class States(enum.Enum):
    ROTATE = 0
    GO_TO = 1

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

    robot = supervisor.getFromDef("OPPONENT")

    if not robot:
        print("DummyRobot didn't found an opponent robot, collisions disabled")
    else:
        print("DummyRobot found an opponent robot!, will stop on collision")

    destination = random.choice(POSITIONS)

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
                destination = random.choice(POSITIONS)
                next_state = States.ROTATE

            else:
                velocity_factor = random.uniform(MIN_VELOCITY, MAX_VELOCITY)

                delta_x = math.cos(target_angle) * velocity_factor
                delta_y = math.sin(target_angle) * velocity_factor

                if not robot or not are_colliding(dummy_translation, robot.getField('translation')):
                    pos[0] += delta_x
                    pos[1] += delta_y

                if pos[0] != destination[0] and pos[1] != destination[1]:
                    dummy_translation.setSFVec3f(pos)
                else:
                    pos[0] = destination[0]
                    pos[1] = destination[1]

if __name__ == '__main__':
    main()
