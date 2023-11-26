import time
import math
from enum import Enum
import numpy as np

from api import sim, connect


class AngularSpeed(Enum):
    VERY_SLOW = 10  # Delicate handling, precision tasks
    SLOW = 20       # Slower, controlled movements
    MODERATE = 30   # Balanced speed for general tasks
    FAST = 40       # Faster movements
    VERY_FAST = 50  # Maximum controlled speed without load


class BotSpeed(Enum):
    STOP = 0
    SLOW = 1
    MODERATE = 5
    FAST = 10
    VERY_FAST = 20


class Joint:
    def __init__(self, client, name):
        self.name = name
        self.client_id = client
        self.handle = self.__set_handle(name)

    def __set_handle(self, joint_name):
        error, handle = sim.simxGetObjectHandle(self.client_id, joint_name, sim.simx_opmode_oneshot_wait)
        if not error:
            return handle
        else:
            raise ConnectionError(error)


class Wheel(Joint):
    def move(self, speed):
        sim.simxSetJointTargetVelocity(self.client_id, self.handle, speed, sim.simx_opmode_oneshot_wait)


class Revolute(Joint):
    def __init__(self, client, name, min_range, max_range, starting_position):
        super().__init__(client, name)
        self.range = (min_range, max_range)
        self.current_position = starting_position

    def move(self, angle):
        self.__rotate(angle)

    def __rotate(self, angle):
        if angle > self.range[1]:
            angle = self.range[1]
        elif angle < self.range[0]:
            angle = self.range[0]
        try:
            sim.simxSetJointTargetPosition(self.client_id, self.handle, np.deg2rad(angle), sim.simx_opmode_oneshot)
            self.current_position = angle
        except Exception as e:
            return str(e)


class Prismatic(Joint):
    def __init__(self, client, name, type, min_range:float=0, max_range:float=None, starting_position:float=0):
        super().__init__(client, name)
        self.range = (min_range, max_range)
        self.current_position = starting_position
        self.move_type = type

    def move(self, movement):
        if self.move_type == "position":
            self.__extend(movement)

        elif self.move_type == "velocity":
            self.__move_to(movement)

    def __move_to(self, speed):
        sim.simxSetJointTargetVelocity(self.client_id, self.handle, speed, sim.simx_opmode_oneshot_wait)
        # sim.simxSetJointTargetVelocity(self.client_id, self.handle, speed, sim.simx_opmode_streaming)

    def __extend(self, distance):
        if distance < self.range[0]:
            distance = self.range[0]
        elif distance > self.range[1]:
            distance = self.range[1]
        else:
            # code = sim.simxSetJointTargetPosition(self.client_id, self.handle, distance, sim.simx_opmode_oneshot)
            # code = sim.simxSetJointPosition(self.client_id, self.handle, distance, sim.simx_opmode_oneshot)
            # code = sim.simxSetJointPosition(self.client_id, self.handle, distance, sim.simx_opmode_streaming)
            code = sim.simxSetJointPosition(self.client_id, self.handle, distance, sim.simx_opmode_oneshot_wait)
            print(code)
            self.current_position = distance


class Grip:
    def __init__(self, client, prismatic0, prismatic1):
        self.client_id = client
        self.grip0 = prismatic0
        self.grip1 = prismatic1
        self.max_grip = math.fabs(self.grip1.range[1] - self.grip1.range[0])

    def grip(self, target_distance):
        # grip0:
        # option1 = sim.simxSetJointTargetPosition(self.client_id, self.grip0.handle, 0, sim.simx_opmode_oneshot)
        # option2 = sim.simxSetJointTargetPosition(self.client_id, self.grip0.handle, 0, sim.simx_opmode_streaming)
        # option3 = sim.simxSetJointTargetPosition(self.client_id, self.grip0.handle, 0, sim.simx_opmode_oneshot_wait)
        #
        # option4 = sim.simxSetJointPosition(self.client_id, self.grip0.handle, 0, sim.simx_opmode_oneshot)
        # option5 = sim.simxSetJointPosition(self.client_id, self.grip0.handle, 0, sim.simx_opmode_streaming)
        # option6 = sim.simxSetJointPosition(self.client_id, self.grip0.handle, 0, sim.simx_opmode_oneshot_wait)
        #
        # option7 = sim.simxSetJointTargetVelocity(self.client_id, self.grip0.handle, 0.5, sim.simx_opmode_oneshot)
        # option8 = sim.simxSetJointTargetVelocity(self.client_id, self.grip0.handle, 0.5, sim.simx_opmode_streaming)
        # option9 = sim.simxSetJointTargetVelocity(self.client_id, self.grip0.handle, 0.5, sim.simx_opmode_oneshot_wait)
        #
        # # grip1:
        # option1 = sim.simxSetJointTargetPosition(self.client_id, self.grip1.handle, 0, sim.simx_opmode_oneshot)
        # option2 = sim.simxSetJointTargetPosition(self.client_id, self.grip1.handle, 0, sim.simx_opmode_streaming)
        # option3 = sim.simxSetJointTargetPosition(self.client_id, self.grip1.handle, 0, sim.simx_opmode_oneshot_wait)
        #
        # option4 = sim.simxSetJointPosition(self.client_id, self.grip1.handle, 0, sim.simx_opmode_oneshot)
        option5 = sim.simxSetJointPosition(self.client_id, self.grip1.handle, 0, sim.simx_opmode_streaming)
        # option6 = sim.simxSetJointPosition(self.client_id, self.grip1.handle, 0, sim.simx_opmode_oneshot_wait)
        #
        # option7 = sim.simxSetJointTargetVelocity(self.client_id, self.grip1.handle, 0.5, sim.simx_opmode_oneshot_wait)
        # option8 = sim.simxSetJointTargetVelocity(self.client_id, self.grip1.handle, 0.5, sim.simx_opmode_oneshot_wait)
        # option9 = sim.simxSetJointTargetVelocity(self.client_id, self.grip1.handle, 0.5, sim.simx_opmode_oneshot_wait)

        # print(code)
        # print("grip called with", target_distance)
        # target_distance = target_distance if target_distance <= self.max_grip else self.max_grip
        # try:
        #     self.grip0.move(self.grip0.range[0] + self.max_grip - target_distance)
        # except Exception:
        #     print(0)
        # try:
        #     self.grip1.move(self.grip1.range[0] + self.max_grip - target_distance)
        #
        # except Exception:
        #     print(1)


class youBotArm:
    def __init__(self, client):
        grip_joint0 = Prismatic(client, "/youBot/youBotGripperJoint1", "position", 0, 0.025, starting_position=0.025)
        grip_joint1 = Prismatic(client, "/youBot/youBotGripperJoint2", "velocity", -0.05, 0, starting_position=-0.05)
        self.joints = [
            Revolute(client, "/youBot/youBotArmJoint0", min_range=-169, max_range=169, starting_position=0),
            Revolute(client, "/youBot/youBotArmJoint1", min_range=-90, max_range=-90+165, starting_position=30.91),
            Revolute(client, "/youBot/youBotArmJoint2", min_range=-131, max_range=-131+262, starting_position=52.42),
            Revolute(client, "/youBot/youBotArmJoint3", min_range=-102, max_range=-102+204, starting_position=72.68),
            Revolute(client, "/youBot/youBotArmJoint4", min_range=-90, max_range=90, starting_position=-0.006),
            Grip(client, grip_joint0, grip_joint1)
        ]

    def move(self, joint, description):
        self.joints[joint].move(description)


class MovingRobot:
    def __init__(self, name, port) -> None:
        self.name = name
        self.client_id = connect.connect_to_port(port)
        self.speed = BotSpeed.STOP.value


class YouBot(MovingRobot):
    def __init__(self, port=19999):
        super().__init__("youBot", port)
        self.WHEEL_RADIUS = 0.05
        self.WHEEL_DISTANCE = 0.30046
        self.front_left_wheel = Wheel(self.client_id, f"/{self.name}/rollingJoint_fl")
        self.back_left_wheel = Wheel(self.client_id, f"/{self.name}/rollingJoint_rl")
        self.front_right_wheel = Wheel(self.client_id, f"/{self.name}/rollingJoint_fr")
        self.back_right_wheel = Wheel(self.client_id, f"/{self.name}/rollingJoint_rr")
        self.arm = Joint(self.client_id, f"/{self.name}/{self.name}ArmJoint0")
        self.front_axis = [self.front_left_wheel, self.front_right_wheel]
        self.back_axis = [self.back_left_wheel, self.back_right_wheel]
        self.left_wheels = [self.back_left_wheel, self.front_left_wheel]
        self.right_wheels = [self.back_right_wheel, self.front_right_wheel]
        self.all_wheels = self.front_axis + self.back_axis
        self.arm = youBotArm(self.client_id)

    def set_wheel_velocities(self, left_wheel_velocity, right_wheel_velocity, duration):
        for wheel in self.left_wheels:
            wheel.move(left_wheel_velocity)

        for wheel in self.right_wheels:
            wheel.move(right_wheel_velocity)

        duration = duration if duration > 0 else 0
        time.sleep(duration)
        self.stop()

    # def set_rotation_in_place(self, angle_degrees, angular_speed_deg_s):
    #     # Convert degrees to radians
    #     angle_rad = math.radians(angle_degrees)
    #     angular_speed_rad_s = math.radians(angular_speed_deg_s)
    #
    #     # Calculate wheel velocities
    #     v = (self.WHEEL_DISTANCE / 2) * angular_speed_rad_s
    #     left_wheel_velocity = v / self.WHEEL_RADIUS
    #     right_wheel_velocity = -v / self.WHEEL_RADIUS
    #
    #     # Set velocities for a duration to achieve the desired angle
    #     duration = angle_rad / angular_speed_rad_s
    #     self.set_wheel_velocities(left_wheel_velocity, right_wheel_velocity, duration)

    def set_rotation_in_place(self, angle_degrees, angular_speed_deg_s):
        # Convert degrees to radians
        angle_rad = math.radians(angle_degrees)
        angular_speed_rad_s = math.radians(abs(angular_speed_deg_s))

        # Calculate wheel velocities
        v = (self.WHEEL_DISTANCE / 2) * angular_speed_rad_s
        wheel_velocity = v / self.WHEEL_RADIUS

        # Determine direction based on the sign of the angle
        if angle_degrees > 0:
            left_wheel_velocity = wheel_velocity
            right_wheel_velocity = -wheel_velocity
        else:
            left_wheel_velocity = -wheel_velocity
            right_wheel_velocity = wheel_velocity

        # Set velocities for a duration to achieve the desired angle
        duration = abs(angle_rad / angular_speed_rad_s)
        self.set_wheel_velocities(left_wheel_velocity, right_wheel_velocity, duration)

    def set_arc_movement(self, linear_speed_m_s, radius_m, arc_angle_degrees):
        # Calculate the arc length for the given angle
        arc_length = radius_m * math.radians(arc_angle_degrees)

        # Duration to traverse the arc
        duration = arc_length / linear_speed_m_s

        # Calculate wheel velocities for the arc
        v_outer = linear_speed_m_s * (radius_m + self.WHEEL_DISTANCE / 2) / radius_m
        v_inner = linear_speed_m_s * (radius_m - self.WHEEL_DISTANCE / 2) / radius_m

        # Convert linear velocity to angular velocity for the wheels
        outer_wheel_velocity = v_outer / self.WHEEL_RADIUS
        inner_wheel_velocity = v_inner / self.WHEEL_RADIUS

        # Set the wheel velocities with the calculated duration
        self.set_wheel_velocities(outer_wheel_velocity, inner_wheel_velocity, duration)

    def move(self, speed, duration):
        self.set_wheel_velocities(speed, speed, duration)

    def stop(self):
        # Stops all wheels
        for wheel in self.all_wheels:
            wheel.move(BotSpeed.STOP.value)


YOUBOT = YouBot(port=19999)
