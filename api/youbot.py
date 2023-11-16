import time
import math
from enum import Enum

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


class Robot:
    def __init__(self, name, port) -> None:
        self.name = name
        self.client_id = connect.connect_to_port(port)
        self.speed = BotSpeed.STOP


class YouBot(Robot):
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

    def set_wheel_velocities(self, left_wheel_velocity, right_wheel_velocity, duration):
        for wheel in self.left_wheels:
            wheel.move(left_wheel_velocity)

        for wheel in self.right_wheels:
            wheel.move(right_wheel_velocity)

        time.sleep(duration)
        self.stop()

    def set_rotation_in_place(self, angle_degrees, angular_speed_deg_s):
        # Convert degrees to radians
        angle_rad = math.radians(angle_degrees)
        angular_speed_rad_s = math.radians(angular_speed_deg_s)

        # Calculate wheel velocities
        v = (self.WHEEL_DISTANCE / 2) * angular_speed_rad_s
        left_wheel_velocity = v / self.WHEEL_RADIUS
        right_wheel_velocity = -v / self.WHEEL_RADIUS

        # Set velocities for a duration to achieve the desired angle
        duration = angle_rad / angular_speed_rad_s
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
            wheel.move(BotSpeed.STOP)


YOUBOT = YouBot(port=19999)
