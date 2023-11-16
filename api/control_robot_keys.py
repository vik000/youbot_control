import sim
import time
import numpy as np
import sys
from api import connect
from youbot import YOUBOT

PORT = 19999


if __name__ == "__main__":
    client_id = connect.connect_to_port(PORT)

    time.sleep(1)
    error_code, left_wheel_handle = sim.simxGetObjectHandle(client_id, YOUBOT.left_wheel, sim.simx_opmode_oneshot_wait)
    error_code, right_wheel_handle = sim.simxGetObjectHandle(client_id, YOUBOT.right_wheel,
                                                             sim.simx_opmode_oneshot_wait)

    error_code = sim.simxSetJointTargetVelocity(client_id, left_wheel_handle, 0.2, sim.simx_opmode_oneshot_wait)
    error_code = sim.simxSetJointTargetVelocity(client_id, right_wheel_handle, 0.2, sim.simx_opmode_oneshot_wait)
