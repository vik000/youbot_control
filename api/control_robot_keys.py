import sim
import time
import numpy as np
import sys
from youbot import YOUBOT

print("Program started")
sim.simxFinish(-1)
client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if client_id != -1:
    print('Connection Successful')
else:
    sys.exit("Failed to connect")

time.sleep(1)
error_code, left_wheel_handle = sim.simxGetObjectHandle(client_id, YOUBOT.left_wheel, sim.simx_opmode_oneshot_wait)
error_code, right_wheel_handle = sim.simxGetObjectHandle(client_id, YOUBOT.right_wheel, sim.simx_opmode_oneshot_wait)

error_code = sim.simxSetJointTargetVelocity(client_id, left_wheel_handle, 0.2, sim.simx_opmode_oneshot_wait)
error_code = sim.simxSetJointTargetVelocity(client_id, right_wheel_handle, 0.2, sim.simx_opmode_oneshot_wait)
