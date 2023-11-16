from api import sim
import sys

LOCALHOST = '127.0.0.1'


def connect_to_port(port):
    print("Program started")
    sim.simxFinish(-1)
    client_id = sim.simxStart(LOCALHOST, port, True, True, 5000, 5)

    if client_id != -1:
        print('Connection Successful')
    else:
        sys.exit("Failed to connect")

    return client_id