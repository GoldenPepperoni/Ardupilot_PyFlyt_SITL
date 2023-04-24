"""
Run File for PyFlyt Reversed Tri-plane Physics backend
"""
# Run below line in /ardupilot/ArduPlane/
# sim_vehicle.py -f JSON:192.168.0.135 --console --map --add-param-file=/home/jimwong/Documents/Ardupilot_SITL/Prototype_Params_File.param -w

import os, inspect, sys

import socket
import struct
import json
import math
import time
import numpy as np

# use pymavlink for ArduPilot convention transformations
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion
from math import degrees, radians
from PyFlyt.core import Aviary


def vector_to_AP(vec):
    '''convert pybullet vector tuple to ArduPilot reference frame'''
    return (vec[0], -vec[1], -vec[2])


def connect(sock, parse_format, magic):
    """ 
        Function to connect to socket
    """
    try:
        data, address = sock.recvfrom(100)
    except:
        print("Trying to connect to Ardupilot SITL")
        time.sleep(0.1)
        return False

    if len(data) != struct.calcsize(parse_format):
        print(
            "got packet of len %u, expected %u"
            % (len(data), struct.calcsize(parse_format))
        )
        return False

    decoded = struct.unpack(parse_format, data)

    if magic != decoded[0]:
        print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
        return False

    # If connected, and dataframe is correct
    print(f"Successfully connected to Address:{address}")
    return True



def packageDataFrame(sock, address, state, sim_time):
    """ 
        Function to send sensors data to Ardupilot
    """

    gyro = vector_to_AP(state[0])
    euler = vector_to_AP(state[1])
    velo = vector_to_AP(state[2])
    pos = vector_to_AP(state[3])
    accel = vector_to_AP(state[4])

    # build JSON format
    IMU_fmt = {"gyro": gyro, "accel_body": accel}
    JSON_fmt = {
        "timestamp": sim_time,
        "imu": IMU_fmt,
        "position": pos,
        "attitude": euler,
        "velocity": velo,
    }
    JSON_string = "\n" + json.dumps(JSON_fmt, separators=(",", ":")) + "\n"

    # Send to AP
    sock.sendto(bytes(JSON_string, "ascii"), address)




if __name__ == "__main__":
    # Runs SITL with PyFlyt Backend
    # Bind to localhost port
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 9002))
    sock.settimeout(0.1)

    # Initialise simulation status
    last_SITL_frame = -1
    last_velocity = np.array([0.0, 0.0, 0.0])
    connected = False
    ended = False
    frame_count = 0

    # Dataframe info
    parse_format = "HHI16H"
    magic = 18458

    # Start PyFlyt simulator
    # Starting position and orientation of UAV
    start_pos = np.array([[0.0, 0.0, 0.5]])
    start_orn = np.array([[0.0, 0.0, 0.0]])

    phys_rate = 400
    phys_timestep = 1 / phys_rate
    env = Aviary(
        start_pos=start_pos, start_orn=start_orn, render=True, drone_type="revtriplane", drone_options=dict(control_hz=400, use_camera=True, starting_velocity=np.array([0, 0, 0])), physics_hz=phys_rate
    )

    # Connect to Ardupilot SITL
    while not connected:
        connected = connect(sock, parse_format, magic)

    while not ended:
        # Main simulation loop

        # Read data from Ardupilot
        try:
            data, address = sock.recvfrom(100)
        except:
            print("Error receiving data from Ardupilot, trying again...")
            continue
        
        decoded = struct.unpack(parse_format, data)

        frame_rate_hz = decoded[1]
        frame_count = decoded[2]

        # Check if the fame is in expected order
        if frame_count < last_SITL_frame:
            # Controller has reset, quit simulation
            print("Controller reset, Quitting...")
            break
        elif frame_count == last_SITL_frame:
            # Duplicate frame, skip
            print("Duplicate input frame")
            continue
        elif frame_count != last_SITL_frame + 1 and connected:
            print("Missed {0} input frames".format(frame_count - last_SITL_frame - 1))

        # Increment frame count
        last_SITL_frame = frame_count
        frame_count += 1
        sim_time = phys_timestep * frame_count

        # Simulation step
        pwm = [*decoded[3:10], *decoded[11:14]] # [LAil, RAil, Elev, Rud, LTilt, RTilt, FrontTilt, FrontMotor, RMotor, LMotor]
        action = np.array(pwm)
        env.set_setpoint(0, action)
        env.step()
        
        # Get UAV's states from simulation
        state = env.state(0).copy()

        # Calculate acceleration 
        accel = np.array([(state[2] - last_velocity) / phys_timestep])
        accel[-1] += 1
        state = np.concatenate((state, accel), axis=0)

        # Package sensor data into JSON and send to Ardupilot
        packageDataFrame(sock, address, state, sim_time)
        
        # Update for next loop
        last_velocity = state[2]