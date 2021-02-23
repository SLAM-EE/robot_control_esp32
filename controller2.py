#!/usr/bin/env python3
"""
Module to controll the robot using keyboard inputs
using udp communication
"""

import numpy as np
import socket
import sys
# import time
# import _thread
import threading


from pynput import keyboard

UDP_IP = ""
UDP_PORT = ""

SOCK_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)


VALID = ['A', 'W', 'S', 'D', 'a', 'w', 's', 'd', 'p']
V_OPT = 40
THETA = 0
THETA_MAX = 45
VL = V_OPT
VR = V_OPT
# ACCL = 0
# temp = 50
# DECL = 1
# V_MAX = 45
V_MIN = 35
# V_CUTOFF = 33
# A_STEP = 2


# def deccelerate(threadName, delay):
#     global VL, VR
#     while(True):
#         VL = VL - (DECL + ACCL * 0.8) if VL > V_CUTOFF else V_CUTOFF
#         VR = VR - (DECL + ACCL * 0.8)if VR > V_CUTOFF else V_CUTOFF
#         print("added friction",VL,VR)
#         if(VL <= V_CUTOFF and VR <= V_CUTOFF):
#             message = bytearray('p', 'utf-8')
#             message.append(int(VL) % V_MAX)
#             message.append(int(VR) % V_MAX)
#             SOCK_.sendto(message, (UDP_IP, UDP_PORT))
#         time.sleep(1)

def on_press(key):
    # global ACCL, THETA, VL, VR, temp
    global VL, VR, THETA

    try:
        data_in = key.char
        if data_in in VALID:
            # ACCL = ACCL+A_STEP if(data_in == 'w') else ACCL-A_STEP if(data_in == 's') else ACCL
            # ACCL %= 4
            THETA = THETA+1 if(data_in == 'd') else THETA-1 if(data_in == 'a') else THETA
            drn = np.sign(THETA)            # direction of turn

            THETA = drn * THETA_MAX if np.abs(THETA) > THETA_MAX else THETA
            VL = VL + drn * np.cos(np.pi * THETA / 180) if (THETA != 0) else VL
            VR = VR - drn * np.cos(np.pi * THETA / 180) if (THETA != 0) else VR
            message = bytearray(data_in, 'utf-8')
            if(data_in == 'w'):
                VL = V_OPT
                VR = V_OPT
            message.append(int(VL))
            message.append(int(VR))

            SOCK_.sendto(message, (UDP_IP, UDP_PORT))
            print(" 1. Client Sent : ", data_in)
        else:
            print("ERROR: INVALID INPUT!")

    except AttributeError:
        print('special key {0} pressed'.format(key))

        # close the socket
        SOCK_.close()
        sys.exit(0)


def on_release(key):
    """ sending stop signal on key release"""
    message = bytearray('p', 'utf-8')
    message.append(int(VL))
    message.append(int(VR))
    SOCK_.sendto(message, (UDP_IP, UDP_PORT))


if(__name__ == "__main__"):
    if len(sys.argv) == 3:
        # Get "IP address of Server" and also the
        # "port number" from argument 1 and argument 2
        UDP_IP = sys.argv[1]
        UDP_PORT = int(sys.argv[2])
    else:
        print("Run like : " + sys.argv[0] + " <arg1 server ip 192.168.1.102> \
                <arg2 server port 4444 >")
        sys.exit(1)

    # send key press through UDP
    print("Type command (A(L),W(U),S(D),D(R)) =>")

    # x = threading.Thread(target=deccelerate, args=("Deccel2", 1))
    # x.start()
    listener = keyboard.Listener(on_press=on_press, on_release=on_release, suppress=True)
    listener.start()

    listener.join()
