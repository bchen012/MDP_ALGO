#!/usr/bin/env python3
"""Predefined constant values to be used in the algorithms
"""
import numpy as np

# DIRECTIONS
NORTH = 1
EAST = 2
SOUTH = 3
WEST = 4

# MOVEMENTS
LEFT = "L"
RIGHT = "R"
FORWARD = "F"

# MAP CONSTANTS
MAX_ROWS = 20
MAX_COLS = 15
START = np.asarray([18, 1])
GOAL = np.asarray([1, 13])
BOTTOM_LEFT_CORNER = START
BOTTOM_RIGHT_CORNER = np.asarray([18, 13])
TOP_RIGHT_CORNER = GOAL
TOP_LEFT_CORNER = np.asarray([1, 1])
SIMU_MAP_FILE = './map/finalmap.txt'
REAL_MAP_FILE = './map/currentmap.txt'

# SOCKET CONNECTION
SERVER = "192.168.103.26"
PORT = 4000
ADDR = (SERVER, PORT)
FORMAT = "utf-8"
DISCONNECT_MESSAGE = "!DISCONNECT"
FSP_DONE_MSG = 'Done'

