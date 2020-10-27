#!/usr/bin/env python3
import numpy as np
from algo.constants import *
from algo.exploration import Exploration
from algo.mapmethod import map_from_file
import time

def send(string):
    time.sleep(0.1)

def receive():
    time.sleep(0.1)

class Robot:

    """
    Robot class is a representation of the robot. 
    Its child class SimuRobot is for simulation, RealRobot is for the actual robot.

    Attributes:

        current_map: Numpy array
        The current map known by the robot

        direction: int
        NORTH = 1
        EAST = 2
        SOUTH = 3
        WEST = 4

    """

    def __init__(self, direction, start_location, update_frontend=None):
        """
        Constructor:

        Args:
            start_location: Numpy Array
        """
        self.direction = direction
        self.center = start_location
        self.current_map = np.zeros([20, 15])
        self.mark_neighborhood(start_location, 1)
        self.head = None
        if direction == NORTH:
            self.head = start_location + [-1, 0]
        elif direction == SOUTH:
            self.head = start_location + [1, 0]
        elif direction == WEST:
            self.head = start_location + [-1, 0]
        else:
            self.head = start_location + [-1, 0]

        self.update_frontend = update_frontend

    def update_map(self):
        """
        Update the current map known by the robot according to sensor data.
        """
        pass

    def mark_neighborhood(self, center, value):
        """
        To mark a 3x3 neighbourhood around the center location with a user defined value
        """
        self.current_map[center[0]-1:center[0] +
                         2, center[1]-1:center[1]+2] = value

    def compute_coverage(self):
        return np.count_nonzero(self.current_map) / (15*20)


class SimuRobot(Robot):
    '''
    child class SimuRobot extends Robot, is for simulation.
    '''

    def __init__(self, direction, start_location, simu_map, update_frontend=None,send=send, receive=receive):
        super().__init__(direction, start_location, update_frontend)
        self.simu_map = simu_map
        self.update_frontend = update_frontend
        self.update_map()
        self.send = send
        self.receive = receive

    def get_sensor_grids(self):
        """
        generate the indices which the six sensors ( in clockwise direction from front left) would detect

        Returns:
           the indices (list of list)
        """
        short_range = 2
        long_range = 4
        r, c = self.center
        grids = []

        # Front Left
        if self.direction == NORTH:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c-1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip([r-1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(
                zip([r+1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        else:
            idxs = list(zip(range(r+2, r+short_range+2), [c+1]*short_range))
            grids.append(idxs)

        # Front Center
        if self.direction == NORTH:
            idxs = list(
                zip(range(r-short_range-1, r-1), [c]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip([r]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)

        elif self.direction == WEST:
            idxs = list(
                zip([r]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        else:
            idxs = list(zip(range(r+2, r+short_range+2), [c]*short_range))
            grids.append(idxs)

        # Front Right
        if self.direction == NORTH:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c+1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip([r+1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(
                zip([r-1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        else:
            idxs = list(zip(range(r+2, r+short_range+2), [c-1]*short_range))
            grids.append(idxs)

        # Right Top
        if self.direction == NORTH:
            idxs = list(zip([r-1]*long_range, range(c+2, c+long_range+2)))
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip(range(r+2, r+long_range+2), [c+1]*long_range))
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(zip(range(r-long_range-1, r-1),
                            [c-1]*long_range))[::-1]
            grids.append(idxs)
        else:
            idxs = list(
                zip([r+1]*long_range, range(c-long_range-1, c-1)))[::-1]
            grids.append(idxs)

        # Left Bottom
        if self.direction == NORTH:
            idxs = list(
                zip([r+1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c-1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(zip(range(r+2, r+short_range+2), [c+1]*short_range))
            grids.append(idxs)
        else:
            idxs = list(zip([r+1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)

        # Left Top
        if self.direction == NORTH:
            idxs = list(
                zip([r-1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c+1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(zip(range(r+2, r+short_range+2), [c-1]*short_range))
            grids.append(idxs)
        else:
            idxs = list(zip([r-1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)

        return grids

    def update_map(self):
        """
        simulate sensor data from simu_map, and update the current map known by the robot
        """
        grids = self.get_sensor_grids()
        for each in grids:
            for (r, c) in each:
                if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS):
                    if self.simu_map[r][c] == 2:
                        if self.current_map[r][c] == 0:
                            self.current_map[r][c] = 2
                        break
                    else:
                        if self.current_map[r][c] == 0:
                            self.current_map[r][c] = 1
        self.update_frontend(self.current_map, self.center,self.cal_head())
    
    def cal_head(self):
        if self.direction == NORTH:
            head = self.center + [-1, 0]
        elif self.direction == SOUTH:
            head = self.center + [1, 0]
        elif self.direction == WEST:
            head = self.center + [0, -1]
        else:
            head = self.center + [0, 1]
        return head

    def execute(self, movement: str):
        """
        move the robot according to the instruction given by a str (single movement)
        """

        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
            elif movement == LEFT:
                self.direction = WEST
            else:
                self.center = self.center + [-1, 0]
                self.mark_neighborhood(self.center, 1)
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
            elif movement == LEFT:
                self.direction = NORTH
            else:
                self.center = self.center + [0, 1]
                self.mark_neighborhood(self.center, 1)

        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
            elif movement == LEFT:
                self.direction = EAST
            else:
                self.center = self.center + [1, 0]
                self.mark_neighborhood(self.center, 1)
        else:
            if movement == RIGHT:
                self.direction = NORTH
            elif movement == LEFT:
                self.direction = SOUTH
            else:
                self.center = self.center + [0, -1]
                self.mark_neighborhood(self.center, 1)

        time.sleep(0.1)

    def execute_nosend(self, movement: str):
        """
        move the robot according to the instruction given by a str (single movement)
        """
        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
            elif movement == LEFT:
                self.direction = WEST
            else:
                self.center = self.center + [-1, 0]
                self.mark_neighborhood(self.center, 1)
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
            elif movement == LEFT:
                self.direction = NORTH
            else:
                self.center = self.center + [0, 1]
                self.mark_neighborhood(self.center, 1)

        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
            elif movement == LEFT:
                self.direction = EAST
            else:
                self.center = self.center + [1, 0]
                self.mark_neighborhood(self.center, 1)
        else:
            if movement == RIGHT:
                self.direction = NORTH
            elif movement == LEFT:
                self.direction = SOUTH
            else:
                self.center = self.center + [0, -1]
                self.mark_neighborhood(self.center, 1)

        # update head
        if self.direction == NORTH:
            self.head = self.center + [-1, 0]
        elif self.direction == SOUTH:
            self.head = self.center + [1, 0]
        elif self.direction == WEST:
            self.head = self.center + [0, -1]
        else:
            self.head = self.center + [0, 1]

    def correct_direction(self,target_direction):
        movement = []
        if self.direction == NORTH:
            movement.append(LEFT)
        elif self.direction == EAST:
            movement.extend([LEFT,LEFT])
        elif self.direction == SOUTH:
            movement.append(RIGHT)
        
        return movement


class RealRobot(Robot):
    '''
    child class RealRobot extends Robot, is for the real execution.
    '''

    def __init__(self, direction, start_location, send, receive, update_frontend):
        super().__init__(direction, start_location, update_frontend)
        self.mark_neighborhood(start_location, 1)
        self.send = send
        self.receive = receive
        self.update_map()

    # def getValue(self, inds, value, distance, sr):
    #     print(inds)
    #     vals = []
    #     for idx, (r, c) in enumerate(inds):
    #         if (0 <= r < 20) and (0 <= c < 15):
    #             # for override
    #             if (self.current_map[r][c] == 2 and vals[idx] == 1 and sr and (not right)):
    #                 self.current_map[r][c] = vals[idx]
    #             elif self.current_map[r][c] == 2:
    #                 break
    #             elif (self.current_map[r][c] == 0):
    #                 self.current_map[r][c] = vals[idx]
    #         else:
    #             if self.current_map[r][c] == 2:
    #                 break
    #             elif (self.current_map[r][c] == 0):
    #                 self.current_map[r][c] = vals[idx]
    #             # without override
    #             # if self.current_map[r][c] == 0:
    #             #     self.current_map[r][c] = vals[idx]
    #             # # elif (sr and self.marked[r][c] == 0):
    #             # #     self.current_map[r][c] = vals[idx]
    #             # #     self.marked[r][c] = 1
    #             # elif self.current_map[r][c] == 2:
    #             #     break

    # def getSensors(self, sensor_vals):
    #     """Generated indices to get values from sensors and gets the values using getValue() function.
    #     Returns:
    #         Numpy array of Numpy arrays: Sensor values from all sensors
    #     """
    #     distanceShort = 2
    #     distanceLong = 4
    #     r, c = self.center
    #     # sensor_vals = [FL_SR, FC_SR, FR_SR, RT_SR, RB_LR, LT_LR]
    #     print(sensor_vals)
    #     # Front Left
    #     if self.direction == NORTH:
    #         self.getValue(list(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort))[::-1],
    #                       sensor_vals[0], distanceShort, True)
    #     elif self.direction == EAST:
    #         self.getValue(list(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2))),
    #                       sensor_vals[0], distanceShort, True)
    #     elif self.direction == WEST:
    #         self.getValue(list(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1)))[::-1],
    #                       sensor_vals[0], distanceShort, True)
    #     else:
    #         self.getValue(list(zip(range(r+2, r+distanceShort+2), [c+1]*distanceShort)),
    #                       sensor_vals[0], distanceShort, True)

    #     # Front Center
    #     if self.direction == NORTH:
    #         self.getValue(list(zip(range(r-distanceShort-1, r-1), [c]*distanceShort))[::-1],
    #                       sensor_vals[1], distanceShort, True)
    #     elif self.direction == EAST:
    #         self.getValue(list(zip([r]*distanceShort, range(c+2, c+distanceShort+2))),
    #                       sensor_vals[1], distanceShort, True)
    #     elif self.direction == WEST:
    #         self.getValue(list(zip([r]*distanceShort, range(c-distanceShort-1, c-1)))[::-1],
    #                       sensor_vals[1], distanceShort, True)
    #     else:
    #         self.getValue(list(zip(range(r+2, r+distanceShort+2), [c]*distanceShort)),
    #                       sensor_vals[1], distanceShort, True)

    #     # Front Right
    #     if self.direction == NORTH:
    #         self.getValue(list(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort))[::-1],
    #                       sensor_vals[2], distanceShort, True)
    #     elif self.direction == EAST:
    #         self.getValue(list(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2))),
    #                       sensor_vals[2], distanceShort, True)
    #     elif self.direction == WEST:
    #         self.getValue(list(zip([r-1]*distanceShort, range(c-distanceShort-1, c-1)))[::-1],
    #                       sensor_vals[2], distanceShort, True)
    #     else:
    #         self.getValue(list(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort)),
    #                       sensor_vals[2], distanceShort, True)

    #     # Right Top
    #     if self.direction == NORTH:
    #         self.getValue(list(zip([r-1]*distanceLong, range(c+2, c+distanceLong+2))),
    #                       sensor_vals[3], distanceLong, True, True)
    #     elif self.direction == EAST:
    #         self.getValue(list(zip(range(r+2, r+distanceLong+2), [c+1]*distanceLong)),
    #                       sensor_vals[3], distanceLong, True, True)
    #     elif self.direction == WEST:
    #         self.getValue(list(zip(range(r-distanceLong-1, r-1), [c-1]*distanceLong))[::-1],
    #                       sensor_vals[3], distanceLong, True, True)
    #     else:
    #         self.getValue(list(zip([r+1]*distanceLong, range(c-distanceLong-1, c-1)))[::-1],
    #                       sensor_vals[3], distanceLong, True, True)

    #     # Left Bottom
    #     if self.direction == NORTH:
    #         self.getValue(list(zip([r+1]*distanceShort, range(c-distanceShort-1, c-1)))[::-1],
    #                       sensor_vals[4], distanceShort, False)
    #     elif self.direction == EAST:
    #         self.getValue(list(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort))[::-1],
    #                       sensor_vals[4], distanceShort, False)
    #     elif self.direction == WEST:
    #         self.getValue(list(zip(range(r-distanceShort-1, r-1), [c-1]*distanceShort)),
    #                       sensor_vals[4], distanceShort, False)
    #     else:
    #         self.getValue(list(zip([r-1]*distanceShort, range(c+2, c+distanceShort+2))),
    #                       sensor_vals[4], distanceShort, False)

    #     # Left Top
    #     if self.direction == NORTH:
    #         self.getValue(list(zip([r-1]*distanceShort, range(c-distanceShort-1, c-1)))[::-1],
    #                       sensor_vals[5], distanceShort, False)
    #     elif self.direction == EAST:
    #         self.getValue(list(zip(range(r-distanceShort-1, r-1), [c+1]*distanceShort))[::-1],
    #                       sensor_vals[5], distanceShort, False)
    #     elif self.direction == WEST:
    #         self.getValue(list(zip(range(r+2, r+distanceShort+2), [c-1]*distanceShort)),
    #                       sensor_vals[5], distanceShort, False)
    #     else:
    #         self.getValue(list(zip([r+1]*distanceShort, range(c+2, c+distanceShort+2))),
    #                       sensor_vals[5], distanceShort, False)
    def get_sensor_grids(self):
        """
        generate the indices which the six sensors ( in clockwise direction from front left) would detect

        Returns:
           the indices (list of list)
        """
        short_range = 2
        long_range = 4
        r, c = self.center
        grids = []

        # Front Left
        if self.direction == NORTH:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c-1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip([r-1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(
                zip([r+1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        else:
            idxs = list(zip(range(r+2, r+short_range+2), [c+1]*short_range))
            grids.append(idxs)

        # Front Center
        if self.direction == NORTH:
            idxs = list(
                zip(range(r-short_range-1, r-1), [c]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip([r]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)

        elif self.direction == WEST:
            idxs = list(
                zip([r]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        else:
            idxs = list(zip(range(r+2, r+short_range+2), [c]*short_range))
            grids.append(idxs)

        # Front Right
        if self.direction == NORTH:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c+1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip([r+1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(
                zip([r-1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        else:
            idxs = list(zip(range(r+2, r+short_range+2), [c-1]*short_range))
            grids.append(idxs)

        # Right Top
        if self.direction == NORTH:
            idxs = list(zip([r-1]*long_range, range(c+2, c+long_range+2)))
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip(range(r+2, r+long_range+2), [c+1]*long_range))
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(zip(range(r-long_range-1, r-1),
                            [c-1]*long_range))[::-1]
            grids.append(idxs)
        else:
            idxs = list(
                zip([r+1]*long_range, range(c-long_range-1, c-1)))[::-1]
            grids.append(idxs)

        # Left Bottom
        if self.direction == NORTH:
            idxs = list(
                zip([r+1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c-1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(zip(range(r+2, r+short_range+2), [c+1]*short_range))
            grids.append(idxs)
        else:
            idxs = list(zip([r-1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)

        # Left Top
        if self.direction == NORTH:
            idxs = list(
                zip([r-1]*short_range, range(c-short_range-1, c-1)))[::-1]
            grids.append(idxs)
        elif self.direction == EAST:
            idxs = list(zip(range(r-short_range-1, r-1),
                            [c+1]*short_range))[::-1]
            grids.append(idxs)
        elif self.direction == WEST:
            idxs = list(zip(range(r+2, r+short_range+2), [c-1]*short_range))
            grids.append(idxs)
        else:
            idxs = list(zip([r+1]*short_range, range(c+2, c+short_range+2)))
            grids.append(idxs)

        return grids

    def update_map(self):
        msg = self.receive()
        split_data = msg.split(":")
        if (split_data[0] == 'Explore'):
            print(list(split_data[1])[:6])
            sensors = list(map(float, list(split_data[1])[:6]))
            
            grids = self.get_sensor_grids()
            print(grids)
            for idx, each in enumerate(grids):
                blocks_empty = sensors[idx]
                for i in range(len(each)):
                    (r, c) = each[i]
                    if (0 <= r < MAX_ROWS) and (0 <= c < MAX_COLS):
                        if i >= blocks_empty:
                            # if self.current_map[r][c] == 0:
                            self.current_map[r][c] = 2
                            break
                        else:
                            # if self.current_map[r][c] == 0:
                            self.current_map[r][c] = 1

            print(self.cal_head())
            self.update_frontend(self.current_map, self.center,
                                 self.cal_head())

    def cal_head(self):
        if self.direction == NORTH:
            head = self.center + [-1, 0]
        elif self.direction == SOUTH:
            head = self.center + [1, 0]
        elif self.direction == WEST:
            head = self.center + [0, -1]
        else:
            head = self.center + [0, 1]
        return head

    def descriptor_1(self):
        descriptor = np.zeros([20, 15]).astype(int)
        descriptor[self.current_map[::-1, :] != 0] = 1
        bits = '11'
        for row in descriptor:
            bits += ''.join(map(str, row.tolist()))
        bits += '11'
        hex_str = ['%X' % int(bits[i:i+4], 2)
                   for i in range(0, len(bits)-3, 4)]
        return ''.join(hex_str)

    def descriptor_2(self):
        bits = ''
        for row in self.current_map[::-1, :]:
            for bit in row:
                if bit == 2:
                    bits += '1'
                elif bit != 0:
                    bits += '0'
        bits += '0'*(4 - len(bits) % 4)
        hex_str = ['%X' % int(bits[i:i+4], 2)
                   for i in range(0, len(bits)-3, 4)]
        return ''.join(hex_str)

    def execute(self, movement: str):
        """
        move the robot according to the instruction given by a str (single movement)
        """
        self.send(movement)
        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
            elif movement == LEFT:
                self.direction = WEST
            else:
                self.center = self.center + [-1, 0]
                self.mark_neighborhood(self.center, 1)
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
            elif movement == LEFT:
                self.direction = NORTH
            else:
                self.center = self.center + [0, 1]
                self.mark_neighborhood(self.center, 1)

        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
            elif movement == LEFT:
                self.direction = EAST
            else:
                self.center = self.center + [1, 0]
                self.mark_neighborhood(self.center, 1)
        else:
            if movement == RIGHT:
                self.direction = NORTH
            elif movement == LEFT:
                self.direction = SOUTH
            else:
                self.center = self.center + [0, -1]
                self.mark_neighborhood(self.center, 1)

        # update head
        if self.direction == NORTH:
            self.head = self.center + [-1, 0]
        elif self.direction == SOUTH:
            self.head = self.center + [1, 0]
        elif self.direction == WEST:
            self.head = self.center + [0, -1]
        else:
            self.head = self.center + [0, 1]

        # if self.update_frontend:
        #     self.update_frontend(self.current_map, self.center, self.head)

    def correct_direction(self,target_direction):
        movement = []
        if self.direction == NORTH:
            movement.append(LEFT)
        elif self.direction == EAST:
            movement.extend([LEFT,LEFT])
        elif self.direction == SOUTH:
            movement.append(RIGHT)
        
        return movement


    def execute_nosend(self, movement: str):
        """
        move the robot according to the instruction given by a str (single movement)
        """
        if self.direction == NORTH:
            if movement == RIGHT:
                self.direction = EAST
            elif movement == LEFT:
                self.direction = WEST
            else:
                self.center = self.center + [-1, 0]
                self.mark_neighborhood(self.center, 1)
        elif self.direction == EAST:
            if movement == RIGHT:
                self.direction = SOUTH
            elif movement == LEFT:
                self.direction = NORTH
            else:
                self.center = self.center + [0, 1]
                self.mark_neighborhood(self.center, 1)

        elif self.direction == SOUTH:
            if movement == RIGHT:
                self.direction = WEST
            elif movement == LEFT:
                self.direction = EAST
            else:
                self.center = self.center + [1, 0]
                self.mark_neighborhood(self.center, 1)
        else:
            if movement == RIGHT:
                self.direction = NORTH
            elif movement == LEFT:
                self.direction = SOUTH
            else:
                self.center = self.center + [0, -1]
                self.mark_neighborhood(self.center, 1)

        # update head
        if self.direction == NORTH:
            self.head = self.center + [-1, 0]
        elif self.direction == SOUTH:
            self.head = self.center + [1, 0]
        elif self.direction == WEST:
            self.head = self.center + [0, -1]
        else:
            self.head = self.center + [0, 1]

