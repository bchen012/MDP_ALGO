#!/usr/bin/env python3
import numpy as np
import time
from algo.constants import *
from algo.fastest import FastestPath
import copy
import generate


class Exploration:

    """
    Right wall follower algorithm for a robot in an unknown map.

    Attributes:

    """

    def __init__(self, robot, time_limit=None, coverage=None, simulation=True):
        """
        Constructor

        Args:
            time_limit: float
            Maximum time allowed for exploration, in seconds

            simulation:  boolean, optional 
            To tell the class if the mode is simulation or real
        """
        self.robot = robot
        self.time_limit = time_limit
        self.coverage = coverage
        self.simulation = simulation
        self.virtual_wall = [0, 0, MAX_ROWS, MAX_COLS]
        self.one_round_completed = False

    def start(self):
        """
        Starting point of right wall follower explore
        trajectory: str
            for debugging only
        """
        start_time = time.time()
        if self.time_limit:
            end_time = start_time + self.time_limit

        trajectory = ''
        # the first round
        while not (self.check_completed() or self.one_round_completed):
            trajectory += self.move()
            if self.time_limit:
                if time.time() > end_time:
                    print('Time is over.')
                    print(trajectory)
                    print('Time used: {:.2f}'.format(time.time() - start_time))
                    return
            if self.coverage:
                actual_coverage = self.robot.compute_coverage()
                if actual_coverage >= self.coverage:
                    print('Coverage: {:.2f}'.format(actual_coverage))
                    return
        # complete one round but not explored fully
        while not self.check_completed():
            # unexplored block
            unexplored = np.argwhere(self.robot.current_map == 0)
            r, c = unexplored[0, 0], unexplored[0, 1]
            # get the neighbours of it
            neighour_layer1 = []
            for i in [-2, 2]:
                for j in [-1, 0, 1]:
                    neighour_layer1 += [(r+i, c+j), (r+j, c+i)]
            neighour_layer1 = [
                coord for coord in neighour_layer1 if self.check_valid(coord)]
            neighour_layer2 = []
            for i in [-3, 3]:
                for j in [-1, 0, 1]:
                    neighour_layer2 += [(r+i, c+j), (r+j, c+i)]
            neighour_layer2 = [
                coord for coord in neighour_layer2 if self.check_valid(coord)]
            neighour_layer3 = [(r+1, c-5), (r-5, c-1), (r-1, c+5), (r+5, c+1)]
            neighour_layer3 = [
                coord for coord in neighour_layer3 if self.check_valid(coord)]
            # try to explore this block
            exit_condition_1 = False
            for n in neighour_layer1 + neighour_layer2:
                if exit_condition_1:
                    break
                try:
                    fsp = FastestPath(self.robot.current_map, self.robot.center, np.asarray(
                        n), self.robot.direction)
                    fsp.run()
                    movements = fsp.movements
                    for move in movements:
                        self.robot.execute(move)
                        self.robot.update_map()
                        if self.coverage:
                            actual_coverage = self.robot.compute_coverage()
                            if actual_coverage >= self.coverage:
                                print('Coverage: {:.2f}'.format(
                                    actual_coverage))
                                return
                        if self.robot.current_map[r][c] != 0:
                            exit_condition_1 = True
                            break
                        if self.time_limit:
                            if time.time() > end_time:
                                print('Time is over.')
                                print('Time used: {:.2f}'.format(
                                    time.time() - start_time))
                                print(trajectory)
                                print(fsp.map_)
                                print(self.robot.center)
                                return
                except:
                    continue
            if self.robot.current_map[r][c] == 0:
                exit_condition_2 = False
                for n in neighour_layer3:
                    if exit_condition_2:
                        break
                    try:
                        fsp = FastestPath(self.robot.current_map, self.robot.center, np.asarray(
                            n), self.robot.direction)
                        fsp.run()
                        movements = fsp.movements + [LEFT, LEFT, LEFT]
                        for move in movements:
                            self.robot.execute(move)
                            self.robot.update_map()
                            if self.coverage:
                                actual_coverage = self.robot.compute_coverage()
                                if actual_coverage >= self.coverage:
                                    print('Coverage: {:.2f}'.format(
                                        actual_coverage))
                                    return
                            if self.robot.current_map[r][c] != 0:
                                exit_condition_2 = True
                                break
                            if self.time_limit:
                                if time.time() > end_time:
                                    print('Time is over.')
                                    print('Time used: {:.2f}'.format(
                                        time.time() - start_time))
                                    print(trajectory)
                                    print(fsp.map_)
                                    print(self.robot.center)
                                    return
                    except:
                        continue
            if self.time_limit:
                if time.time() > end_time:
                    print('Time is over.')
                    print('Time used: {:.2f}'.format(time.time() - start_time))
                    print(trajectory)
                    print(fsp.map_)
                    print(self.robot.center)
                    return
        # print(self.robot.current_map)
        print('Congrats! Exploration done.')
        np.savetxt('map/currentmap.txt',
                   self.robot.current_map, '%d', delimiter='')
        # update current map

        self.robot.send('E')
        time.sleep(0.5)
        # return to the start zone after exploration
        fsp = FastestPath(self.robot.current_map,
                          self.robot.center, START, self.robot.direction)
        fsp.run()
        movements = fsp.movements
        if(len(movements) == 0):
            self.robot.send('S')
        else:
            self.robot.send(movements)
        for move in movements:
            self.robot.execute_nosend(move)
            if self.robot.update_frontend:
                self.robot.update_frontend(
                    self.robot.current_map, self.robot.center, self.robot.cal_head())

        self.robot.receive()
        if self.robot.direction != WEST:
            calibrate_movement = self.robot.correct_direction(WEST)
            for move in calibrate_movement:
                self.robot.execute(move)
                if self.robot.update_frontend:
                    self.robot.update_frontend(
                        self.robot.current_map, self.robot.center, self.robot.cal_head())
        time.sleep(5)
        # MDF_string = generate.MDFString()
        # self.robot.send('Z'+MDF_string)
        self.robot.send('C')

    def check_valid(self, coord):
        """
        Check if the given coordinate is valid ( 3x3 neighourhood is within the wall and has value 1)
        """
        r, c = coord
        x, y = np.meshgrid([-1, 0, 1], [-1, 0, 1])
        x, y = x+r, y+c
        if np.any(x < 0) or np.any(y < 0) or np.any(x >= MAX_ROWS) or np.any(y >= MAX_COLS):
            return False
        elif np.any(self.robot.current_map[x[0, 0]:x[0, 2]+1, y[0, 0]:y[2, 0]+1] != 1):
            return False
        else:
            return True

    def check_completed(self):
        """
        check if the exploration is completed.

        Returns:
            Boolean 
        """
        return np.all(self.robot.current_map)

    def move(self):
        """
        compute the next move and execute the action
        the move returned is for recording and bebugging only
        """
        move = ''

        if self.direction_clear('LEFT'):
            self.robot.execute(LEFT)
            move += LEFT
            self.robot.update_map()
            if self.direction_clear('FRONT'):
                self.robot.execute(FORWARD)
                move += FORWARD
                self.robot.update_map()
        elif self.direction_clear('FRONT'):
            self.robot.execute(FORWARD)
            move += FORWARD
            self.robot.update_map()
        elif self.direction_clear('RIGHT'):
            self.robot.execute(RIGHT)
            move += RIGHT
            self.robot.update_map()
            if self.direction_clear('FRONT'):
                self.robot.execute(FORWARD)
                move += FORWARD
                self.robot.update_map()
        else:
            self.robot.execute(LEFT)
            move += LEFT
            self.robot.update_map()
            self.robot.execute(LEFT)
            move += LEFT
            self.robot.update_map()

        # mark if robot returns to the initial position after one round
        if (self.robot.center == START).all():
            self.one_round_completed = True
            print('one round completed!!!!')
            np.savetxt('map/currentmap.txt',
                       self.robot.current_map, '%d', delimiter='')
        return move

    def direction_clear(self, direction: str):
        """
        Checks if the given direction has no obstacles ahead.
        Return:
            Boolean
        """
        # convert to the objective direction corresponding to robot's subjective direction FRONT, LEFT, RIGHT
        orientation = dict()
        if self.robot.direction == NORTH:
            orientation = {'FRONT': NORTH, 'LEFT': WEST, 'RIGHT': EAST}
        elif self.robot.direction == EAST:
            orientation = {'FRONT': EAST, 'LEFT': NORTH, 'RIGHT': SOUTH}
        elif self.robot.direction == SOUTH:
            orientation = {'FRONT': SOUTH, 'LEFT': EAST, 'RIGHT': WEST}
        else:
            orientation = {'FRONT': WEST, 'LEFT': SOUTH, 'RIGHT': NORTH}

        # get the actual grids to check according to the objective direction
        row, col = self.robot.center
        grids = list()
        if orientation[direction] == NORTH:
            grids = [[row-2, col-1], [row-2, col], [row-2, col+1]]
        elif orientation[direction] == EAST:
            grids = [[row-1, col+2], [row, col+2], [row+1, col+2]]
        elif orientation[direction] == SOUTH:
            grids = [[row+2, col-1], [row+2, col], [row+2, col+1]]
        else:
            grids = [[row-1, col-2], [row, col-2], [row+1, col-2]]

        # check if the grids to move is within the wall and empty
        grids_empty = True
        for (x, y) in grids:
            if x < self.virtual_wall[0] or x >= self.virtual_wall[2] or y < self.virtual_wall[1] or y >= self.virtual_wall[3]:
                return False
            grids_empty = grids_empty and self.robot.current_map[x, y] == 1
        return grids_empty
