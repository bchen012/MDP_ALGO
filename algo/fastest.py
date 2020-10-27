#!/usr/bin/env python3
import copy
import numpy as np

from algo.constants import NORTH, SOUTH, EAST, WEST, MAX_ROWS, MAX_COLS, FORWARD, LEFT, RIGHT, START, GOAL, SIMU_MAP_FILE
from algo.mapmethod import map_from_file


class Node:

    """
    The Node of the FSP graph

    Attributes:
        coord: tuple
            (row, col)
        g: int
            cost from start to current
        h: int
            (under)estimated cost from current to goal
        parent: Node
            The parent of this node
        grid_val: int
            The value( 0 or 1 or 2) of the grid in the map
    """

    def __init__(self, coord, grid_val, h):
        self.coord = coord
        self.grid_val = grid_val
        self.parent = None
        self.h = h
        self.g = float('inf')


class FastestPath:

    """
    Implementation of the Astar algorithm to get fastest path in the 20x15 maze

    """

    def __init__(self, map_, start, goal, init_direction, waypoint=np.array([]), simulation=True):
        """
        Constructor to initialize an instance of the FastestPath class

        Args:
            map_: np array
                The map passed into FSP
            graph: List
            start: List 
                Coordinates of the starting position
            goal: List 
                Coordinates of the goal cell (center of the goal zone)
            waypoint: List
                The coordinate of the way-point
            direction: int
                The direction of the virtual robot
            init_direction: int (constant)
                The starting direction of the robot
            fsp: List<Node>
                The fastest path
            simulation: Boolean
                Whether it's in simulation mode or real run
            movements: List<int>:
                The movements robot should perform
        """
        self.map_ = map_
        self.graph = []
        self.start = start
        self.goal = goal
        self.waypoint = waypoint
        self.direction = init_direction
        self.init_direction = init_direction
        self.fsp = []
        self.simulation = simulation
        self.movements = []
        self.center_directions = []

    def run(self):
        """
        starting point of the FSP algo
        """
        fsp = []
        print(self.waypoint)
        # if self.waypoint.size != 0:
        #     # from start to waypoint
        #     self.init_graph(self.start, self.waypoint)
        #     fsp.extend(self.astar(self.start, self.waypoint)[:-1])
        #     # from waypoint to goal
        #     self.init_graph(self.waypoint, self.goal)
        #     fsp.extend(self.astar(self.start, self.waypoint))
        # else:
        self.init_graph(self.start, self.goal)
        fsp.extend(self.astar(self.start, self.goal))
        # show fsp
        self.fsp = fsp
        prev = None
        for coor in fsp:
            if prev:
                self.turn_direction(prev, coor)
            prev = coor
        # calculate movements
        self.cal_movements()

    def compute_heuristic(self, start, goal):
        col_idx, row_idx = np.meshgrid(range(0, 15), range(0, 20))
        h_matrix = (row_idx - goal[0]) + (col_idx - goal[1])
        return h_matrix

    def init_graph(self, start, goal):
        h_matrix = self.compute_heuristic(start, goal)
        for r in range(MAX_ROWS):
            each_row = []
            for c in range(MAX_COLS):
                each_row.append(Node((r, c), self.map_[r][c], h_matrix[r][c]))
            self.graph.append(each_row)

    def astar(self, start, goal):
        goal_node = self.graph[goal[0]][goal[1]]
        # set of visited nodes
        visited = list()
        # set of frontier nodes
        frontier = list()
        # current node
        cur = self.graph[start[0]][start[1]]
        cur.g = 0
        frontier.append(cur)
        prev = None

        while frontier:
            cur = min(frontier, key=lambda x: x.g + x.h)
            if cur == goal_node:
                path = []
                while cur:
                    path.append(cur.coord)
                    cur = cur.parent
                return path[::-1]
            else:
                frontier.remove(cur)
                for neighour in self.get_neighbours(cur):
                    if neighour in visited:
                        continue
                    if neighour in frontier:
                        # calculate new g()
                        new_g = cur.g + \
                            self.calculate_g(cur.coord, neighour.coord)
                        if neighour.g > new_g:
                            neighour.g = new_g
                            neighour.parent = cur
                    else:
                        # cost of moving between neighbours is 1
                        neighour.g = cur.g + \
                            self.calculate_g(cur.coord, neighour.coord)
                        neighour.parent = cur
                        if neighour not in frontier:
                            frontier.append(neighour)
                if cur not in visited:
                    visited.append(cur)
            prev = copy.deepcopy(cur)
        # exception is no path is found
        raise ValueError('No Path Found')

    # def sensor_reachable(self, cur, goal):
    #     cur = cur.coord
    #     r, c = goal.coord
    #     if cur in [(r-2, c), (r+2, c), (r, c-2), (r, c+2)] and self.check_valid(cur):
    #         return True
    #     else:
    #         return False

    def turn_direction(self, prev_coord, cur_coord):
        print(prev_coord)
        print(cur_coord)
        if prev_coord[0] < cur_coord[0]:
            self.direction = SOUTH
        elif prev_coord[1] < cur_coord[1]:
            self.direction = EAST
        elif prev_coord[1] > cur_coord[1]:
            self.direction = WEST
        else:
            self.direction = NORTH
        print(self.direction)

    def get_neighbours(self, node):
        """
        Returns the coordinates of valid neighours which are explored and are not obstacles 
        ( has grid alue 1)

        """
        r, c = node.coord
        neighour_idxs = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
        neighours = [self.graph[n[0]][n[1]]
                     for n in neighour_idxs if self.check_valid(n)]
        return neighours

    def check_valid(self, coord):
        """
        Check if the given coordinate is valid ( 3x3 neighourhood is within the wall and has value 1)
        """
        r, c = coord
        x, y = np.meshgrid([-1, 0, 1], [-1, 0, 1])
        x, y = x+r, y+c
        if np.any(x < 0) or np.any(y < 0) or np.any(x >= MAX_ROWS) or np.any(y >= MAX_COLS):
            return False
        elif np.any(self.map_[x[0, 0]:x[0, 2]+1, y[0, 0]:y[2, 0]+1] != 1):
            return False
        else:
            return True

    def calculate_g(self, cur_coord, next_coord):
        if self.direction in [NORTH, SOUTH]:
            return 1 if cur_coord[1] == next_coord[1] else 2
        else:
            return 1 if cur_coord[0] == next_coord[0] else 2

    def cal_movements(self):
        robot_center = copy.deepcopy(self.start)
        robot_direction = copy.deepcopy(self.init_direction)
        for coord in self.fsp:
            movement = []
            tmp_center_directions = []
            if robot_center.tolist() != coord:
                diff = robot_center - np.asarray(coord)
                robot_center = tuple(robot_center)
                if (diff[0] == -1 and diff[1] == 0):  # Going south
                    if robot_direction == NORTH:
                        movement.extend((RIGHT, RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, EAST), (robot_center, SOUTH), (coord, SOUTH)])
                    elif robot_direction == EAST:
                        movement.extend((RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, SOUTH), (coord, SOUTH)])
                    elif robot_direction == SOUTH:
                        movement.append(FORWARD)
                        tmp_center_directions.extend([(coord, SOUTH)])
                    else:
                        movement.extend((LEFT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, WEST), (coord, SOUTH)])
                    robot_direction = SOUTH
                elif (diff[0] == 0 and diff[1] == 1):  # Going west
                    if robot_direction == NORTH:
                        movement.extend((LEFT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, WEST), (coord, WEST)])
                    elif robot_direction == EAST:
                        movement.extend((RIGHT, RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, SOUTH), (robot_center, WEST), (coord, WEST)])
                    elif robot_direction == SOUTH:
                        movement.extend((RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, WEST), (coord, WEST)])
                    else:
                        movement.append(FORWARD)
                        tmp_center_directions.extend([(coord, WEST)])
                    robot_direction = WEST
                elif (diff[0] == 0 and diff[1] == -1):  # Going east
                    if robot_direction == NORTH:
                        movement.extend((RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, EAST), (coord, EAST)])
                    elif robot_direction == EAST:
                        movement.append(FORWARD)
                        tmp_center_directions.extend([(coord, EAST)])
                    elif robot_direction == SOUTH:
                        movement.extend((LEFT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, EAST), (coord, EAST)])
                    else:
                        movement.extend((RIGHT, RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, NORTH), (robot_center, EAST), (coord, EAST)])
                    robot_direction = EAST
                elif (diff[0] == 1 and diff[1] == 0):  # Going north
                    if robot_direction == NORTH:
                        movement.append(FORWARD)
                        tmp_center_directions.extend([(coord, NORTH)])
                    elif robot_direction == EAST:
                        movement.extend((LEFT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, NORTH), (coord, NORTH)])
                    elif robot_direction == SOUTH:
                        movement.extend((RIGHT, RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, WEST), (robot_center, NORTH), (coord, NORTH)])
                    else:
                        movement.extend((RIGHT, FORWARD))
                        tmp_center_directions.extend(
                            [(robot_center, NORTH), (coord, NORTH)])
                    robot_direction = NORTH
                robot_center = np.asarray(robot_center)
                robot_center -= diff
            self.movements.extend(movement)
            self.center_directions.extend(tmp_center_directions)


# testing FSP
if __name__ == '__main__':
    map_ = map_from_file(SIMU_MAP_FILE)
    fsp = FastestPath(map_, START, GOAL, NORTH)
    fsp.run()
