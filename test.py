import socket
import os
import time
import numpy as np
import threading
from algo.exploration import Exploration
from algo.fastest import FastestPath
import tornado.web
import tornado.ioloop
import tornado.websocket
import asyncio
import sys
import json
from algo.constants import *
from algo.mapmethod import map_from_file
from algo.robot import *
from threading import Thread
from collections import Counter

currentMap = np.zeros([20, 15])
area = 0
exp = ''
fsp = ''
visited = dict()
waypoint = np.asarray([5, 5])
steps = 0
numCycle = 1
# t_s = 0
direction = 1

clients = dict()


class FuncThread(threading.Thread):

    """Class to create and run functions on different threads
    """

    def __init__(self, target, *args):
        """Construction to initialize the thread

        Args:
            target (function): Function to be run on new threads
            *args: arguments to be passed to the function
        """
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        """Overrides run function to run the function with given arguments
        """
        self._target(*self._args)


class WebSocketHandler(tornado.websocket.WebSocketHandler):

    """Handles web-socket requests from the front-end to receive/send messages
    Attributes:
        id (string): id string from GET request
    """

    def open(self):
        """Open a web socket for communication
        """
        self.id = self.get_argument(name="Id", default="Id")
        # self.stream.set_nodelay(True)
        clients[self.id] = {"id": self.id, "object": self}
        print("WebSocket opened")

    def on_message(self, message):
        """Displays any message received
        Args:
            message (string): Message received from front-end
        """
        print("Client " + str(self.id) + " received a message : " + str(message))

    def on_close(self):
        """Run when the web socket is closed
        """
        print("WebSocket closed")
        if self.id in clients:
            del clients[self.id]


class staticRequestHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("GUI/index.html")


class StartHandler(tornado.web.RequestHandler):

    """Handles the start of exploration for the maze
    """
    def get(self):
        # self.write("Starting...")
        # self.step = self.get_argument("step")
        # self.limit = self.get_argument("limit")
        # self.coverage = self.get_argument("coverage")
        # global step
        # step = float(self.step)
        startExploration()
        self.flush()


class FSPHandler(tornado.web.RequestHandler):

    """Handles the start of exploration for the maze
    """
    def get(self):
        # self.write("Starting...")
        # self.step = self.get_argument("step")
        # self.limit = self.get_argument("limit")
        # self.coverage = self.get_argument("coverage")
        # global step
        # step = float(self.step)
        startFSP()
        self.flush()


def startExploration():
    simu_map = map_from_file(SIMU_MAP_FILE)
    robot = SimuRobot(NORTH, START, simu_map, update_frontend)
    exploration = Exploration(robot, time_limit=100)
    exploration.start()


def startFSP():
    simu_map = map_from_file(SIMU_MAP_FILE)
    print(START, GOAL)
    fsp = FastestPath(simu_map, START, GOAL, NORTH)
    fsp.run()
    print(START, GOAL)
    movements = fsp.movements
    print(fsp.movements)
    center, direction = START, NORTH
    print(center, START, direction)
    update_frontend(simu_map, center, cal_head(center, direction))
    for movement in movements:
        center, direction = execute(center, direction, movement)
        update_frontend(simu_map, center, cal_head(center, direction))


def cal_head(center, direction):
    if direction == NORTH:
        head = center + [-1, 0]
    elif direction == SOUTH:
        head = center + [1, 0]
    elif direction == WEST:
        head = center + [0, -1]
    else:
        head = center + [0, 1]
    return head


def execute(center, direction, movement):
    """
    move the robot according to the instruction given by a str (single movement)
    """

    if direction == NORTH:
        if movement == RIGHT:
            direction = EAST
        elif movement == LEFT:
            direction = WEST
        else:
            center = center + [-1, 0]
    elif direction == EAST:
        if movement == RIGHT:
            direction = SOUTH
        elif movement == LEFT:
            direction = NORTH
        else:
            center = center + [0, 1]
    elif direction == SOUTH:
        if movement == RIGHT:
            direction = WEST
        elif movement == LEFT:
            direction = EAST
        else:
            center = center + [1, 0]
    else:
        if movement == RIGHT:
            direction = NORTH
        elif movement == LEFT:
            direction = SOUTH
        else:
            center = center + [0, -1]
    time.sleep(0.3)
    return center, direction


def update_frontend(current_map, center, head):
    """To send messages to update the front-end
    Args:
        current_map (Numpy array): Current state of the exploration map
        exploredArea (int): Number of cells that have been explored
        center (list): Location of center of the robot
        head (list): Location of head of the robot
        start (list): Location of the starting point for the robot
        goal (list): Location of the finishing point for the robot
        elapsedTime (float): The time that has elapsed since exploration started
    """
    asyncio.set_event_loop(asyncio.new_event_loop())
    for key in clients:
        message = dict()
        # message['area'] = '%.2f' % (exploredArea)
        tempMap = current_map.copy()
        tempMap[START[0]-1: START[0]+2, START[1]-1: START[1]+2] = 3
        tempMap[GOAL[0]-1: GOAL[0]+2, GOAL[1]-1: GOAL[1]+2] = 4
        message['map'] = json.dumps(tempMap.astype(int).tolist())
        message['center'] = json.dumps(center.astype(int).tolist())
        message['head'] = json.dumps(head.astype(int).tolist())
        # message['time'] = '%.2f' % (elapsedTime)
        clients[key]['object'].write_message(json.dumps(message))


class RPi:
    def __init__(self):
        pass
    
    def keep_main(self):
        while True:
            time.sleep(0.5)

    def send(self, msg):
        if msg != 'E':
            encode_s = ''
            countF = 0
            i = 0
            while i < len(msg):
                if msg[i] == 'F':
                    countF = 1
                    i+=1
                    while i<len(msg) and msg[i]=='F':
                        countF+=1
                        i+=1
                    encode_s += 'F'+str(countF)
                else:
                    encode_s += msg[i]
                    i += 1                    
            msg = encode_s
        msg = '[a]' + msg + '\0'
        print(f'[SEND] {msg}')
    

    def receive(self):
        while True:
            msg = self.client.recv(2048).decode(FORMAT)
            if msg:
                print(f'[RECEIVED] {msg} from RPi')
                return msg

    def receive_manual(self):
        msg = input()
        if msg:
            print(f'[RECEIVED] {msg} from RPi')
            return msg

    def startlistener(self):
        print('going into start listener')
        while True:
            msg = self.receive_manual()
            split_data = msg.split(" ")
            if (split_data[0] == 'StartPoint'):
                start = np.asarray([int(split_data[1]), int(split_data[2])])
                robot = RealRobot(NORTH, start, self.send,
                                  self.receive_manual, update_frontend)
                exp = Exploration(robot, coverage=0.7)
                exp.start()
                print(f"start point:{split_data[1]}, {split_data[2]}")
            elif (split_data[0] == 'WayPoint'):
                global waypoint
                waypoint = np.asarray([split_data[1], split_data[2]])
                waypoint[0] = 19 - waypoint[0]
                print(f"way point:{waypoint}")
            elif (split_data[0] == 'FSP'):
                simu_map = map_from_file(SIMU_MAP_FILE)
                fsp = FastestPath(simu_map, START, GOAL, waypoint)
                fsp.run()
                movements = fsp.movements
                self.send(movements)
                self.FSPmovement(fsp, simu_map)

    def FSPmovement(self, fsp, map_):
        init_center, init_direction = START, NORTH
        update_frontend(map_, init_center, cal_head(
            init_center, init_direction))
        idx = 0

        while True:
            msg = self.receive_manual()
            # calculate center, head
            (c, d) = fsp.center_directions[idx]
            c = np.asarray(c)
            update_frontend(map_, c, self.cal_head(c, d))
            idx += 1
            if msg == FSP_DONE_MSG:
                break

    def cal_head(self, center, direction):
        if direction == NORTH:
            head = center + [-1, 0]
        elif direction == SOUTH:
            head = center + [1, 0]
        elif direction == WEST:
            head = center + [0, -1]
        else:
            head = center + [0, 1]
        return head

    def disconnect(self):
        self.send(DISCONNECT_MESSAGE)
        self.client


# def func1():
#     print("Starting communication with RPi")
#     rpi = RPi()
#     t1 = FuncThread(rpi.startlistener)
#     t1.start()
#     t1.join()

def func1():
    print("Starting communication with RPi")
    client_rpi = RPi()
    rt = threading.Thread(target=client_rpi.startlistener)
    rt.daemon = True
    rt.start()

settings = {
    'static_path': '/var/www/static/',
    # other settings
}


app = tornado.web.Application([
    (r"/", staticRequestHandler),
    (r'/websocket', WebSocketHandler),
    (r'/start', StartHandler),
    (r'/fsp', FSPHandler),
    (r'/(.*)', tornado.web.StaticFileHandler,
     {'path': os.path.join(os.path.dirname(__file__), "GUI")})
], **settings)


def func2():
    print("Starting communication with front-end")
    asyncio.set_event_loop(asyncio.new_event_loop())
    app.listen(8881)
    t1 = FuncThread(tornado.ioloop.IOLoop.instance().start)
    t1.start()
    t1.join()


class WebServer(threading.Thread):
    def run(self):
        print("Starting communication with front-end")
        asyncio.set_event_loop(asyncio.new_event_loop())
        app.listen(8881)
        tornado.ioloop.IOLoop.instance().start()



if __name__ == '__main__':
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    Thread(target=func1).start()
    WebServer().start()
    # Thread(target=func2).start()

