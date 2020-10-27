import tornado.web
import tornado.ioloop
import tornado.websocket
import asyncio
import sys
import os
import json
import time
from algo.constants import *
from algo.exploration import Exploration
from algo.mapmethod import map_from_file
from algo.robot import *

clients = dict()

class WebSocketHandler(tornado.websocket.WebSocketHandler):

    """Handles web-socket requests from the front-end to receive/send messages

    Attributes:
        id (string): id string from GET request
    """

    def open(self):
        """Open a web socket for communication
        """
        self.id = self.get_argument("Id")
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

def startExploration():
    simu_map = map_from_file(SIMU_MAP_FILE)
    robot = SimuRobot(NORTH, START, simu_map)
    exploration = Exploration(robot,time_limit=2)
    start_time = time.time()
    end_time = start_time + exploration.time_limit
    
    trajectory = ''
    while not exploration.check_completed():
        update(exploration.robot.current_map)
        trajectory += exploration.move()
        if exploration.time_limit:
            if time.time() > end_time:
                print('Time is over.')
                print(trajectory)
                return
    print(exploration.robot.current_map)
    print('Congrats! Exploration done.')



def update(current_map):
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
    for key in clients:
        message = dict()
        # message['area'] = '%.2f' % (exploredArea)
        tempMap = current_map.copy()
        # tempMap[start[0]-1: start[0]+2, start[1]-1: start[1]+2] = 3
        # tempMap[goal[0]-1: goal[0]+2, goal[1]-1: goal[1]+2] = 4
        message['map'] = json.dumps(tempMap.astype(int).tolist())
        # message['center'] = json.dumps(center.astype(int).tolist())
        # message['head'] = json.dumps(head.astype(int).tolist())
        # message['time'] = '%.2f' % (elapsedTime)
        clients[key]['object'].write_message(json.dumps(message))


if __name__ == "__main__":
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    settings = {
    'static_path': '/var/www/static/',
    # other settings
    }


    app = tornado.web.Application([
        (r"/", staticRequestHandler),
        (r'/websocket', WebSocketHandler),
        (r'/start', StartHandler),
        (r'/(.*)', tornado.web.StaticFileHandler, {'path': os.path.join(os.path.dirname(__file__), "GUI")})
    ])

    app.listen(8881)
    print("I'm listening on port 8881")
    tornado.ioloop.IOLoop.current().start()