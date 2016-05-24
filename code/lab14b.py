import create2
import odometry
import math
import aStar
import numpy as np
import matplotlib.pyplot as plt

class Run:
    def __init__(self, create, time, sonar, servo):
        self.create = create
        self.time = time
        self.sonar = sonar
        self.servo = servo
        self.odometry = odometry.Odometry()

        # get path from A* path finding algorithm
        self.m = np.loadtxt("map14.txt", delimiter=",")
        self.start_location = (9, 1)
        
        # Without dynamic obstacles
        #self.goal_location = (3, 9)
        #self.goal_location = (9, 9)
        #self.goal_location = (7, 3)

        # With dynamic obstacles
        self.goal_location = (2, 1)
        #self.goal_location = (1, 1)

        self.path = aStar.a_star(self.m, self.start_location, self.goal_location)

        # robot initial orientation
        self.orientation = "east"
        self.theta = 0

    def sleep(self, time_in_sec, dt = 0):
        start = self.time.time()
        last_update = None
        while True:
            state = self.create.update()
            t = self.time.time()
            if state is not None:
                if last_update == None or t - last_update >= dt:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                
                    last_update = t
            if start + time_in_sec <= t:
                    break

    def angle_adjust(self, angle):
        if angle > math.pi:
            return angle - 2 * math.pi
        elif angle < -math.pi:
            return 2 * math.pi  + angle
        return angle

    def movement(self, move):
        base_speed = 100

        goal_theta = -math.pi / 2.0

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        
        if move == "forward":
            print("moving forward")
            x = self.odometry.x
            y = self.odometry.y
            while True:
                self.create.drive_direct(100, 100)
                self.sleep(0.001)
                dx = self.odometry.x - x
                dy = self.odometry.y - y
                #0.215
                if math.sqrt(dx * dx + dy * dy) >= 0.3:
                    break
            self.create.drive_direct(0, 0)

        if move == "step":
            print("steping forward")
            x = self.odometry.x
            y = self.odometry.y
            while True:
                self.create.drive_direct(100, 100)
                self.sleep(0.001)
                dx = self.odometry.x - x
                dy = self.odometry.y - y
                if math.sqrt(dx * dx + dy * dy) >= 0.02:
                    break
            self.create.drive_direct(0, 0)
            

        if move == "left":
            print("turning left")
            target_theta = self.theta + (math.pi/2)
            self.theta = target_theta
            while True:
                self.create.drive_direct(35, -35)
                self.sleep(0.001)
                if self.angle_adjust(target_theta - self.odometry.theta) <= 0.002:
                    break
            self.create.drive_direct(0, 0)


        if move == "right":
            print("turning right")
            target_theta = self.theta - (math.pi/2)
            self.theta = target_theta
            while True:
                self.create.drive_direct(-35, 35)
                self.sleep(0.001)
                if self.angle_adjust(self.odometry.theta - target_theta) <= 0.002:
                    break
            self.create.drive_direct(0, 0)

        if move == "reverse":
            print("reversing")
            target_theta = self.theta - (math.pi)
            self.theta = target_theta
            while True:
                self.create.drive_direct(-35, 35)
                self.sleep(0.001)
                if self.angle_adjust(self.odometry.theta - target_theta) <= 0.002:
                    break
            self.create.drive_direct(0, 0)

    def adjust_direction(self, move):
        # Start from North
        if move == "NtoW":
            print("doing NtoW")
            self.movement("left")

        if move == "NtoE":
            print("doing NtoE")
            self.movement("right")

        if move == "NtoS":
            print("doing NtoS")
            self.movement("reverse")

        # Start from East
        if move == "EtoN":
            print("doing EtoN")
            self.movement("left")

        if move == "EtoS":
            print("doing EtoS")
            self.movement("right")

        if move == "EtoW":
            print("doing EtoW")
            self.movement("reverse")

        # Start from South
        if move == "StoE":
            print("doing StoE")
            self.movement("left")

        if move == "StoW":
            print("doing StoW")
            self.movement("right")

        if move == "StoN":
            print("doing StoN")
            self.movement("reverse")

        # Start from West
        if move == "WtoS":
            print("doing WtoS")
            self.movement("left")

        if move == "WtoN":
            print("doing WtoN")
            self.movement("right")

        if move == "WtoE":
            print("doing WtoE")
            self.movement("reverse")


    def run(self):

        # Start the robot
        self.create.start()
        self.create.safe()
        
        self.travel()

    def travel(self):
        # Get route
        print("path is {}".format(self.path))
        route = self.path
        #print(route)

        # Defining mode
        first_node_x = route[0][0]
        first_node_y = route[0][1]

        second_node_x = route[1][0]
        second_node_y = route[1][1]

        temp = ""

        if (first_node_x == second_node_x):
            print("initial mode is horizontal")
            # make point east
            if (first_node_y < second_node_y):

                if (self.orientation == "north"):
                    self.adjust_direction("NtoE")
                if (self.orientation == "south"):
                    self.adjust_direction("StoE")
                if (self.orientation == "west"):
                    self.adjust_direction("WtoE")

                self.orientation = "east"


            # make point west
            if (first_node_y > second_node_y):

                if (self.orientation == "north"):
                    self.adjust_direction("NtoW")
                if (self.orientation == "south"):
                    self.adjust_direction("StoW")
                if (self.orientation == "east"):
                    self.adjust_direction("WtoE")

                self.orientation = "west"


            mode = "horizontal"

        if (first_node_y == second_node_y):
            print("initial mode is vertical")
            # make point south
            if (first_node_x < second_node_x):

                if (self.orientation == "north"):
                    self.adjust_direction("NtoS")
                if (self.orientation == "east"):
                    self.adjust_direction("EtoS")
                if (self.orientation == "west"):
                    self.adjust_direction("WtoS")

                self.orientation = "south"
                    

            # make point notrth
            if (first_node_x > second_node_x):

                if (self.orientation == "south"):
                    self.adjust_direction("StoN")
                if (self.orientation == "east"):
                    self.adjust_direction("EtoN")
                if (self.orientation == "west"):
                    self.adjust_direction("WtoN")

                self.orientation = "north"

            mode = "vertical"
           
        # Get start node
        prev_x = route[0][0]
        prev_y = route[0][1]
        self.path.pop(0)

        while True:
            state = self.create.update()
            t = self.time.time()
            if state is not None:

                while len(self.path) > 0:

                    if (mode == "horizontal"):

                        cur_x = route[0][0]
                        cur_y = route[0][1]

                        if (prev_x == cur_x):
                            self.checkWall(prev_x, prev_y, cur_x, cur_y)
                            self.movement("forward")
            
                        elif (prev_x > cur_x):
                            if (self.orientation == "east"):
                                self.adjust_direction("EtoN")
                            if (self.orientation == "west"):
                                self.adjust_direction("WtoN")

                            self.orientation = "north"

                            self.movement("step")

                            self.checkWall(prev_x, prev_y, cur_x, cur_y)
                            self.movement("forward")
                            mode = "vertical"

                        elif (prev_x < cur_x):
                            if (self.orientation == "east"):
                                self.adjust_direction("EtoS")
                            if (self.orientation == "west"):
                                self.adjust_direction("WtoS")

                            self.orientation = "south"
                            self.movement("step")

                            self.checkWall(prev_x, prev_y, cur_x, cur_y)
                            self.movement("forward")
                            mode = "vertical"

                        prev_y = cur_y
                        route.pop(0)

                    elif (mode == "vertical"):

                        cur_x = route[0][0]
                        cur_y = route[0][1]

                        if (prev_y == cur_y):
                            self.checkWall(prev_x, prev_y, cur_x, cur_y)
                            self.movement("forward")
            
                        elif (prev_y > cur_y):
                            if (self.orientation == "north"):
                                self.adjust_direction("NtoW")
                            if (self.orientation == "south"):
                                self.adjust_direction("StoW")

                            self.orientation = "west"
                            self.movement("step")

                            self.checkWall(prev_x, prev_y, cur_x, cur_y)
                            self.movement("forward")
                 
                            mode = "horizontal"
                        elif (prev_y < cur_y):
                            if (self.orientation == "north"):
                                self.adjust_direction("NtoE")
                            if (self.orientation == "south"):
                                self.adjust_direction("StoE")

                            self.orientation = "east"
                            self.movement("step")

                            self.checkWall(prev_x, prev_y, cur_x, cur_y)
                            self.movement("forward")
                            mode = "horizontal"

                        prev_x = cur_x
                        route.pop(0)


    def checkWall(self, prev_x, prev_y, cur_x, cur_y):
        # Get distance to the wall
        distance = self.sonar.get_distance()
        print("sonar reading is {}".format(distance))

        if distance < 0.3:
            # Recalculate path
            print("wall detected")
            print("prev_x is {} and prev_y is {}".format(prev_x, prev_y))
            print("cur_x is {} and cur_y is {}".format(cur_x, cur_y))
            self.m[cur_x][cur_y] = 1
            self.start_location = (prev_x, prev_y)
            self.path = aStar.a_star(self.m, self.start_location, self.goal_location)
            self.travel()
     
