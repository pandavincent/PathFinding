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
        m = np.loadtxt("map14.txt", delimiter=",")
        self.start_location = (9, 1)

        self.goal_location = (3, 9)
        #self.goal_location = (7, 3)
        #self.goal_location = (1, 1)
        #self.goal_location = (9, 9)
        #self.goal_location = (5, 1)

        self.path = aStar.a_star(m, self.start_location, self.goal_location)

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
                if math.sqrt(dx * dx + dy * dy) >= 0.215:
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
                if math.sqrt(dx * dx + dy * dy) >= 0.13:
                    break
            self.create.drive_direct(0, 0)



        if move == "left":
            print("turning left")
            target_theta = self.odometry.theta + (math.pi/2)
            while True:
                self.create.drive_direct(35, -35)
                self.sleep(0.001)
                if self.angle_adjust(target_theta - self.odometry.theta) <= 0.002:
                    break
            self.create.drive_direct(0, 0)



        if move == "right":
            print("turning right")
            target_theta = self.odometry.theta - (math.pi/2)
            while True:
                self.create.drive_direct(-35, 35)
                self.sleep(0.001)
                if self.angle_adjust(self.odometry.theta - target_theta) <= 0.002:
                    break
            self.create.drive_direct(0, 0)

    def run(self):


        # Start the robot
        self.create.start()
        self.create.safe()

        """
        # Algorithm Walkthrough
        # moving horizonal
            # if p_prev == p[0], move forward , pop

            # else if p_prev > p[0], turn left, move forward, 
            #change to vertical mode, pop

            # else if p_prev < p[0], turn right, move forward, 
            #change to vertical mode, pop

        # moving vertical
            # if p_prev == p[1], move forward, pop

            # esle if p_prev > p[1], turn left, move forward, 
            #change to horizontal mode, pop

            # else if p_prev < p[1], turn right, move forward, 
            #change to horizontal mode, pop
            
        """
        
        route = self.path
        print(route)
        # Defining mode
        first_node_x = route[0][0]
        first_node_y = route[0][1]

        second_node_x = route[1][0]
        second_node_y = route[1][1]

        mode = "horizontal"

        if (first_node_y == second_node_y):
            self.movement("left")
            mode = "vertical"
           
        # Get start node
        prev_x = route[0][0]
        prev_y = route[0][1]

        while True:
            state = self.create.update()
            t = self.time.time()
            if state is not None:
                while len(self.path) > 0:

                    if (mode == "horizontal"):

                        cur_x = route[0][0]
                        cur_y = route[0][1]

                        if (prev_x == cur_x):
                            self.movement("forward")
            
                        elif (prev_x > cur_x):
                            self.movement("left")
                            self.movement("step")
                            self.movement("forward")
                            mode = "vertical"

                        elif (prev_x < cur_x):
                            self.movement("right")
                            self.movement("step")
                            self.movement("forward")
                            mode = "vertical"

                        prev_y = cur_y
                        route.pop(0)

                    elif (mode == "vertical"):

                        cur_x = route[0][0]
                        cur_y = route[0][1]

                        if (prev_y == cur_y):
                            self.movement("forward")
            
                        elif (prev_y > cur_y):
                            self.movement("left")
                            self.movement("step")
                            self.movement("forward")
                 
                            mode = "horizontal"
                        elif (prev_y < cur_y):
                            self.movement("right")
                            self.movement("step")
                            self.movement("forward")
                            mode = "horizontal"

                        prev_x = cur_x
                        route.pop(0)
            
