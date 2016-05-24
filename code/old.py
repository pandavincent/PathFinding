import create2
import odometry
import pid_controller
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
        self.pidWall = pid_controller.PIDController(1000, 0, 50, [0, 0], [-150, 150], is_angle=False)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-150, 150], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-150, 150], is_angle=False)
        # get path from A* path finding algorithm
        m = np.loadtxt("map14.txt", delimiter=",")
        self.path = aStar.a_star(m, (9, 1), (7, 5))


    def turn_right(self, speed, angle, dt):
        self.create.drive_direct(-speed, speed)
        self.sleep(math.pi * create2.Specs.WheelDistanceInMM / (360 / angle) / speed, dt)

    def sleep(self, time_in_sec, dt = 0):
        start = self.time.time()
        last_update = None
        while True:
            state = self.create.update()
            t = self.time.time()
            if state is not None:
                if last_update == None or t - last_update >= dt:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    # don't include the groundTruth data if you run it on the robot!
                    groundTruth = self.create.sim_get_position()
                    #print("{},{},{},{},{}".format(self.odometry.x, self.odometry.y, self.odometry.theta * 180 / math.pi, groundTruth[0], groundTruth[1]))
                    last_update = t
            if start + time_in_sec <= t:
                break


    def run(self):

        print(self.path)
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        

        base_speed = 300
        #goal_wayPoints = [[1.0, 0.0], [3.0, 2.0], [2.5, 2.0], [0.0, 1.5], [0.0, 0.0]]
        #goal_wayPoints = [[0.6, 0.0], [1.2, 0], [1.2, 0.5], [1.8, 0.5], [1.8, 1.0], [2.4, 1.0]]
                         
        # [(9, 1), (9, 2), (9, 3), (9, 4), (9, 5), (8, 5), (7, 5))
        # # minus 9
        # [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (1, 5), (2, 5)]
        # # reverse
        # [(1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (5, 1), (5, 2)]
        # # scale by 1:0.24
        # [(0.24, 0), (0.48, 0), (0.72, 0), (0.96, 0), (1.2, 0), (1.2, 0.24), ()]  
     
        # minus 9
        mp = []
        for p in self.path:
            n0 = abs(p[0] - 9)
            n1 = p[1]
            mp.append((n0,n1))

        # reverse
        rp = []
        for p in mp:
            n0 = p[1]
            n1 = p[0]
            rp.append((n0,n1))

        # scale
        sp = []
        for p in rp:
            n0 = p[0]*0.247
            n1 = p[1]*0.247
            sp.append((n0,n1))

        # get rid first item
        sp.pop(0)
        print(sp)

        goal_wayPoints = sp 
        goal_wallDistance = 0.2
        positionTolerance = 0.05
        thetaTolerance = 0.01

        for wayPoint in goal_wayPoints:

            goal_x = wayPoint[0]
            goal_y = wayPoint[1]
            nearWallFirstTime = True
            wallTolerance = 0
            thetaWall = 0
            wallIterations = 0
            waypointIterations = 0
            new_origin_x = 0
            new_origin_y = 0
            while True:
                state = self.create.update()
                sonarDistance = self.sonar.get_distance()
                if (state is not None) and (sonarDistance is not None):
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

                    # Robot is near waypoint
                    if ((self.odometry.x < goal_x + positionTolerance and self.odometry.x > goal_x - positionTolerance) and
                            (self.odometry.y < goal_y + positionTolerance and self.odometry.y > goal_y - positionTolerance)):
                        break

                    # # Robot is near wall
                    if sonarDistance < goal_wallDistance + wallTolerance:
                        print("robot is near a wall")
                        # Replaning use A*
                        waypointIterations = 0
                        if nearWallFirstTime:
                            new_origin_x = self.odometry.x
                            new_origin_y = self.odometry.y
                            #thetaWall = math.atan2((self.odometry.y),(self.odometry.x))
                            self.turn_right(base_speed, 70, 0)
                            self.servo.go_to(70)
                            nearWallFirstTime = False
                            wallTolerance = 0.2
                            wallIterations = 0
                            continue

                        # Theta is relative to the new origin coordinates
                        current_theta = math.atan2(self.odometry.y - new_origin_y, self.odometry.x - new_origin_x)
                        print("ThetaWall: {}, OdTheta: {}".format(0.0, current_theta))

                        if ((-math.pi + thetaTolerance > current_theta) or
                                (math.pi - thetaTolerance < current_theta) or
                                ((thetaTolerance > current_theta) and (0.0 < current_theta)) or
                                (-thetaTolerance < current_theta) and (0.0 > current_theta)):
                            self.create.drive_direct(0,0)
                            self.servo.go_to(0)
                            self.sleep(2)
                            nearWallFirstTime = True
                            wallTolerance = 0
                            continue

                        #print("Sonar Distance: {}".format(sonarDistance))
                        output_wall = self.pidWall.update(sonarDistance, goal_wallDistance, self.time.time())
                        self.create.drive_direct(int(base_speed - output_wall), int(base_speed + output_wall))
                        self.time.sleep(0.01)
                        wallIterations = wallIterations + 1
                        continue

                    # This makes sure that the robot doesn't stop wall following too son
                    if waypointIterations > 20:
                        waypointIterations = 0
                        wallIterations = 0
                        nearWallFirstTime = True
                        new_origin_x = 0
                        new_origin_y = 0
                        self.servo.go_to(0)

                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    goal_distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, goal_distance, self.time.time())

                    #print("Goals: [{}, {}, {}, {}]\n".format(goal_x, goal_y, goal_theta, goal_distance))
                    #print("Odometry: [{},{},{}]\n".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

                    waypointIterations = waypointIterations + 1