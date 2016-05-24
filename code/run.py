"""
Actual helper script to execute code.
It takes care of proper error handling (e.g. if you press CTRL+C) and the difference between
running code on the robot vs. in simulation.

Usage:
  python3 run.py --sim lab1 [for simulation]
  python3 run.py lab1 [to run on a robot]
"""


import sys
import argparse
import importlib

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", help="run specified module")
    parser.add_argument("--sim", help="Run using VREP simulation", action="store_true")
    args = parser.parse_args()
    clientID = None
    if args.sim:
        from simulation.vrep import vrep as vrep
        vrep.simxFinish(-1)  # just in case, close all opened connections
        clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP

        # enable the synchronous mode on the client:
        vrep.simxSynchronous(clientID, True)

        from simulation import Create2Vrep, Sonar, Servo, TimeHelper
        create = Create2Vrep(clientID)
        timeHelper = TimeHelper(clientID)
        sonar = Sonar(clientID)
        servo = Servo(clientID)
    else:
        from robot import Create2Driver, Sonar, Servo
        import time
        create = Create2Driver("/dev/ttyS2", 87)
        timeHelper = time
        sonar = Sonar(104)
        servo = Servo(0)

    try:
        mod = importlib.import_module(args.run)
        Run = getattr(mod, "Run")
        r = Run(create, timeHelper, sonar, servo)
        r.run()
    except KeyboardInterrupt:
        pass

    create.drive_direct(0, 0)
    create.digits_leds_ascii(bytes("    ", encoding='ascii'))
    create.stop()
    if args.sim:
        # stop the simulation:
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        # close the connection to V-REP:
        vrep.simxFinish(clientID)

    # quit
    sys.exit()
