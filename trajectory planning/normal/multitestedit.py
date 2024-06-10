#!/usr/bin/env python
# one row grasp

import time
import argparse

# Utility methods
from utility import quat2eulerZYX
from utility import parse_pt_states
from utility import list2str

import sys
sys.path.insert(0, "/home/robot/motion/flexiv_rdk/lib_py")
import flexivrdk

def main():
    robot_ip = "192.168.2.100"            
    local_ip = "192.168.2.104"

    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # new
    safe_origin = [0.471448, -0.369169, 0.536714, 177.687, 0.176, 157.846]
    precatch = [0.487438, -0.071409, 0.536714, -0.896, -179.517, 47.301]
    catch = [0.487420, -0.071394, 0.466700, -0.896, -179.517, 47.304]
    up = [0.487438, -0.071409, 0.536714, -0.896, -179.517, 47.301]
    move_tube = [0.471448, -0.369169, 0.536714, 177.687, 0.176, 157.846]
    place_tube = [0.471448, -0.369169, 0.466266, 177.687, 0.176, 157.846]
    move_up = [0.471448, -0.369169, 0.536714, 177.687, 0.176, 157.846]
    
    dx = 0.009
    x_increment  = 0
    precatch[0] += x_increment
    catch[0] += x_increment
    up[0] += x_increment
    move_tube[0] += x_increment
    place_tube[0] += x_increment
    try:
        robot = flexivrdk.Robot(robot_ip, local_ip)
        robot_states = flexivrdk.RobotStates()

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            robot.clearFault()
            time.sleep(2)
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot
        log.info("Enabling robot ...")
        robot.enable()

        while not robot.isOperational():
            time.sleep(1)
        log.info("Robot is now operational")

        # primitive execution mode
        robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)
        gripper = flexivrdk.Gripper(robot)

        num = 1
        total = 5
        increment = 0.009

        log.info("MoveL:safe origin")
        robot.executePrimitive(
            f"MoveL(target={list2str(safe_origin)} WORLD WORLD_ORIGIN, maxVel=0.1)"
        )
        while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
            time.sleep(1)
        
        gripper.move(0.0125, 0.01, 30)
        time.sleep(2)
        while num < total:           
            log.info("MoveL:pre catch")
            robot.executePrimitive(
                f"MoveL(target={list2str(precatch)} WORLD WORLD_ORIGIN, maxVel=0.1)"
            )       
            while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
                time.sleep(1)

            log.info("MoveL:catch")
            robot.executePrimitive(
                f"MoveL(target={list2str(catch)} WORLD WORLD_ORIGIN, maxVel=0.07)"
            )
            while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
                time.sleep(1)
        
            log.info("Closing gripper")
            gripper.move(0.0088, 0.003, 45)
            time.sleep(2)
            
            log.info("MoveL:up")
            robot.executePrimitive(
                f"MoveL(target={list2str(up)} WORLD WORLD_ORIGIN, maxVel=0.08)"
            )
            while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
                time.sleep(1)
      
            log.info("Opening gripper")
            gripper.move(0.0125, 0.1, 20)
            time.sleep(1)

            num += 1
            precatch[1] -= increment
            catch[1] -= increment
            up[1] -= increment
            move_tube[1] -= increment
            place_tube[1] -= increment
            move_up[1] -= increment
            print("precatch[1]", precatch[1])
            print("num", num)

        robot.stop()

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()

