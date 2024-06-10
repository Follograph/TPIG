#!/usr/bin/env python
import time
import argparse
import pyzed.sl as sl
import numpy as np
import cv2
import math


# Utility methods
from utility import quat2eulerZYX
from utility import parse_pt_states
from utility import list2str

import sys
sys.path.insert(0, "/home/robot/motion/flexiv_rdk/lib_py")
import flexivrdk
# fmt: on

def main():
    robot_ip = "192.168.2.100"            
    local_ip = "192.168.2.104"

    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    safe_origin = [0.471448, -0.369169, 0.536714, 177.687, 0.176, 157.846]
    photo_pose = [0.436954, -0.171356, 0.516743, 179.042, 14.506, 179.717]
    begin_pose = [0.436954, -0.171356, 0.536743, 177.687, 0.176, 179.717]

    pre_catch = [0.487438, -0.071409, 0.536714, -0.896, -179.517, 47.301]
    catch = [0.487420, -0.071394, 0.466700, -0.896, -179.517, 47.304]
    up = [0.487438, -0.071409, 0.536714, -0.896, -179.517, 47.301]
    move_tube = [0.471448, -0.369169, 0.536714, 177.687, 0.176, 157.846]
    place_tube = [0.471448, -0.369169, 0.461266, 177.687, 0.176, 157.846]
    move_up = [0.471448, -0.369169, 0.536714, 177.687, 0.176, 157.846]

    x_increment  = 0
    pre_catch[0] += x_increment
    catch[0] += x_increment
    up[0] += x_increment
    move_tube[0] += x_increment
    place_tube[0] += x_increment
    move_up[0] += x_increment


    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use ULTRA depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.depth_minimum_distance = 0.3

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : " + repr(status) + ". Exit program.")
        exit(1)

    image = sl.Mat()
    runtime = sl.RuntimeParameters(enable_fill_mode = True)
    
    
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

        num = 0
        total = 2
        increment = 0.009
        
        log.info("MoveL:safe origin")
        robot.executePrimitive(
            f"MoveL(target={list2str(safe_origin)} WORLD WORLD_ORIGIN, maxVel=0.1)"
        )
        while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
            time.sleep(1)
        gripper.move(0.0125, 0.01, 30)
        time.sleep(1)

        log.info("MoveL:photo")
        robot.executePrimitive(
            f"MoveL(target={list2str(photo_pose)} WORLD WORLD_ORIGIN, maxVel=0.1)"
        )
        while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
            time.sleep(1)
        time.sleep(1)
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.RIGHT)
            img = image.get_data()
            cv2.imwrite('tube_zed2.jpg', img)
            zed.close() # Close the camera
        time.sleep(6)
        print('''
        0
        []
        normal pattern''')

        log.info("MoveL:begin_origin")
        robot.executePrimitive(
            f"MoveL(target={list2str(begin_pose)} WORLD WORLD_ORIGIN, maxVel=0.07)"
        )
        while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
            time.sleep(1)
            
        
        while num < total:
            log.info("MoveL:pre catch")
            robot.executePrimitive(
                f"MoveL(target={list2str(pre_catch)} WORLD WORLD_ORIGIN, maxVel=0.1)"
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

            log.info("MoveL:move tube")
            robot.executePrimitive(
                f"MoveL(target={list2str(move_tube)} WORLD WORLD_ORIGIN, maxVel=0.1)"
            )
            while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
                time.sleep(1)

            log.info("MoveL:place tube")
            robot.executePrimitive(
                f"MoveL(target={list2str(place_tube)} WORLD WORLD_ORIGIN, maxVel=0.08)"
            )
            while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
                time.sleep(1)
      
            log.info("Opening gripper")
            gripper.move(0.0125, 0.1, 20)
            time.sleep(1)

            log.info("MoveL:move up")
            robot.executePrimitive(
                f"MoveL(target={list2str(move_up)} WORLD WORLD_ORIGIN, maxVel=0.1)"
            )
            while parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1":
                time.sleep(1)

            num += 1
            pre_catch[1] -= increment
            catch[1] -= increment
            up[1] -= increment
            move_tube[1] -= increment
            place_tube[1] -= increment
            move_up[1] -= increment
            print("pre_catch[1]", pre_catch[1])
            print("num", num)

        robot.stop()

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()

