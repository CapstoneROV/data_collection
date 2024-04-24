#!/usr/bin/env python
# Script to navigate through a series of points using MAVLink, relative to the initial local position
import rospy
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

def calibrate_sensors(master):
    rospy.loginfo("Starting sensor calibration...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 1, 0, 0, 0, 0, 0, 0)  # Accel calibration
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 1, 0, 0, 0, 0, 0)  # Gyro calibration

def get_initial_position(master):
    rospy.loginfo("Fetching initial local position...")
    pos_NED = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    if pos_NED:
        return pos_NED.x, pos_NED.y, pos_NED.z
    else:
        rospy.logwarn("Failed to fetch initial position!")
        return 0, 0, 0  # Fallback to origin if no position is received

def isclose(a, b, abs_tol=0.0):
    return abs(a-b) <= abs_tol

def spin_till_position_reached(master, x, y, z):
    rospy.loginfo("Spinning till position is reached...")
    while not rospy.is_shutdown():
        pos_NED = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if not pos_NED:
            continue
        x_pos = pos_NED.x
        y_pos = pos_NED.y
        z_pos = pos_NED.z
        if isclose(x_pos, x, abs_tol=1e-5) and isclose(y_pos, y, abs_tol=1e-5) and isclose(z_pos, z, abs_tol=1e-5):
            rospy.loginfo("Position reached!")
            break

def set_target_position(master, x, y, z, use_global, initial_position):
    x_rel, y_rel, z_rel = x - initial_position[0], y - initial_position[1], z - initial_position[2]
    rospy.loginfo("Setting target local position to X: %f, Y: %f, Z: %f", x_rel, y_rel, z_rel)
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000, x_rel, y_rel, z_rel, 0, 0, 0, 0, 0, 0, 0, 0)

def navigate_through_points(master, points, initial_position):
    for point in points:
        rospy.loginfo("Navigating to point: %s", str(point))
        set_target_position(master, *point, False, initial_position)
        rospy.loginfo("Waiting to reach the target point...")
        rospy.sleep(50)  # Wait time may need adjustment based on vehicle speed and distance

def arm_and_wait(master):
    rospy.loginfo("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()

def disarm_and_wait(master):
    rospy.loginfo("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()

if __name__ == '__main__':
    try:
        rospy.init_node('mavlink_navigation_node', anonymous=True)
        connection_url = rospy.get_param('~connection_url', 'udpin:0.0.0.0:14550')
        points = rospy.get_param('~points', [[0, 0, -10]])  # Default to one point
        calibrate = rospy.get_param('~calibrate', False)

        master = mavutil.mavlink_connection(connection_url)
        global boot_time
        boot_time = time.time()
        master.wait_heartbeat()
        rospy.loginfo("Connected to MAVLink on %s", connection_url)
        if calibrate:
            calibrate_sensors(master)

        initial_position = get_initial_position(master)
        arm_and_wait(master)
        rospy.sleep(5)
        navigate_through_points(master, points, initial_position)
        disarm_and_wait(master)

    except rospy.ROSInterruptException:
        disarm_and_wait(master)  # Disarm if interrupted
        pass
