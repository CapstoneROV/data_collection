#!/usr/bin/env python
# Script to navigate through a series of points using MAVLink
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
    
def get_initial_orientation(master):
    rospy.loginfo("Fetching initial orientation...")
    attitude = master.recv_match(type='ATTITUDE', blocking=True)
    if attitude:
        return attitude.roll, attitude.pitch, attitude.yaw
    else:
        rospy.logwarn("Failed to fetch initial orientation!")
        return 0, 0, 0
    
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
        if isclose(x_pos, x, abs_tol=2e-1) and isclose(y_pos, y, abs_tol=2e-1) and isclose(z_pos, z, abs_tol=2e-1):
            rospy.loginfo("Position reached!")
            break

def set_target_position(master, x, y, z, initial_position, initial_orientation):
    # Transform to x,y,z rpy origin
    x_rel, y_rel, z_rel = x + initial_position[0], y + initial_position[1], z + initial_position[2]
    # Transform to x,y,z rpy target
    x_rel, y_rel, z_rel = x_rel * math.cos(initial_orientation[2]) - y_rel * math.sin(initial_orientation[2]), x_rel * math.sin(initial_orientation[2]) + y_rel * math.cos(initial_orientation[2]), z_rel
    rospy.loginfo("Setting target local position to X: %f, Y: %f, Z: %f", x_rel, y_rel, z_rel)
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000, x_rel, y_rel, z_rel, 0, 0, 0, 0, 0, 0, 0, 0)

def navigate_through_points(master, points, initial_position, initial_orientation):
    for point in points:
        rospy.loginfo("Navigating to point: %s", str(point))
        set_target_position(master, *point, initial_position=initial_position, initial_orientation=initial_orientation);
        rospy.loginfo("Waiting to reach the target point...")
        rospy.sleep(15)  # Wait time may need adjustment based on vehicle speed and distance
        # spin_till_position_reached(master, *point)

def arm_and_wait(master):
    rospy.loginfo("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()

def disarm_and_wait(master):
    rospy.loginfo("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()

# Function to send mavlink message to bluerov2 to go to GUIDED mode
def set_guided_mode(master):
    # Use pymavlink to set to GUIDED mode
    # USE PYMAVLINK NOT MAVROS
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_GUIDED_ARMED,
        0, 0, 0, 0, 0, 0)  # MAV_MODE_GUIDED_ARMED = 4
    rospy.loginfo("Set mode to GUIDED")

def arm_and_set_mode(master, mode_name):
    # print(f"Arming vehicle and setting mode to {mode_name}...")
    master.arducopter_arm()
    master.motors_armed_wait()
    mode_id = master.mode_mapping()[mode_name]
    master.set_mode(mode_id)
    print("Vehicle is armed and mode is set!")
    while not master.wait_heartbeat().custom_mode == mode_id:
        master.set_mode(mode_name)

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
        
        
        arm_and_set_mode(master, 'GUIDED') # Set to GUIDED mode
        # Quickly force it to start at that point(or guided will push ROV into a dive)
        initial_position = get_initial_position(master)
        initial_orientation = get_initial_orientation(master)
        set_target_position(master, *initial_position, initial_position=initial_position, initial_orientation=initial_orientation)
        rospy.sleep(5)  # Wait for 5 seconds to stabilize
        rospy.loginfo("Initial position: %s", str(initial_position))
        navigate_through_points(master, points, initial_position, initial_orientation)
        disarm_and_wait(master)
    except rospy.ROSInterruptException:
        disarm_and_wait(master)  # Disarm if interrupted
        pass
