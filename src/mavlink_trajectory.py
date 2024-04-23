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
    
def spin_till_position_reached(master, x, y, z, use_global):
    rospy.loginfo("Spinning till position is reached...")
    while not rospy.is_shutdown():
            if use_global:
                position_data = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if not position_data:
                    continue
                lat = position_data.lat / 1e7
                lon = position_data.lon / 1e7
                alt = position_data.alt / 1000
                if isclose(lat, x, abs_tol=1e-5) and isclose(lon, y, abs_tol=1e-5) and isclose(alt, z, abs_tol=1e-5):
                    rospy.loginfo("Position reached!")
                    break
            else:
                pos_NED = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
                if not pos_NED:
                    continue
                x_pos = pos_NED.x
                y_pos = pos_NED.y
                z_pos = pos_NED.z
                if isclose(x_pos, x, abs_tol=1e-5) and isclose(y_pos, y, abs_tol=1e-5) and isclose(z_pos, z, abs_tol=1e-5):
                    rospy.loginfo("Position reached!")
                    break

def set_target_position(master, x, y, z, use_global):
    if use_global:
        rospy.loginfo("Setting target global position to Latitude: %f, Longitude: %f, Altitude: %f", x, y, z)
        master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - boot_time)),
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)
    else:
        rospy.loginfo("Setting target local position to X: %f, Y: %f, Z: %f", x, y, z)
        master.mav.set_position_target_local_ned_send(
            int(1e3 * (time.time() - boot_time)),
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)

def navigate_through_points(master, points, use_global):
    for point in points:
        rospy.loginfo("Navigating to point: %s", str(point))
        set_target_position(master, *point, use_global=use_global)
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
        use_global_coordinates = rospy.get_param('~use_global_coordinates', False)
        calibrate = rospy.get_param('~calibrate', False)

        master = mavutil.mavlink_connection(connection_url)
        global boot_time
        boot_time = time.time()
        master.wait_heartbeat()
        rospy.loginfo("Connected to MAVLink on %s", connection_url)
        if calibrate:
            calibrate_sensors(master)

        arm_and_wait(master)
        rospy.sleep(5)
        navigate_through_points(master, points, use_global_coordinates)
        disarm_and_wait(master)

    except rospy.ROSInterruptException:
        disarm_and_wait(master)  # Disarm if interrupted
        pass
