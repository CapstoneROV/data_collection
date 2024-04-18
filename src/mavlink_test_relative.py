#!/usr/bin/env python
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

def fetch_and_print_position(master):
    position_data = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if position_data:
        lat = position_data.lat / 1e7
        lon = position_data.lon / 1e7
        alt = position_data.alt / 1000
        rospy.loginfo("Current Position - Latitude: %f, Longitude: %f, Altitude: %fm", lat, lon, alt)

def set_target_position(master, x, y, z):
    rospy.loginfo("Setting target pos to X: %f, Y: %f, Z: %f", x, y, z);
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0);

def set_target_attitude(master, roll, pitch, yaw):
    q = QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)])
    rospy.loginfo("Setting target attitude - roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # convert to milliseconds
        master.target_system, master.target_component,
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        q, 0, 0, 0, 0);

def arm_and_wait(master):
    rospy.loginfo("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()

def disarm_and_wait(master):
    rospy.loginfo("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()

def main():
    rospy.init_node('mavlink_interface_node', anonymous=True)
    connection_url = rospy.get_param('~connection_url', 'tcp:localhost:5760')
    x_pos = rospy.get_param('~x_pos', 0)
    y_pos = rospy.get_param('~y_pos', 0)
    z_pos = rospy.get_param('~z_pos', -10)  # Depth in meters
    roll = rospy.get_param('~roll', 0)
    pitch = rospy.get_param('~pitch', 0)
    yaw = rospy.get_param('~yaw', 0)

    master = mavutil.mavlink_connection(connection_url)
    global boot_time
    boot_time = time.time()
    master.wait_heartbeat()
    rospy.loginfo("Connected to MAVLink on %s", connection_url)

    calibrate_sensors(master)
    arm_and_wait(master)
    set_target_position(master, x_pos, y_pos, z_pos)
    set_target_attitude(master, roll, pitch, yaw)
    fetch_and_print_position(master)
    rospy.sleep(30)  # Maintain the settings for a duration
    disarm_and_wait(master)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
