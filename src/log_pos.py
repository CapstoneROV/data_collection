#!/usr/bin/env python
# Script to request and get LOCAL_POSITION_NED data from the BlueROV2, then store it in a numpy array as a ROS Node
import rospy
from pymavlink import mavutil
import numpy as np

def request_message_interval(master, frequency_hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        1e6 / frequency_hz,
        0,
        0, 0, 0, 0
    )
    rospy.loginfo("Requested the message successfully.")

def get_local_position_ned_data(master, save_name):
    message_index = 0
    data_array = np.empty((0, 4), int)  # Assuming we're storing x, y, z, and time_boot_ms
    try:
        while not rospy.is_shutdown():
            msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
            if msg:
                x, y, z, time_boot_ms = msg.to_dict()['x'], msg.to_dict()['y'], msg.to_dict()['z'], msg.to_dict()['time_boot_ms']
                rospy.logdebug("Message index: %d, X: %f, Y: %f, Z: %f, Timestamp: %d", message_index, x, y, z, time_boot_ms)
                data_point = np.array([[x, y, z, time_boot_ms]])
                data_array = np.append(data_array, data_point, axis=0)
                message_index += 1
    except Exception as e:
        rospy.loginfo("ERROR, JESUS TAKE THE WHEEL: %s", e)
    finally:
        # Save the data
        np.savetxt(save_name, data_array, delimiter=",")
        rospy.loginfo("Exiting & Saving data to %s.", save_name)

if __name__ == '__main__':
    rospy.init_node('blue_rov_position_data_collector', log_level=rospy.DEBUG);

    # Read parameters
    connection_string = rospy.get_param('~mavlink_url', 'udpin:localhost:14550')
    data_save_path = rospy.get_param('~save_path', 'local_position_ned_data.txt')
    message_frequency = rospy.get_param('~message_frequency', 4) # Frequency in Hz

    rospy.loginfo("Connecting to MAVLink on %s", connection_string)

    # Connect to master
    master = mavutil.mavlink_connection(connection_string)

    try:
        master.wait_heartbeat()
        rospy.loginfo("Heartbeat from system (system %u component %u)", master.target_system, master.target_component)
        request_message_interval(master, message_frequency)
        get_local_position_ned_data(master, data_save_path)
    except Exception as e:
        rospy.loginfo("Whoops an error occurred, whomp whomp: %s", e)
