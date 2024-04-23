#!/usr/bin/env python
# Script to request and get RAW_IMU data from the BlueROV2, then store it in a numpy array as a ROS Node
import rospy
from pymavlink import mavutil
import numpy as np

def request_message_interval(master, frequency_hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
        1e6 / frequency_hz,
        0,
        0, 0, 0, 0
    )
    rospy.loginfo("Requested the message successfully.")

def get_raw_imu_data(master, save_name):
    message_index = 0
    data_array = np.empty((0, 3), int)
    try:
        while not rospy.is_shutdown():
            msg = master.recv_match(type='RAW_IMU', blocking=True, timeout=5)
            if msg:
                xacc, yacc, zacc = msg.to_dict()['xacc'], msg.to_dict()['yacc'], msg.to_dict()['zacc']
                rospy.logdebug("Message index: %d, Xacc: %d, Yacc: %d, Zacc: %d", message_index, xacc, yacc, zacc)
                data_point = np.array([[xacc, yacc, zacc]])
                data_array = np.append(data_array, data_point, axis=0)
                message_index += 1
    except Exception as e:
        rospy.loginfo("ERROR, JESUS TAKE THE WHEEL: %s", e)
    finally:
        # Save the data
        np.savetxt(save_name, data_array, delimiter=",")
        rospy.loginfo("Exiting & Saving data to %s.", save_name)

if __name__ == '__main__':
    rospy.init_node('blue_rov_imu_data_collector')#, log_level=rospy.DEBUG);

    # Read parameters
    connection_string = rospy.get_param('~mavlink_url', 'udpin:localhost:14550')
    data_save_path = rospy.get_param('~save_path', 'raw_imu_data.txt')
    message_frequency = rospy.get_param('~message_frequency', 4) # Frequency in Hz

    rospy.loginfo("Connecting to MAVLink on %s", connection_string)

    # Connect to master
    master = mavutil.mavlink_connection(connection_string)

    try:
        master.wait_heartbeat()
        rospy.loginfo("Heartbeat from system (system %u component %u)", master.target_system, master.target_component)
        request_message_interval(master, message_frequency)
        get_raw_imu_data(master, data_save_path)
    except Exception as e:
        rospy.loginfo("Whoops an error occured, whomp whomp: %s", e)
