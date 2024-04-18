# """
# Example of how to set target depth in depth hold mode with pymavlink
# """

import time
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase


def calibrate_sensors():
    # Accel calibration
    print("Starting accelerometer calibration...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # Command
        0,  # Confirmation
        1,  # Param 1: 1 to trigger accelerometer calibration
        0,  # Param 2: 0 to ignore
        0,  # Param 3: 0 to ignore
        0,  # Param 4: 0 to ignore
        0,  # Param 5: 0 to ignore
        0,  # Param 6: 0 to ignore
        0   # Param 7: 0 to ignore
    )
    
    # Gyro calibration
    print("Starting gyro calibration...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # Command
        0,  # Confirmation
        0,  # Param 1: 0 to ignore
        1,  # Param 2: 1 to trigger gyro calibration
        0,  # Param 3: 0 to ignore
        0,  # Param 4: 0 to ignore
        0,  # Param 5: 0 to ignore
        0,  # Param 6: 0 to ignore
        0   # Param 7: 0 to ignore
    )

def fetch_and_print_position(master):
    """
    Fetches and prints the current global position of the vehicle.
    """
    position_data = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if position_data:
        # Decode the message
        position_data = position_data.to_dict()
        lat = position_data['lat'] / 1e7  # Convert from 1E7 scaled to degrees
        lon = position_data['lon'] / 1e7  # Convert from 1E7 scaled to degrees
        alt = position_data['alt'] / 1000  # Convert from mm to meters
        print(f"Current Position - Latitude: {lat}, Longitude: {lon}, Altitude: {alt}m")


def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        type_mask=( # ignore everything except z position
            # mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            # mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), x=0.01, y=0.01, z=-5, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# arm ArduSub autopilot and wait until confirmed
# calibrate_sensors()
# fetch_and_print_position(master)
# master.arducopter_arm()
# master.motors_armed_wait()

# set the desired operating mode
# DEPTH_HOLD = 'GUIDED'
# DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
# while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
#     master.set_mode(DEPTH_HOLD)

# x = 10
# z = 10 
# master.arducopter_arm()
# master.motors_armed_wait()
# master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,
#                         master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b010111111000), x, 0, z, 0, 0, 0, 0, 0, 0, 0, 0))


# while 1:
#     msg = master.recv_match(
#         type='LOCAL_POSITION_NED', blocking=True)
#     print(msg)

# master.arducopter_disarm()
# master.motors_disarmed_wait()

DEPTH_HOLD = 'GUIDED'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]

while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

x_target_distance = -10  # The desired x distance to move
z_target_depth = -10  # The desired depth

master.arducopter_arm()
master.motors_armed_wait()

# Get initial x-position after arming
initial_message = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
initial_x = initial_message.x

# Set the target position
master.mav.set_position_target_local_ned_send(
    time_boot_ms=10,
    target_system=master.target_system,
    target_component=master.target_component,
    coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    type_mask=int(0b0000111111111000),
    x=initial_x + x_target_distance,
    y=0,
    z=z_target_depth,
    vx=0,
    vy=0,
    vz=0,
    afx=0,
    afy=0,
    afz=0,
    yaw=0,
    yaw_rate=0
)


while True:
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    current_x = msg.x
    print(f"Current x: {current_x}, Initial x: {initial_x}, Target x: {initial_x + x_target_distance}")

    # Check if the ROV has reached or exceeded the target x distance
    if (x_target_distance > 0 and current_x >= initial_x + x_target_distance) or \
       (x_target_distance < 0 and current_x <= initial_x + x_target_distance):
        print("Target distance reached. Disarming vehicle.")
        break
        
master.arducopter_disarm()
master.motors_disarmed_wait()
