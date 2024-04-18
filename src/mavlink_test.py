#!/usr/bin/python
# REQUIRED MAVROS
# Create script to send mavlink message to bluerov2 to go to GUIDED mode
# and then to go to a specific position
import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

# Create a class to send mavlink message to bluerov2
class MavlinkTest:
    def __init__(self):
        # Create a publisher to send mavlink message to bluerov2
        self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        # Create a subscriber to get the state of bluerov2
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        # Create a publisher to send position message to bluerov2
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Create a variable to store the state of bluerov2
        self.current_state = State()
        # Create a variable to store the position of bluerov2
        self.pose = PoseStamped()

    # Function to send mavlink message to bluerov2 to go to GUIDED mode
    def set_guided_mode(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            set_mode = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            set_mode(True)
            print("Set mode to GUIDED")
        except rospy.ServiceException as e:
            print("Service call failed: %s",e);

    # Function to send mavlink message to bluerov2 to go to a specific position
    def set_position(self):
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = -50
        for i in range(100):
            self.local_pos_pub.publish(self.pose)
            time.sleep(0.1)
        print("Set position to (0, 0, -50)")

    # Function to get the state of bluerov2
    def state_cb(self, msg):
        self.current_state = msg

    # Function to send mavlink message to bluerov2 to go to GUIDED mode
    def set_guided_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_mode(custom_mode="GUIDED")
            print("Set mode to GUIDED")
        except rospy.ServiceException as e:
            print("Service call failed: %s", e)

    # Function to send mavlink message
    def send_mavlink(self):
        # Send mavlink message to bluerov2 to go to GUIDED mode
        self.set_guided_mode()
        # Send mavlink message to bluerov2 to go to a specific position
        self.set_position()

# Main function
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('mavlink_test', anonymous=True)
    # Create an object of the class MavlinkTest
    mavlink_test = MavlinkTest()
    # Send mavlink message
    mavlink_test.send_mavlink()
    # Spin
    rospy.spin()