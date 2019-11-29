#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from nano_drone.msg import sendRC
from std_msgs.msg import Bool
import traceback

class Joy2RCin(object):
    """Joy2RCin ROS Node"""
    def __init__(self):
        # Initialize the Node
        rospy.init_node("Joy2RCin")
        self.msg = sendRC()
        self.msg.channels[0] = 1500
        self.msg.channels[1] = 1500
        self.msg.channels[2] = 1500
        self.msg.channels[3] = 1000
        self.arm = False
        
        # Setup the Joy topic subscription
        self.joy_subscriber = rospy.Subscriber("joy", Joy, self.handleJoyMessage, queue_size=1)

        # Setup the Arm topic publisher
        self.arm_pub = rospy.Publisher("/pi_drone/arm", Bool, queue_size=1)
        
        # Setup the Twist topic publisher
        self.rc_publisher = rospy.Publisher("/pi_drone/RC_in", sendRC, queue_size=1)
        rate = rospy.Rate(200) # 200hz

        while not rospy.is_shutdown():
           self.rc_publisher.publish(self.msg)
           rate.sleep()

    def handleJoyMessage(self, data):
        """Handles incoming Joy messages"""
        self.msg.channels[0] = 1500 - (int)(data.axes[2] * 500)
        self.msg.channels[1] = 1500 + (int)(data.axes[5] * 500)
        self.msg.channels[3] = 1500 + (int)(data.axes[1] * 500)
        self.msg.channels[2] = 1500 - (int)(data.axes[0] * 500)

        if(data.buttons[5] == 1):
            if not self.arm:
               self.arm = True
               self.arm_pub.publish(self.arm)
            
            else:
               self.msg.channels[4] = 1000
               self.arm = False
               self.arm_pub.publish(self.arm)


###  If Main  ###
if __name__ == '__main__':
    try:
       Joy2RCin()
    except:
        rospy.logerr("Unhandled Exception in the Joy2RCin"+
                     " Node:+\n"+traceback.format_exc())

