#!/usr/bin/env python

import roslib
import rospy
import actionlib
import simulation_control.msg
import time

from std_msgs.msg import Float32

class long_grippers_server():
    def __init__(self):

        #variables
        self.target_pos = Float32()

        #publishers
        self.grip_control = rospy.Publisher('/f550_amazing/longgrip_rad', Float32, queue_size=1)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.long_grippersResult()
        self.server = actionlib.SimpleActionServer('long_grippers',
                                                    simulation_control.msg.long_grippersAction,
                                                    execute_cb=self.execute_cb,
                                                    auto_start=False)
        self.server.start()

	# Main function
    def execute_cb(self, goal):
        self.grip_control.publish(goal.grip_rad_goal)
        time.sleep(3)
        self.result.goal_reached.data = True
        self.server.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('long_grippers_server')
    server = long_grippers_server()
    rospy.spin()
