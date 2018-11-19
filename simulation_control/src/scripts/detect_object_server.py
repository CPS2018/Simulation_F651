#!/usr/bin/env python
import rospy
import actionlib
import simulation_control.msg
'''Object detection for F651
  Copyright (C) 2018, CPS2018 Challenge by Team Halmstad. All rights reserved.
 '''
from geometry_msgs.msg import PoseStamped, Point
class detect_object_server():
    def __init__(self):

        #variables
        self.local_pose = PoseStamped()
        self.detected = False

        #publishers

        #subscribers
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.detect_objectResult()
        self.action_server = actionlib.SimpleActionServer('detect_object', simulation_control.msg.detect_objectAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()


    def execute_cb(self, goal):
        rospy.sleep(0.1)
        while not self.detected:
            self.rate.sleep()

        rospy.loginfo("Target Detected")
        self.result.detected_position = self.local_pose
        self.action_server.set_succeeded(self.result)


    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


    def _local_pose_callback(self, data):
        self.local_pose = data

if __name__ == '__main__':
    try:

        rospy.init_node('detect_object_server')
        detect_object_server()
    except rospy.ROSInterruptException:
        pass
