'''Object Descend for F651
 
  Copyright (C) 2018, CPS2018 Challenge by Team Halmstad. All rights reserved.
 
  BSD license:
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of
     contributors to this software may be used to endorse or promote
     products derived from this software without specific prior written
     permission.
 
  THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 '''
#!/usr/bin/env python
import rospy
import actionlib
import simulation_control.msg
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String

class descend_on_object_server():
    def __init__(self):

        #variables
        self.local_pose = PoseStamped()
        self.des_pose = PoseStamped()
        self.object_pose = Point()

        #publishers
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)
        self.vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)

        #subscribers
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.descend_on_objectResult()
        self.action_server = actionlib.SimpleActionServer('descend_on_object',
                                                            simulation_control.msg.descend_on_objectAction,
                                                             execute_cb=self.execute_cb,
                                                              auto_start=False)
        self.last_object_pose = Point()
        self.action_server.start()


    def execute_cb(self, goal):
        rospy.loginfo("Starting to descend")
        self.mode_control.publish('velctr')
        rospy.sleep(0.1)

        while self.local_pose.pose.position.z > 1.0:
            self.rate.sleep()
            print("x = ",self.local_pose.pose.position.x)
            print("y = ",self.local_pose.pose.position.y)
            print("z = ",self.local_pose.pose.position.z)
            rospy.sleep(0.2)


            if self.detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
                self.last_object_pose = self.object_pose
                self.des_pose.pose.position.x = self.object_pose.x
                self.des_pose.pose.position.y = self.object_pose.y
                self.des_pose.pose.position.z = self.local_pose.pose.position.z
                self.vel_control.publish(self.des_pose)
                rospy.loginfo("Centering...")
                while not self.target_reached:
                    rospy.sleep(2)

            elif self.detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                self.des_pose.pose.position.x = 0
                self.des_pose.pose.position.y = 0
                self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.9
                rospy.loginfo("Descending...")
                self.vel_control.publish(self.des_pose)
                while not self.target_reached:
                    rospy.sleep(2)

        while self.local_pose.pose.position.z < 1.0 and self.local_pose.pose.position.z > 0.1:
            self.rate.sleep()
            print("x = ",self.local_pose.pose.position.x)
            print("y = ",self.local_pose.pose.position.y)
            print("z = ",self.local_pose.pose.position.z)
            rospy.sleep(0.2)
 
            if self.detected and (abs(self.object_pose.x) > 0.05 or abs(self.object_pose.y) > 0.05):
                self.last_object_pose = self.object_pose
                self.des_pose.pose.position.x = self.object_pose.x
                self.des_pose.pose.position.y = self.object_pose.y
                self.vel_control.publish(self.des_pose)
                rospy.loginfo("Centering...")
                while not self.target_reached:
                    rospy.sleep(2)

            elif self.detected and abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                self.des_pose.pose.position.x = 0
                self.des_pose.pose.position.y = 0
                self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.1
                rospy.loginfo("Descending...")
                self.vel_control.publish(self.des_pose)
                while not self.target_reached:
                    rospy.sleep(2)

        print("Landing")
        self.vel_control.publish(self.des_pose)
        self.rate.sleep()
        self.result.position_reached.data = True
        self.action_server.set_succeeded(self.result)

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

    def _local_pose_callback(self, data):
        self.local_pose = data

    def distance_reached_cb(self, data):
        self.target_reached = data.data

if __name__ == '__main__':
    try:

        rospy.init_node('descend_on_object_server')
        descend_on_object_server()
    except rospy.ROSInterruptException:
        pass
