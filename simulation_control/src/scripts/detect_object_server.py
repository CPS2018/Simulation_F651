'''Object detection for F651
 
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
