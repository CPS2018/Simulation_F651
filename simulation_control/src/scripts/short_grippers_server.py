'''Short Grippers for F651
 
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

import roslib
import rospy
import actionlib
import simulation_control.msg
import time

from std_msgs.msg import Float32

class short_grippers_server():
    def __init__(self):

        #variables
        self.target_pos = Float32()

        #publishers
        self.grip_control = rospy.Publisher('/f550_amazing/grip_rad', Float32, queue_size=1)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.short_grippersResult()
        self.server = actionlib.SimpleActionServer('short_grippers',
                                                    simulation_control.msg.short_grippersAction,
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
    rospy.init_node('short_grippers_server')
    server = short_grippers_server()
    rospy.spin()
