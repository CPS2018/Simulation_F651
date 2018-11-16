'''Action controller for F651
 
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
roslib.load_manifest('simulation_control')
import rospy
import actionlib
import mavros_state
import time

from simulation_control.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal

from std_msgs.msg import Float32

if __name__ == '__main__':
    rospy.init_node('action_controller')
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Takeoff succeded")

    # Close legs
    long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
    long_grippers_client.wait_for_server()
    rospy.loginfo("Moving legs to pickup position")
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(0.5000000000))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Legs moved to pickup position")
    else:
        rospy.loginfo("Error with moving legs to pickup position")
    # /Close legs

    goto_position_goal.destination.pose.position.x = -105
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 3
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Arrived at search position.")

    rospy.loginfo("Searching...")
    detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
    detect_object_client.wait_for_server()
    detect_object_goal = detect_objectGoal()
    detect_object_client.send_goal(detect_object_goal)
    detect_object_client.wait_for_result()
    print(detect_object_client.get_result())

    rospy.loginfo("Going to detected position")
    goto_position_goal.destination.pose.position.x = detect_object_client.get_result().detected_position.pose.position.x
    goto_position_goal.destination.pose.position.y = detect_object_client.get_result().detected_position.pose.position.y
    goto_position_goal.destination.pose.position.z = detect_object_client.get_result().detected_position.pose.position.z
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Is at position (x,y,z)=({}, {}, {})".format(detect_object_client.get_result().detected_position.pose.position.x,
                                                               detect_object_client.get_result().detected_position.pose.position.y,
                                                               detect_object_client.get_result().detected_position.pose.position.z))
    
    rospy.loginfo("Descending on object")
    descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
    descend_on_object_client.wait_for_server()
    rospy.loginfo("Descending server started")
    descend_on_object_goal = descend_on_objectGoal()
    descend_on_objectGoal = 2.0
    descend_on_object_client.send_goal(descend_on_object_goal)
    descend_on_object_client.wait_for_result()
    if descend_on_object_client.get_result().position_reached.data:
        print("landing")
        mv_state.arm(False)
    else:
        rospy.loginfo("Couldnt land exiting")

    time.sleep(3)

    # Grip object
    short_grippers_client = actionlib.SimpleActionClient('short_grippers', short_grippersAction)
    short_grippers_client.wait_for_server()
    rospy.loginfo("Closing short grippers")
    short_grippers_goal = short_grippersGoal(grip_rad_goal=Float32(1.22000000000))
    short_grippers_client.send_goal(short_grippers_goal)
    short_grippers_client.wait_for_result()
    if short_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Short grippers closed")
    else:
        rospy.loginfo("Error when closing short grippers")
    # /Grip object

    time.sleep(3)

    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)

    time.sleep(1)

    rospy.loginfo("Lifting")
    goto_position_goal.destination.pose.position.x = -105
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 1.5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(1)

    # Close legs
    rospy.loginfo("Closing legs around object")
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(1.57000000000))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        print("Legs closed")
    else:
        rospy.loginfo("Error with closing legs")
    # /Close legs

    time.sleep(1)

    rospy.loginfo("Going to drop off position")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 105
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(3)

    rospy.loginfo("Deceding prior to drop off")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 105
    goto_position_goal.destination.pose.position.z = 0.5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(3)

    # Open legs
    rospy.loginfo("Opening legs around object")
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(0.00000000000))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        print("Legs opened")
    else:
        rospy.loginfo("Error with leg opening")
    # /Open legs

    time.sleep(1)

    # Release object
    rospy.loginfo("short_grippers")
    short_grippers_goal = short_grippersGoal(grip_rad_goal=Float32(0.00000000000))
    short_grippers_client.send_goal(short_grippers_goal)
    short_grippers_client.wait_for_result()
    if short_grippers_client.get_result().goal_reached.data:
        print("Grip position reached")
    else:
        rospy.loginfo("Grip position not reached")
    # /Release object

    time.sleep(3)

    rospy.loginfo("Going Home")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = -105
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    mv_state.land(0.0)
