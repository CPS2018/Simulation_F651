#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest, CommandBoolResponse, CommandTOL, CommandTOLRequest
import time
from tf.transformations import *
import numpy as np

class mavros_state():
    def __init__(self):
        ### subscriber ###

        # state subscriber
        self._rate_state = rospy.Rate(1)
        self.current_state = State()
        rospy.Subscriber('/mavros/state', State, self.current_state_callback)

        # wait until connection with FCU
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.Rate(20)
        print 'FCU connection successful'

    #callback function for state
    def current_state_callback(self, data):
        self.current_state = data


    #setmode for the drone
    #'OFFBOARD'
    def set_mode(self, mode):
        if not self.current_state.connected:
            print "No FCU connection"

        elif self.current_state.mode == mode:
            print "Already in " + mode + " mode"

        else:

            # wait for service
            rospy.wait_for_service("mavros/set_mode")

            # service client
            set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

            # set request object
            req = SetModeRequest()
            req.custom_mode = mode

            # zero time
            t0 = rospy.get_time()

            # check response
            while not rospy.is_shutdown() and (self.current_state.mode != req.custom_mode):
                if rospy.get_time() - t0 > 2.0:  # check every 5 seconds

                    try:
                        # request
                        set_mode.call(req)

                    except rospy.ServiceException, e:
                        print "Service did not process request: %s" % str(e)

                    t0 = rospy.get_time()

            print "Mode: " + self.current_state.mode + " established"


    #Arm the vehicle
    def arm(self, do_arming):

        if self.current_state.armed and do_arming:
            print "already armed"

        else:
            # wait for service
            rospy.wait_for_service("mavros/cmd/arming")

            # service client
            set_arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

            # set request object
            req = CommandBoolRequest()
            req.value = do_arming

            # zero time
            t0 = rospy.get_time()

            # check response
            if do_arming:
                while not rospy.is_shutdown() and not self.current_state.armed:
                    if rospy.get_time() - t0 > 2.0:  # check every 5 seconds

                        try:
                            # request
                            set_arm.call(req)

                        except rospy.ServiceException, e:
                            print "Service did not process request: %s" % str(e)

                        t0 = rospy.get_time()

                print "armed: ", self.current_state.armed

            else:
                while not rospy.is_shutdown() and self.current_state.armed:
                    if rospy.get_time() - t0 > 0.5:  # check every 5 seconds

                        try:
                            # request
                            set_arm.call(req)

                        except rospy.ServiceException, e:
                            print "Service did not process request: %s" % str(e)

                        t0 = rospy.get_time()

    def land(self, height):

        if not self.current_state.armed:

            print "not armed yet"

        else:

            # wait for service
            rospy.wait_for_service("mavros/cmd/land")

            # service client
            set_rq = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)

            # set request object
            req = CommandTOLRequest()
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = height
            req.min_pitch = 0.0

            # zero time
            t0 = rospy.get_time()

            # check response
            while self.current_state.armed:
                if rospy.get_time() - t0 > 5.0:  # check every 5 seconds

                    try:
                        # request
                        set_rq.call(req)

                    except rospy.ServiceException, e:
                        print "Service did not process request: %s" % str(e)

                    t0 = rospy.get_time()

            print "landed savely"

    def takeoff(self):

        if not self.current_state.armed:

            print "not armed yet"

        else:

            # wait for service
            rospy.wait_for_service("mavros/cmd/takeoff")

            # service client
            set_rq = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)

            # set request object
            req = CommandTOLRequest()
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 4.0
            req.min_pitch = 0.0

            # zero time
            t0 = rospy.get_time()

            # check response
            while self.current_state.armed:
                if rospy.get_time() - t0 > 5.0:  # check every 5 seconds

                    try:
                        # request
                        set_rq.call(req)

                    except rospy.ServiceException, e:
                        print "Service did not process request: %s" % str(e)

                    t0 = rospy.get_time()

            print "landed savely"
