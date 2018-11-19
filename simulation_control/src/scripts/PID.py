#!/usr/bin/env python
'''PID for F651
   Copyright (C) 2018, CPS2018 Challenge by Team Halmstad. All rights reserved.
'''
import rospy

class PID:
    def __init__(self, Kp=0.2, Ki=0.001, Kd=0.001, maxI=0.1, maxOut=3.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.target = 0
        self.output = 0
        self.error = 0
        self.maxI = maxI
        self.maxOut = maxOut
        self.reset()
        self.lastTime = rospy.get_time()

    def update(self, target, state, time):
        # u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        self.target = target
        self.state = state
        self.error = self.target - self.state

        self.time = time
        dTime = self.time - self.lastTime
        dError = self.error - self.lastError
        #print("DTime: {}, dError: {}".format(dTime, dError))

        p = self.error
        self.intError += self.error * dTime
        if (dTime > 0):
            d = dError / dTime
        else:
            d = 0
            rospy.logdebug("Change in time is zero.")
        # Make sure I does not exceed maximum windup
        if (self.maxI is not None and self.intError > self.maxI):
            i = self.maxI
        elif (self.maxI is not None and self.intError < -self.maxI):
            i = self.maxI
        else:
            i = self.intError

        # Remember last time and last error for next calculation
        self.lastTime = self.time
        self.lastError = self.lastError

        output = self.Kp * p + self.Ki * i + self.Kd * d
        # Make sure output does not exceed maximum
        if (self.maxOut is not None and output > self.maxOut):
            output = self.maxOut
        elif (self.maxOut is not None and output < -self.maxOut):
            output = -self.maxOut

        return output

    def setKp(self, Kp):
        self.Kp = Kp

    def setKi(self, Ki):
        self.Ki = Ki

    def setKd(self, Kd):
        self.Kd = Kd

    def setMaxI(self, maxI):
        self.maxI = maxI
    def setMaxO(self, MaxO):
        self.maxOut = MaxO

    def reset(self):
        self.target = 0.0
        self.error = 0.0
        self.state = 0.0
        self.intError = 0.0
        self.lastError = 0.0
        self.output = 0.0
