#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, UC Regents
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of California nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
import rospy


class PidController(object):
    """
    Very simple PID controller class. Inspired by, and similar interface to,
    pid.cpp from pr2_controllers/control_toolbox
    """

    """
    Initialize values
    """
    kp = 0
    ki = 0
    kd = 0
    i_term = 0.0
    p_term = 0.0
    d_term = 0.0
    p_error_last = 0.0
    p_error = 0.0
    i_error = 0.0
    d_error = 0.0
    current_output = 0.0

    def __init__(self, kp, ki, kd, i_min=None, i_max=None, d_avg_max_count=1, debug=False):
        self.d_avg_max_count = d_avg_max_count
        self.set_pid_parameters(kp, ki, kd)
        self.i_max = i_max
        self.i_min = i_min
        self.debug = debug
        self.d_avg_filter = [0.0 for i in range(d_avg_max_count)]
        if self.kd > 0.0 or self.kd < 0.0:
            print self.kd
        
    def reset_pid(self):
        self.i_term = 0.0
        self.p_term = 0.0
        self.d_term = 0.0
        self.p_error_last = 0.0
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        self.current_output = 0.0
        self.d_avg_filter = [0.0 for i in range(self.d_avg_max_count)]

    def set_pid_parameters(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        self.reset_pid()
        
    def update(self, error, dt=0.0, error_dot=None):
        if dt == 0:
            output = 0.0
            rospy.logerr('dt == 0 should not happen ')
            return output

        #p-term calc
        self.p_error = error
        p_term = self.kp * self.p_error

        ##d-term calculation
        if error_dot is None:
            # TODO: finite differencing
            if dt > 0.0:
                self.d_error = (self.p_error - self.p_error_last) / dt
        else:
            self.d_error = error_dot
        d_term = self.kd * self.d_error
        self.d_avg_filter.append(d_term)
        if len(self.d_avg_filter) > self.d_avg_max_count:
            self.d_avg_filter.pop(0)
        d_term = sum(self.d_avg_filter) / len(self.d_avg_filter)

        if self.debug and self.kd != 0.0:
            print "d_term:   {0}".format(d_term)

        #PID CHANGE CRAZYFLIE fishy
        #if self.d_error < 0:
        #   self.d_error = 0
        #------------------

        #i-term calc
        self.i_error += self.p_error * dt

        if self.i_max and self.i_error > self.i_max:
            self.i_error = self.i_max
            # rospy.logwarn('I value is max %f', self.i_error)

        elif self.i_min and self.i_error < self.i_min:
            # rospy.logwarn('I value is min %f', self.i_error)
            self.i_error = self.i_min

        i_term = self.ki * self.i_error

        self.p_error_last = self.p_error

        #PID CHANGE CRAZYFLIE
        output = p_term + i_term + d_term
        #------
        #old
        #output = -p_term - i_term - d_term
        self.p_term, self.i_term, self.d_term = p_term, i_term, d_term # maybe useful for debugging
        self.current_output = output

        return output
    
    def get_current_cmd(self):
        return self.current_output


# class altitudeController(object):
 
#     def __init__(self):
#         rospy.init_node('pid', anonymous = True)
        
#         self.pub = rospy.Publisher('/pid/thrust/out', Float32)

#         KP = 60
#         KI = 40
#         KD = 60
#         Ilimit = 40
    
#         self.nominal_thrust = 80
#         self.max_thrust = 255

#         self.setpoint = 0.74;
#         self.lastCall = rospy.Time.now()
        
#         self.pid = PidController(KP, KI, KD, Ilimit)

   
#     def callback(self, data):
#         dt = 1

#         # dt = float(rospy.Time.now() - self.lastCall)
        
#         self.lastcall = rospy.Time.now()
#         error = data.range/100.0 - self.setpoint
#         thrust = (self.nominal_thrust + self.pid.update(error, dt))
#         self.pub.publish(thrust);
#         print thrust 

    
#     def listener(self):
#         rospy.Subscriber('range_data', Range, self.callback)
#         rospy.spin()    

# if __name__ == '__main__':
#     try:
#         conti = altitudeController()
#         conti.listener()
#     except rospy.ROSInterruptException: pass
