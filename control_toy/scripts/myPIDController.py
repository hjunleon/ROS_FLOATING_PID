#!/usr/bin/env python
#Ensure you are using the correct python version.

import time
import math
import rospy
from std_msgs.msg import Float64, Bool

"""
You need to complete the following 3 steps to get this script to work:
1. Add Missing Subscribers
2. Implement your PID Controller
3. Tune your PID Controller
"""

#declaration of global variables
global depth
global setpoint
setpoint = 0
depth = 0

#Declaration of callback functions
def SetPointCallback(msg):
    global setpoint
    setpoint = msg.data

def PIDControlCallback(msg):
    global depth
    depth = msg.data

#Main
if __name__ == '__main__':
    rospy.init_node('PID_Controller')
    pub_thrust = rospy.Publisher('/simulator/thruster', Float64, queue_size=10)
    """
    ==================================================================================
    TODO: Implement your Subscribers to '/simulator/setpoint' & '/simulator/depth' here.
    Hint: They are 'Float64' type.
    ==================================================================================
    """

    rospy.Subscriber("/simulator/setpoint", Float64, SetPointCallback)
    rospy.Subscriber("/simulator/depth", Float64, PIDControlCallback)
    #Step 1: Add Missing Subscribers here.
    
    print("Time to rock and roll")

    """
    Our assumption: Distance is measured in metres and our update interval is a constant 50 ms.
    For fun, assume that the max speed of the thruster is 8 m/s (0.4 metres per 50 ms) and that
    our vehicle sinks at a constant 2 m/s (0.1 metres per 50 ms).
    """
    #Step 3: Tune your PID Controller
    KP = 1      #What Proportional value is good? 0.2, 1
    KI = 0.001      #What integral value is good? 0.0005
    KD = 0.175      #What differential value is good? 0.1 #too little, 0.175, 0.2, 0.3, 0.5
    upwardBias = 0.1    #Is a bias necessary? 0.05, 0.1

    #Declare all the variables that you need here!
    iteration_time = 0.05 #Assume constant update intervals.
    previous_error = 0.0
    previous_depth = -1
    isErrorPositive = True
    thrust = 0.0
    previous_set_point = 0.0
    ep = 0.000001
    cur_i = 0.0
    cur_i_threshold = 750#750, 1000, 2000 seems too much, 500 may have too little movement

    while not rospy.is_shutdown():

        """
        ==================================================================================
        TODO: Implement your PID Control code here.
        Hint: You may need to declare a few more variables.
        The result of your PID Controller should be stored in the variable named "thrust".
        Note that a +ve thrust is upwards while -ve thrust is downwards.
        ==================================================================================
        """
        #Step 2: Add your PID Controller code here.
        isMovingUp = False
        if previous_depth != -1:
            if depth - previous_depth < 0:
                isMovingUp = True
            else:
                isMovingUp = False
        previous_depth = depth
        depth_change = math.fabs(previous_depth - depth) + ep
        print("depth_change: ",depth_change)
        current_error = depth - setpoint
        print("current_error: ", current_error)
        change_in_error = current_error - previous_error
        print("change_in_error: ", change_in_error)
        # if (current_error < previous_error):
        #     change_in_error = -change_in_error
        cur_d = change_in_error / iteration_time
        print("cur_d: ",cur_d)
        cur_i += (current_error)# * 0.01)
        print("cur_i: ",cur_i)
        print("isMovingUp: ",isMovingUp)
        p_error = current_error
        if isMovingUp and current_error > 0:
            print("Adding damp")
            p_error /= 10  #cur_d *= 4#p_error /= 2  #2, 4 , 6, 10, 20(a little low)
        else:
            p_error /= 2
        #cur_i *= upwardBias

        if cur_i > cur_i_threshold:
            cur_i = cur_i_threshold
        elif cur_i < -cur_i_threshold:
            cur_i = -cur_i_threshold

        err = KP * p_error + KI * cur_i + KD * cur_d
        # if isMovingUp and current_error > 0:
        #     err *= upwardBias
        # if current_error > 0: #and current_error < 150:
        #     err = -1
        # if current_error > 0:
        #     err *= (upwardBias - 0.2)
        previous_set_point = setpoint

        
        thrust += err
        previous_error = current_error
        print("err: ",err)
        """
        Just for fun,
        Assume that the equipment is experiencing a constant acceleration due to gravity,
        hence, we do not need to worry about moving downwards.
        To avoid wearing out the hypothetical thruster and save energy,
        we can stop the motor instead of propelling the equipment downwards.
        """
        print("Unmaxed thrust: ",thrust)
        if thrust > 0.4:
            thrust = 0.4
        elif thrust < 0.0:
            thrust = 0.0

        pub_thrust.publish(Float64(thrust))
        print("Depth: {0:.4f} thrust: {1:.4f} setpoint: {2:.4f}".format(round(depth, 5), round(thrust, 5),round(setpoint, 5)))
        time.sleep(iteration_time)

    print("Shutting Down PID Controller...")
    print("Done")
