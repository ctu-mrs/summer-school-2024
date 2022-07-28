#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import UInt8
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandBoolRequest 
from random import randint, uniform,random
import signal
import os

if len(sys.argv) < 3:
    print("Missing UAV's name provided as script's argument.\n") 
    sys.exit()

uav_names = []

if len(sys.argv) >= 2: 
    
    for k in range(1, len(sys.argv)):
        uav_names.append(sys.argv[k])

takeoff_allowed = [False] * len(uav_names)

def callbackReadyToTakeoff(msg, uav_idx):
    rospy.loginfo_throttle_identical(5.0, "Takeoff of %s allowed.", uav_names[uav_idx])
    takeoff_allowed[uav_idx] = True

def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0) 

if __name__ == "__main__":
    
    signal.signal(signal.SIGINT, signal_handler) # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")
    rospy.init_node('takeoff_manager', anonymous=True)

    rospy.loginfo("Initializing subscribers.")

    for k in range(len(uav_names)):
        rospy.Subscriber("/" + uav_names[k] + "/mrim_state_machine/ready_to_takeoff", UInt8, callbackReadyToTakeoff, k)

    rospy.loginfo("Waiting for UAVs' services.")
    arming_srv_list = []
    set_offboard_srv_list = []

    for uav_name in uav_names:
        rospy.wait_for_service('/' + uav_name + '/mavros/cmd/arming')
        arming_srv_list.append(rospy.ServiceProxy('/' + uav_name + '/mavros/cmd/arming', CommandBool))
        rospy.wait_for_service('/' + uav_name + '/mavros/set_mode')
        set_offboard_srv_list.append(rospy.ServiceProxy('/' + uav_name + '/mavros/set_mode', SetMode))

    rospy.loginfo("ROS Service clients initialized.")
    armed = [False] * len(uav_names)
    offboard = [False] * len(uav_names)

    while True: 

        if all(takeoff_allowed): 

            for k in range(len(uav_names)):

                bool_req = CommandBoolRequest()
                bool_req.value = True
                rospy.loginfo("Calling arming service of %s.", str(uav_names[k])) 
                armed[k] = arming_srv_list[k](bool_req)
                if not armed[k]:
                    rospy.loginfo("Arming service call of %s failed.", str(uav_names[k])) 

            rospy.Rate(0.5).sleep()

            if all(armed):
                for k in range(len(uav_names)):
                    set_mode_req = SetModeRequest()
                    set_mode_req.base_mode = 0
                    set_mode_req.custom_mode = 'offboard'
                    rospy.loginfo("Calling offboard service of %s.", str(uav_names[k]))
                    offboard[k] = set_offboard_srv_list[k](set_mode_req)
                    if not offboard[k]:
                        rospy.loginfo("Offboard service call of %s failed.", str(uav_names[k]))

            if all(offboard):
                rospy.loginfo("Offboard mode of all UAVs successfully set.")
                break

        else: 
            rospy.loginfo_throttle(5.0, "Takeoff is not allowed. Waiting for ready to takeoff message.")

        rospy.Rate(0.2).sleep()
