#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# [-0.6, 0, 0.6, 0, 0, 0.6, 0, -0.6]

# ROS
from email.charset import QP
import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from hyrecro.srv import *

# DATA TREATMENT
import pandas as pd
import numpy as np

qPositionMsg = JointState()
qTargetMsg = JointState()
DIVISION = 100
JOINT_NAMES =   ['q1','q2','q3','q4','q5','q6','q7','q8']
SWAP_LEG =      ['C', 'C', 'C', 'C', 'C', 'C', 'C', 'C']

def positionCallback(data):
    global qPositionMsg
    qPositionMsg = data
    return

def swap_leg():
    param = rospy.get_param('/hyrecro/fixed_leg')
    if param == 'A':
        rospy.set_param('/hyrecro/fixed_leg', 'B')
    elif param == 'B':
        rospy.set_param('/hyrecro/fixed_leg', 'A')
    else:
        print('Error with param for fixed leg')


def move_simulator():
    rospy.init_node('move_simulator', anonymous=True)
    rate = rospy.Rate(9)

    rospy.Subscriber("/hyrecro/joints_position", JointState, positionCallback, queue_size=1)

    move_joints_pub = rospy.Publisher('joints_tmp_target', JointState, queue_size=10)
    move_msg = JointState()
    move_msg.name = JOINT_NAMES
    rospy.Rate(0.2).sleep()

    print("------ STARTING THE MOVE ------ ")
    while(index < len(targets)):
        qTargetMsg = next_target()
        rospy.wait_for_message('/hyrecro/joints_position', JointState, 5)


        if qTargetMsg != SWAP_LEG:
            qTarget = qTargetMsg.position
            qTarget = [float(i) for i in qTarget]
            qTarget = np.array(qTarget)
            qPosition = np.array(qPositionMsg.position)
            # qTarget   = np.array(qTargetMsg.position)
            actual_leg = rospy.get_param("/hyrecro/fixed_leg")
            if actual_leg == 'B': qPosition = -qPosition

            if np.size(qTarget) > 0:
                print("Target:   ", qTarget)
                print("Position: ", qPosition)
                q_diff = qTarget -  qPosition
                delta_q = q_diff/DIVISION
                print("Delta:    ", delta_q)
                for i in range(0, DIVISION):
                    qPosition = qPosition + delta_q
                    print("Position: ", qPosition)
                    move_msg.position = qPosition
                    move_joints_pub.publish(move_msg)
                    rate.sleep()

            rospy.Rate(10).sleep()
        else:
            rospy.Rate(1).sleep()
            swap_leg()
            print("--- Leg Change ---")
            rospy.Rate(0.5).sleep()
    
    print('Trajectory completed')

def get_targets():
    # Check node is used correctly
    if len(sys.argv) != 2:
        print('¡¡¡ Wrong Usage !!!')
        print('Usage: rosrun hyrecro FILE_NAME.csv')
        rospy.signal_shutdown('Wrong node call')
    else:
        #Get path to the hyrecro pkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('hyrecro')
        trajectories_path = package_path  + '/trajectories'
        file_name = sys.argv[1]
        file_path = trajectories_path + '/' + file_name
        print(file_path)

    # Read file and get data as a np.array
    df = pd.read_csv(file_path)
    print(df)
    # targets = np.array(df.values.tolist())
    targets = df.values.tolist()

    return targets


def next_target():
    global index
    print('--- Getting next target ---')
    if targets[index] == SWAP_LEG:
        index += 1
        return SWAP_LEG
    else:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_NAMES
        msg.position = targets[index]
        index += 1
        return msg

if __name__ == '__main__':
    index = 0
    sign = 1
    np.set_printoptions(floatmode='fixed', precision=3, suppress=True)
    targets = get_targets()
    move_simulator()