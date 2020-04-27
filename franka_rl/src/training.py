#!/usr/bin/env python


import rospy
import qlearn
import numpy as np
import rospkg
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import random
import matplotlib.pyplot as plt
import time


pub = rospy.Publisher('/franka/panda_joint7_controller/command', Float64, queue_size=100)
check = rospy.Publisher('/make_check', Bool, queue_size=100)

def deg2rad(angle):
    return angle*3.14/180

def rad2deg(angle):
    return angle*180/3.14


def reward_func(angle):
    results = [0.0, 1.57, 3.14, 4,71, 6.28]
    dist = []
    for i in results:
        a = abs(i - angle)
        dist.append(a)
    reward = -1*min(dist)
    return reward


def check_angle(angle):
    #0, 90, 180, 270, 360
    if (abs(angle) < 0.03):
        return True
    if (abs(angle - 1.57) < 0.03):
        return True
    if (abs(angle - 3.14) < 0.03):
        return True
    if (abs(angle - 4.71) < 0.03):
        return True
    if (abs(angle - 6.28) < 0.03):
        return True
    return False
def move(state, action):
    global pub
    global check
    delta = deg2rad(5)
    pos = None
    while pos is None:
        try:
            pos = rospy.wait_for_message('/franka/joint_states', JointState, timeout=5)
        except:
            pass
    angle = pos.position[8]
    done = False
    reward = -1 #wo reward function
    #reward = reward_func(angle)
    isDone = False
    msg = Float64()
    if (action == 0): #rotate clockwise
        msg.data = angle + delta
        pub.publish(msg)
    if (action == 1): #rotate anti clockwise
        msg.data = angle - delta
        pub.publish(msg)
    if (action == 2): #check
        if (check_angle(angle)):
            check_done = make_check(check)
            done = True
            reward = 10
        else:
            done = True
            reward = -10
    pos = None
    while pos is None:
        try:
            pos = rospy.wait_for_message('/franka/joint_states', JointState, timeout=5)
        except:
            pass
    angle = pos.position[8]
    state = round(rad2deg(angle))

    return state, reward, done

def make_check(publisher):
    msg = Bool()
    msg.data = True
    while publisher.get_num_connections() < 1:
        pass
    publisher.publish(msg)
    isDone = False
    while (not isDone):
        isDone = rospy.wait_for_message("/done_checking", Bool)
    print("Done initialization")
    return True

def reset(publisher):
    msg = Bool()
    msg.data = True
    while publisher.get_num_connections() < 1:
        pass
    publisher.publish(msg)
    isDone = False
    while (not isDone):
        isDone = rospy.wait_for_message("/done_init", Bool)
    print("Done initialization")
    return True


if __name__ == '__main__':
    rospy.init_node('training', anonymous=True, log_level=rospy.WARN)
    init = rospy.Publisher('/make_init', Bool, queue_size= 100)

    epsilon = 0.98
    decay = 0.95
    actions = range(3)
    qlearn = qlearn.QLearn(actions=range(3), alpha=0.2, gamma=0.8, epsilon=0.98)
    result = []
    for i in range(250):
        init_done = reset(init)
        pos = None
        while pos is None:
            try:
                pos = rospy.wait_for_message("/franka/joint_states", JointState)
            except:
                pass
        state = round(rad2deg(pos.position[8]))
        print("Episode #"+str(i))
        total_reward = 0
        for j in range(100):

            if epsilon > random.random():
                action = np.random.choice(actions)
            else:
                action = qlearn.chooseAction(state)

            nextState, reward, done = move(state, action)
            print("Action was "+str(action)+" reward we got "+str(reward)+" state we were "+str(state)+" state we got "+str(nextState))
            total_reward += reward
            qlearn.learn(state, nextState, action, reward)
            if done:
                break
            else:
                state = nextState
        if epsilon > 0.1:
            epsilon *= decay
        print("Total reward for " + str(i) + " episode is "+ str(total_reward))
        result.append(total_reward)
    k = np.arange(250)
    plt.figure(figsize = (8,6))
    plt.plot(k, result)
    plt.show()
