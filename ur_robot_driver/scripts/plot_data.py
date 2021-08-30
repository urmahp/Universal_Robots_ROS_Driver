"""
script used for plotting data. 
"""
import matplotlib.pyplot as plt
import numpy as np
import csv
from os.path import expanduser
import statistics

# joints names
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

home = expanduser("~")

def plot_actual_vs_target(target, actual, start_time=0, end_time=1000):
    time = []
    actual_shoulder_pan_joint = []
    actual_shoulder_lift_joint = []
    actual_elbow_joint = []
    actual_wrist_1_joint = []
    actual_wrist_2_joint = []
    actual_wrist_3_joint = []

    target_shoulder_pan_joint = []
    target_shoulder_lift_joint = []
    target_elbow_joint = []
    target_wrist_1_joint = []
    target_wrist_2_joint = []
    target_wrist_3_joint = []

    with open(target, "r") as f:
        plots = csv.reader(f, delimiter=',')
        for row in plots:
            timer = float(row[0])
            if timer > start_time and timer < end_time:
                time.append(float(row[0]))
                target_shoulder_pan_joint.append(float(row[1]))
                target_shoulder_lift_joint.append(float(row[2]))
                target_elbow_joint.append(float(row[3]))
                target_wrist_1_joint.append(float(row[4]))
                target_wrist_2_joint.append(float(row[5]))
                target_wrist_3_joint.append(float(row[6]))

    with open(actual, "r") as f:
        plots = csv.reader(f, delimiter=',')
        for row in plots:
            timer = float(row[0])
            if timer > start_time and timer < end_time:
                actual_shoulder_pan_joint.append(float(row[1]))
                actual_shoulder_lift_joint.append(float(row[2]))
                actual_elbow_joint.append(float(row[3]))
                actual_wrist_1_joint.append(float(row[4]))
                actual_wrist_2_joint.append(float(row[5]))
                actual_wrist_3_joint.append(float(row[6]))

    plt.plot(time, target_shoulder_pan_joint, label="target_" + joints[0])
    plt.plot(time, target_shoulder_lift_joint, label="target_" + joints[1])
    plt.plot(time, target_elbow_joint, label="target_" + joints[2])
    #plt.plot(time, target_wrist_1_joint, label="target_" + joints[3])
    #plt.plot(time, target_wrist_2_joint, label="target_" + joints[4])
    #plt.plot(time, target_wrist_3_joint, label="target_" + joints[5])

    plt.plot(time, actual_shoulder_pan_joint, label="actual_" + joints[0])
    plt.plot(time, actual_shoulder_lift_joint, label="actual_" + joints[1])
    plt.plot(time, actual_elbow_joint, label="actual_" + joints[2])
    #plt.plot(time, actual_wrist_1_joint, label="actual_" + joints[3])
    #plt.plot(time, actual_wrist_2_joint, label="actual_" + joints[4])
    #plt.plot(time, actual_wrist_3_joint, label="actual_" + joints[5])

if __name__ == "__main__":
    target_joint = "{}/ros_driver_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/data/target_joint.txt".format(home)
    actual_joint = "{}/ros_driver_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/data/actual_joint.txt".format(home)

    plt.figure(1)
    plt.xlabel('[s]')
    plt.ylabel('[rad]')
    plt.title('Joint positions over time actual as feedback')
    plot_actual_vs_target(target_joint, actual_joint)
    plt.legend()

    plt.show()