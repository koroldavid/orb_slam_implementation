#! /usr/bin/python3

# Node for extracting images from a rosbag in the format needed for ORB_SLAM3.
# Referenced https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/

import csv
import rospy
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import subprocess
import sys
import os

bridge = CvBridge()
# folder name for dataset
IMU0_PATH = None
# Ensure images have time correspondance.
timestamps = []
cam1_index = -1


def get_cam0(msg):
    global timestamps
    # cam0 will control the time.
    time = msg.header.stamp
    timestamps.append(time)
    # write the next timestamp to the file.
    timestamps_file.write(str(time)+"\n")

def imu0_write_header():
    with open(IMU0_PATH, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]",
                        "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]"])

def get_imu0(msg):
    global cam1_index
    # use the time from cam0.
    cam1_index += 1
    # wait if necessary until cam0 has come in for this timestep.
    time = None
    while len(timestamps)-1 < cam1_index:
        rospy.sleep(0.05)
    try:
        time = timestamps[cam1_index]
    except:
        rospy.logerr("cam1 unable to sync with cam0 at timestamp "+str(len(timestamps))+", index "+str(cam1_index))

    # Extract angular velocity (rad/s)
    w_x = msg.angular_velocity.x
    w_y = msg.angular_velocity.y
    w_z = msg.angular_velocity.z

    # Extract linear acceleration (m/s²)
    a_x = msg.linear_acceleration.x
    a_y = msg.linear_acceleration.y
    a_z = msg.linear_acceleration.z

    with open(IMU0_PATH, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([time, w_x, w_y, w_z, a_x, a_y, a_z])


def run_bash_cmd(command):
    # run something on the command line.
    process = subprocess.Popen(command.split())
    output, error = process.communicate()


def main():
    global timestamps_file, IMU0_PATH
    rospy.init_node('imu_conversion_node')

    DATASET_NAME = sys.argv[1]
    cam0_topic = "/left/downsample_raw"
    imu_topic = "/imu/imu/data"
    
    # create the folder that images will be saved to.
    DIR_PATH = os.path.join(os.path.expanduser('~'), DATASET_NAME)
    IMU0_PATH = os.path.join(DIR_PATH, 'images', 'mav0', 'imu0', 'data.csv')

    run_bash_cmd("mkdir -p "+DIR_PATH+"/images/mav0/imu0")
    rospy.loginfo("IMU will be saved to "+DIR_PATH+".\nYou can now run 'rosbag play ROSBAG_NAME.bag' in a new terminal.\nClose this node with Ctrl+C when your rosbag has finished playing.")
    # Init file for timestamps.
    timestamps_file = open(DIR_PATH+"/timestamps.txt", "w")
    
    # Subscribe to the image streams.
    imu0_write_header()
    rospy.Subscriber(cam0_topic, Image, get_cam0)
    rospy.Subscriber(imu_topic, Imu, get_imu0)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass