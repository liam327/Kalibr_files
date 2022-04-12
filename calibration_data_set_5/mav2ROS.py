#!/usr/bin/env python3
from os import times

#from cv2 import data
from genpy.rostime import Time
# from genpy import Time
import rosbag
import argparse
import csv
import cv2
import cv_bridge
import numpy as np

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu, Image

parser = argparse.ArgumentParser("Convert an ardupilot VIO dataset into a rosbag.")
parser.add_argument("imu", metavar='i', help="The file containing the IMU data.")
parser.add_argument("video", metavar='v', help="The file or folder containing the video data.")
parser.add_argument("stamps", metavar='s', help="The file containing the time stamps of each video frame.")
parser.add_argument("--use-video", action="store_true", help="use this flag if the dataset records video instead of frames.")
parser.add_argument("--ned", action='store_true', help="Use NED(North East Down) instead of default ENU(East North Up).")
args = parser.parse_args()

def ned2enu(position):
    n = position[0]
    e = position[1]
    u = - position[2]

    return [e, n, u]

def lat2radius(lat):
    equatorial_radius = 6378.1370e3 # metres
    polar_radius = 6356.7523e3 # metres

    c = np.cos(lat * np.pi / 180.0)
    s = np.sin(lat * np.pi / 180.0)

    radius = (equatorial_radius**2 * c)**2 + (polar_radius**2 * s)**2
    radius = radius / ((equatorial_radius * c)**2 + (polar_radius * s)**2)
    radius = np.sqrt(radius)
    return float(radius)

def gps2xyz(gps_origin, gps):
    # The origin defines the frame NED using a local approximation
    
    local_radius = lat2radius(gps_origin[0])

    nor = float(np.sin((gps[0]- gps_origin[0]) * np.pi / 180.0) * local_radius)
    eas = float(np.sin((gps[1]- gps_origin[1]) * np.pi / 180.0) * local_radius)
    down = gps_origin[2] - gps[2]

    return [nor, eas, down]

# Open all the files
bag_file = rosbag.Bag("mav_vio.bag", 'w')

print("Writing IMU data...")
with open(args.imu, 'r') as imu_file:
    imu_reader = csv.reader(imu_file)
    next(imu_reader) # skip header
    seq_counter = 1
    for row in imu_reader:
        ts = float(row[0])
        gyr = [float(row[i]) for i in range(1,4)]
        acc = [float(row[i]) for i in range(4,7)]

        

        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu'
        imu_msg.header.seq = seq_counter
        seq_counter += 1
        imu_msg.header.stamp = Time.from_sec(ts)

        imu_msg.angular_velocity.x = gyr[0]
        imu_msg.angular_velocity.y = gyr[1]
        imu_msg.angular_velocity.z = gyr[2]

        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]

        bag_file.write('/imu', imu_msg, t=imu_msg.header.stamp)

print("Writing video data...")
bridge = cv_bridge.CvBridge()
if args.use_video:  
    cap = cv2.VideoCapture(args.video)
    with open(args.stamps, 'r') as stamps_file:
        stamps_reader = csv.reader(stamps_file)
        next(stamps_reader) # skip header

        for row in stamps_reader:
            ts = float(row[0])
            frame_count = int(row[1])

            ret, frame = cap.read()
            if not ret:
                print("Camera data ended at frame {}.".format(frame_count))
                break

            img_msg = bridge.cv2_to_imgmsg(frame)

            img_msg.header.frame_id = 'cam'
            img_msg.header.seq = frame_count
            img_msg.header.stamp = Time.from_sec(ts)

            bag_file.write('/cam', img_msg, t=img_msg.header.stamp)
            # bag_file.write('/cam', img_msg)
    cap.release()
else:
    with open(args.stamps, "r") as stamps_file:
        stamps_reader = csv.reader(stamps_file)
        next(stamps_reader)

        for row in stamps_reader:
            ts = float(row[0])
            frame_count = int(row[1])

            frame = cv2.imread(args.video+"/frame_{}.jpg".format(frame_count))

            img_msg = bridge.cv2_to_imgmsg(frame)

            img_msg.header.seq = frame_count
            img_msg.header.stamp = Time.from_sec(ts)

            bag_file.write('/cam', img_msg, t=img_msg.header.stamp)


print("Writing groundtruth data from GPS recording...")
with open(args.imu, 'r') as gps_file:
    gps_reader = csv.reader(gps_file)
    next(gps_reader) # skip header
    seq_counter = 1
    
    gts = []
    att_data = []
    gps_data = []

    
    for row in gps_reader:
        t = float(row[0])
        att = [float(row[i]) for i in range(8,11)]
        gps = [float(row[i]) for i in range(14,17)] # Change to (11,14) for the first GPS

        # Avoid repeat entries
        if len(gts) > 0 and all(att_data[-1][i] == att[i] for i in range(3)) and all(gps_data[-1][i] == gps[i] for i in range(3)):
            continue
        
        if len(gts) > 0 and all(gps_data[-1][i] == gps[i] for i in range(3)):
            continue
        

        gts.append(t)
        att_data.append(att)
        gps_data.append(gps)


N = len(gts)
for i in range(N):
    position = gps2xyz(gps_data[0], gps_data[i])
    if not args.ned:
        position = ned2enu(position)
    position_msg = PointStamped()
    position_msg.header.stamp = Time.from_sec(gts[i])
    position_msg.header.seq = i+1
    position_msg.header.frame_id = 'gps'

    position_msg.point.x = position[0]
    position_msg.point.y = position[1]
    position_msg.point.z = position[2]

    bag_file.write('/gps', position_msg, t=position_msg.header.stamp)
    
# bag_file.reindex()
bag_file.close()