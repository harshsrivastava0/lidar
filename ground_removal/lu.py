#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pcl
import math

# this will publish the post ground removal PointCloud2 msg to /no_ground_cloud topic
pub = rospy.Publisher('/no_ground_cloud', PointCloud2, queue_size=10)

def main(msg):

    #Converting from PointCloud2 msg type to o3d pointcloud
    pc_data = []
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        #because the current lidar gives coordinates as if its facing towards left. so top left: 1st quadrant, bottom left: 2nd quad...
        #choosing points only in 180 HFOV
        if(point[0]>0 and point[1]>0):
            pc_data.append([point[0], point[1], point[2]])
        elif(point[0]>0 and point[1]<0):
            pc_data.append([point[0], point[1], point[2]])
        # pc_data.append([point[0], point[1], point[2]])

    if pc_data:
        pc_data = np.array(pc_data)
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_data))
        # cloud = cloud.voxel_down_sample(voxel_size=0.08)
    else:
        rospy.loginfo('NO points in pointcloud')
        return

    segment_points = []
    for i in pc_data:
        distance = (i[0]**2 + i[1]**2 + i[2]**2)**0.5
        horizontal_angle = math.atan(-i[1]/i[0])
        if(horizontal_angle)
    

if __name__ == "__main__":
    rospy.init_node("lund")
    rospy.Subscriber("/velodyne_points", PointCloud2, main)

    # Spin to keep the node alive
    rospy.spin()

