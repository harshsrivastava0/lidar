#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import os
import glob
from clustering.msg import arr, arrofarr
import math

pub = rospy.Publisher('/cluster_indices', arrofarr, queue_size=10)

def main(msg):
    
    #deleting all prior clusters from the destination folder
    files = glob.glob(r'/home/harsh/catkin_ws/src/lidar/clusters/*')
    for f in files:
        os.remove(f)
    

    #Converting from PointCloud2 msg type to o3d pointcloud
    pc_data = []
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        pc_data.append([point[0], point[1], point[2]])
    if pc_data:
        pc_data = np.array(pc_data)
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_data))


    #labels is an array which at every index has a number associated with a cluster of that number
    # so '2' at index '3' in labels signifies that the point in pointcloud at index 3 belongs to cluster number 2
    labels = np.array(cloud.cluster_dbscan(eps=0.30, min_points=3, print_progress=False))
    max_label = labels.max()
    #print(f"point cloud has {max_label + 1} clusters")

    rospy.loginfo('Performing dbscan clustering')
    rospy.loginfo(max_label+1)


    #dict1 stores cluster_no. as key and list of indices of points in that cluster as values
    #dict1 starts with key number -1 and the values of key -1 corresponds to noise
    k=0
    dict1 = {}
    for i in range(len(np.asarray(cloud.points))):
        if(labels[k] in dict1):
            dict1[labels[k]].append(i)
        else:
            dict1[labels[k]] = []
            dict1[labels[k]].append(i)
        k+=1
    

    #cluster_indices is a list consisting of lists. eg: the sublist at index 2 contains *index* of all those points in cluster no. 2
    #distofcluster contains 2 at index 3: so distance of cluster 3 is 2 units
    cluster_indices = []
    distofcluster = []
    tempsumofdist = 0
    for i in dict1:
        if (i!=-1):
            cluster_indices.append(dict1[i])
            for j in dict1[i]:
                tempsumofdist += (pc_data[j][0]**2 + pc_data[j][1]**2 + pc_data[j][2]**2)**0.5
            avgdist  = tempsumofdist/len(dict1[i])
            distofcluster.append(avgdist)
            tempsumofdist = 0

    # # performing a check if clusters are valid
    # cone_cluster_indices = []
    # heightcone = 0.325
    # widthcone = 0.228
    
    # # in radians
    # #for velodyne v16 (PUCK)
    # horizontalres = 0.00335103
    # verticalres = 0.03272494
    # for i in range(len(cluster_indices)):
    #     d = distofcluster[i]
    #     expectedpoints = 0.125*(1/d)*(1/d)* heightcone * widthcone * (1/math.tan(verticalres/2))* (1/math.tan(horizontalres/2))
    #     if(len(cluster_indices[i])>=expectedpoints-5 and len(cluster_indices[i])<=expectedpoints+5):
    #         cone_cluster_indices.append(cluster_indices[i])

    # print(len(cluster_indices))
    # print(len(cone_cluster_indices))


    #making a ros msg to publish the indices
    #arr:      int32[] data
    #arrofarr: Header header
    #          arr[] data
    
    indices_msg = arrofarr()
    for i in cluster_indices:
        temp = arr()
        temp.data = i
        indices_msg.data.append(temp)


    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"
    indices_msg.header = header
    
    pub.publish(indices_msg)
    rospy.loginfo('Publishing cluster_indices to /cluster_indices')

    # storing each cluster as seperate pcd file in the destination folder
    j = 0
    for indices in cluster_indices:
        
        #selecting only those points from initial pointcloud whose indices are listed in 'indices'
        cloud_cluster = cloud.select_by_index(indices)
        
        #print(f"PointCloud representing the Cluster: {len(np.asarray(cloud_cluster.points))} data points.")
        
        o3d.io.write_point_cloud(f"/home/harsh/catkin_ws/src/lidar/clusters/cloud_cluster_{j:04d}.pcd", cloud_cluster, write_ascii=False)
        j += 1

if __name__ == "__main__":
    rospy.init_node("cluster_extraction_dbscan")
    rospy.Subscriber("/no_ground_cloud", PointCloud2, main)

    # Spin to keep the node alive
    rospy.spin()
