#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pcl

# this will publish the post ground removal PointCloud2 msg to /no_ground_cloud topic
pub = rospy.Publisher('/no_ground_cloud', PointCloud2, queue_size=10)

def main(msg):

    #Converting from PointCloud2 msg type to o3d pointcloud
    pc_data = []
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        #because the current lidar gives coordinates as if its facing towards left. so top left: 1st quadrant, bottom left: 2nd quad...
        #choosing points only in 180 HFOV
        # if(point[0]>0 and point[1]>0):
        #     pc_data.append([point[0], point[1], point[2]])
        # elif(point[0]>0 and point[1]<0):
        #     pc_data.append([point[0], point[1], point[2]])
        pc_data.append([point[0], point[1], point[2]])

    if pc_data:
        pc_data = np.array(pc_data)
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_data))
        # cloud = cloud.voxel_down_sample(voxel_size=0.08)
    else:
        rospy.loginfo('NO points in pointcloud')
        return

    # Create the segmentation object for the planar model and set all the parameters
    plane_model, inliers = cloud.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)

    if len(inliers) == 0:
        rospy.loginfo('Could not estimate a planar model for the given dataset.')
        return

    # Extract the planar inliers from the input cloud
    cloud_plane = cloud.select_by_index(inliers)

    # Remove the planar inliers, extract the rest
    cloud = cloud.select_by_index(inliers, invert=True)
    

    #converting from o3d pointcloud to pcl point cloud
    post_removal_arr = np.array(cloud.points)
    post_removal_ros_cloud_points = pcl.PointCloud()
    post_removal_ros_cloud_points.from_list(post_removal_arr.tolist())
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    # header.frame_id = "rslidar"  # Set the appropriate frame ID
    header.frame_id = "velodyne"  # Set the appropriate frame ID
    pc_data = [(point[0], point[1], point[2]) for point in post_removal_ros_cloud_points]
    pc2_msg = point_cloud2.create_cloud_xyz32(header, pc_data)


    #Publishing the pointcloud after ground removal to /no_ground_cloud
    rospy.loginfo('Publishing pointcloud post ground removal to /no_ground_cloud')
    pub.publish(pc2_msg)
    
    

if __name__ == "__main__":
    rospy.init_node("ransac3d_ground_removal")
    # rospy.Subscriber("/rslidar_points", PointCloud2, main)
    rospy.Subscriber("/velodyne_points", PointCloud2, main)

    # Spin to keep the node alive
    rospy.spin()

