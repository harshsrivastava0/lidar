#!/usr/bin/env python3
import rospy
from clustering.msg import arr, arrofarr
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pcl

pub_coord = rospy.Publisher('/lidar_coordinate', arrofarr, queue_size=10)
pub_cloud = rospy.Publisher('/cone_pose_cloud', PointCloud2, queue_size=10)

def main(msg):
    final_points = {}
    for points in msg.data:
        if(points.data[3]==-1.0):
            continue
        if(int(points.data[3]) in final_points):
            final_points[int(points.data[3])][0] += points.data[0]
            final_points[int(points.data[3])][0] /= 2
            final_points[int(points.data[3])][1] += points.data[1]
            final_points[int(points.data[3])][1] /= 2
            final_points[int(points.data[3])][2] += points.data[2]
            final_points[int(points.data[3])][2] /= 2
        else:
            final_points[int(points.data[3])] = [points.data[0], points.data[1], points.data[2]]

    coordinates = []
    for i in final_points:
        #correcting the rotation (without this, the coordinates would be as if the lidar was facing left)
        # temp = final_points[i][0]
        # final_points[i][0] = -final_points[i][1]
        # final_points[i][1] = temp
        coordinates.append(final_points[i])

    # creating a PointCloud2 msg from the list and publishing it to /lidar_coordinate if you want to visualise in rviz
    cloud_ros = pcl.PointCloud()
    cloud_ros.from_list(coordinates)
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"  # Set the appropriate frame ID
    dummy = [(point[0], point[1], point[2]) for point in cloud_ros]
    pc2_msg = point_cloud2.create_cloud_xyz32(header, dummy)
    pub_cloud.publish(pc2_msg)

    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    posemsg = arrofarr()
    posemsg.header = header
    for i in coordinates:
        temp = arr()
        temp.data = i
        posemsg.data.append(temp)
    rospy.loginfo('Publishing to /lidar_coordinate topic')
    pub_coord.publish(posemsg)
    

if __name__ == "__main__":
    rospy.init_node("cone_pose")
    rospy.Subscriber("/clusters", arrofarr, main)
    
    # Spin to keep the node alive
    rospy.spin()

