#!/usr/bin/env python3
import rospy
import numpy as np
from clustering.msg import arr, arrofarr


# this will publish the post ground removal PointCloud2 msg to /pixel_coordinate topic
pub = rospy.Publisher('/pixel_coordinate', arrofarr, queue_size=10)

def main(msg):
    msgtosend = arrofarr()

    # defining the K camera matrix for zed/left using the topic /zed/left/camera_info
    K = np.array([[235.27027794031173, 0.0, 336.5], [0.0, 235.27027794031173, 188.5], [0.0, 0.0, 1.0]])

    #converting from lidar coordinates to image coordinates
    for i in msg.data:
        temp = arr()

        #applying transform from frame:velodyne to frame:zed_left_camera
        x_new = i.data[0] + 1.491
        y_new = i.data[1] - 0.06
        z_new = i.data[2] - 0.556

        #rotating the coordinate system to match z axis going into image plane, y axis going downwards, and x axis going rightwards
        initial_z = z_new
        z_new = x_new
        x_new = -y_new
        y_new = -initial_z
        
        O_cam = np.array([[x_new], [y_new], [z_new]])
        O_img = np.matmul(K, O_cam)
        O_pixel = np.array([[O_img[0][0]/O_img[2][0]], [O_img[1][0]/O_img[2][0]], [O_img[2][0]/O_img[2][0]]])
        temp.data.append(O_pixel[0][0])
        temp.data.append(O_pixel[1][0])
        temp.data.append(O_pixel[2][0])
        msgtosend.data.append(temp)

    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"
    msgtosend.header = header
    rospy.loginfo('Publishing pixel coordinates')
    pub.publish(msgtosend)
    
    
if __name__ == "__main__":
    rospy.init_node("coordinate_transform")
    rospy.Subscriber("/lidar_coordinate", arrofarr, main)

    # Spin to keep the node alive
    rospy.spin()

