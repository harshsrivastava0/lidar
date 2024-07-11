#!/usr/bin/env python3
import rospy
from clustering.msg import arr, arrofarr
from sensor_msgs.msg import PointCloud


# this will publish the post ground removal PointCloud2 msg to /colored_cone topic
pub = rospy.Publisher('/colored_cone', arrofarr, queue_size=10)

global camera_msg
camera_msg = []

global coord
coord = []

def orgycoord(msg):
    global coord
    coord = []

    for i in range(len(msg.data)):
        temp = []
        temp.append(msg.data[i].data[0])
        temp.append(msg.data[i].data[1])
        temp.append(msg.data[i].data[2])
        coord.append(temp)


def camera(msg):
    size = int(msg.channels[0].values[0])
    global camera_msg
    camera_msg = []
    for i in range(1, size+1, 1):
        if msg.channels[i].values[0]>0.5:
            temp = []
            temp.append(msg.channels[i].name[0])
            for j in range(1,5,1):
                temp.append(msg.channels[i].values[j])
            camera_msg.append(temp)
       

def lidar(msg):
    finalmsg = arrofarr()
    for i in range(len(msg.data)):
        temp = arr()
        global coord
        #correcting the rotation (without this, the coordinates would be as if the lidar was facing left)
        temp.data.append(-coord[i][1])
        temp.data.append(coord[i][0])
        temp.data.append(coord[i][2])
        global camera_msg
        for j in range(len(camera_msg)):
            if msg.data[i].data[0]>=camera_msg[j][1] and msg.data[i].data[0] <= camera_msg[j][3] and msg.data[i].data[1]>=camera_msg[j][2] and msg.data[i].data[1] <= camera_msg[j][4]:
                if(camera_msg[j][0]=='y'):
                    temp.data.append(0)
                if(camera_msg[j][0]=='b'):
                    temp.data.append(1)
                break
        finalmsg.data.append(temp)
    

    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    finalmsg.header = header
    rospy.loginfo('Publishing to /colored_cone topic')
    pub.publish(finalmsg)
    
    
if __name__ == "__main__":
    rospy.init_node("fusion")
    rospy.Subscriber("/lidar_coordinate", arrofarr, orgycoord)
    rospy.Subscriber("/detect_info", PointCloud, camera)
    rospy.Subscriber("/pixel_coordinate", arrofarr, lidar)

    # Spin to keep the node alive
    rospy.spin()

