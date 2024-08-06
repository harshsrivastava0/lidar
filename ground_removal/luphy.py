#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import pcl
import math

# this will publish the post ground removal PointCloud2 msg to /no_ground_cloud topic
pub = rospy.Publisher('/no_ground_cloud', PointCloud2, queue_size=10)
marker_pub = rospy.Publisher("/visualization_marker", MarkerArray, queue_size=1)
global count
count = 0
global visulist
visulist = []
global plane
plane = {}


global heightlidar
heightlidar = 0.25

global segangle
segangle = math.pi/12

global ringdis
ringdis = 0.4

def propogate(lastpoint, bin_min_points, segment1, segment2, ringcount):
    global plane
    global visulist
    if(ringcount>=50):
        return
    if(segment1 not in bin_min_points or segment2 not in bin_min_points):
        if(segment1 in bin_min_points):
            plane[(segment1, segment2)]= [0, 0, 1, -bin_min_points[segment1][2]]
        if(segment2 in bin_min_points):
            plane[(segment1, segment2)]= [0, 0, 1, -bin_min_points[segment2][2]]
        #to implement:
        # #what if the innermost circle doesn't have a point, because in that case segment1-8 and segment2-8 wont exist
        # if(ringcount==0):
        #     plane[(segment1, segment2)] = plane[(segment1-2, segment2-2)]
        # else:
        #     plane[(segment1, segment2)] = plane[(segment1-(2*math.pi/(3*segangle)), segment2-(2*math.pi/(3*segangle)))]
        propogate(lastpoint, bin_min_points, segment1+(2*math.pi/(3*segangle)), segment2+(2*math.pi/(3*segangle)), ringcount+1)

        return
    
    x1 = lastpoint[0]
    y1 = lastpoint[1]
    z1 = lastpoint[2]

    x2 = bin_min_points[segment1][0]
    y2 = bin_min_points[segment1][1]
    z2 = bin_min_points[segment1][2]

    x3 = bin_min_points[segment2][0]
    y3 = bin_min_points[segment2][1]
    z3 = bin_min_points[segment2][2]

    visulist.append([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x1, y1, z1]])
    a = y2*z3 + y3*z1 + y1*z2 - (y1*z3 + y2*z1 + y3*z2 )
    b = x3*z2 + x1*z3 + x2*z1 - (x3*z1 + x1*z2 + x2*z3 )
    c = x2*y3 + x1*y2 + x3*y1 - (x2*y1 + x1*y3 + x3*y2 )
    d = -x1*a - y1*b - z1*c
    plane[(segment1, segment2)] = [a, b, c, d]
    # print(plane)
    # print()
    newpoint = [(bin_min_points[segment1][0]+bin_min_points[segment2][0])/2, (bin_min_points[segment1][1]+bin_min_points[segment2][1])/2,(bin_min_points[segment1][2]+bin_min_points[segment2][2])/2]
    propogate(newpoint, bin_min_points, segment1+(2*math.pi/(3*segangle)), segment2+(2*math.pi/(3*segangle)), ringcount+1)

def visu():
 
    # Set our initial shape type to be a cube
    markerArraylala = MarkerArray()
    shape = Marker.LINE_STRIP
    global count

    for i in range(1, 10):
        marker = Marker()
        

        # Set the frame ID and timestamp
        marker.header.frame_id = "rslidar"
        marker.header.stamp = rospy.Time.now()

        # Set the namespace and ID for this marker
        marker.ns = "basic_shapes"
        marker.id = count
        count+=1

        # Set the marker type
        marker.type = shape

        # Set the marker action
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.1)

        # Set the pose of the marker
        
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.0
        marker.scale.z = 0.0

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for j in range(19):
            p3 = Point()
            p3.x=i*ringdis*math.sin(j*math.pi/18)
            p3.y=-i*ringdis*math.cos(j*math.pi/18)
            p3.z=0
            marker.points.append(p3)
        markerArraylala.markers.append(marker)

    for i in range(9):
        marker = Marker()
        

        # Set the frame ID and timestamp
        marker.header.frame_id = "rslidar"
        marker.header.stamp = rospy.Time.now()

        # Set the namespace and ID for this marker
        marker.ns = "basic_shapes"
        marker.id = count
        count+=1

        # Set the marker type
        marker.type = shape

        # Set the marker action
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.1)

        # Set the pose of the marker
        
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.0
        marker.scale.z = 0.0

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        p=Point()
        p.x = 0
        p.y = 0
        p.z = 0
        marker.points.append(p)

        p1 = Point()
        p1.x=100*math.sin(i*math.pi/8)
        p1.y=-100*math.cos(i*math.pi/8)
        p1.z=0
        marker.points.append(p1)
        markerArraylala.markers.append(marker)

    for total in visulist:

        marker = Marker()

        # Set the frame ID and timestamp
        marker.header.frame_id = "rslidar"
        marker.header.stamp = rospy.Time.now()

        # Set the namespace and ID for this marker
        marker.ns = "basic_shapes"
        marker.id = count
        count+=1

        # Set the marker type
        marker.type = shape

        # Set the marker action
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.1)

        # Set the pose of the marker
        
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.0
        marker.scale.z = 0.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Define the points for the line strip
        for i in total:
            p2=Point()
            p2.x=i[0]
            p2.y=i[1]
            p2.z=i[2]
            marker.points.append(p2)
        markerArraylala.markers.append(marker)
            
    marker_pub.publish(markerArraylala)

def main(msg):
    global visulist
    global ringdis
    global segangle
    global heightlidar
    visulist = []
    #Converting from PointCloud2 msg type to o3d pointcloud
    pc_data = []
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        #because the current lidar gives coordinates as if its facing towards left. so top left: 1st quadrant, bottom left: 2nd quad...
        #choosing points only in 180 HFOV
        if(point[0]>0):
            pc_data.append([point[0], point[1], point[2]])

    if not pc_data:
        rospy.loginfo('NO points in pointcloud')
        return

    bin_min_points = {}
    bin_all_points = {}
    minhori = 360
    maxhori = 0
    for i in pc_data:
        distance = int(((i[0]**2 + i[1]**2)**0.5)/ringdis)
        horizontal_angle = math.atan2(i[0], -i[1])
        if(horizontal_angle<minhori):
            minhori = horizontal_angle
        if(horizontal_angle>maxhori):
            maxhori = horizontal_angle
        if(horizontal_angle>=2*math.pi/3 or horizontal_angle<=math.pi/6):
            continue
        
        index =int(horizontal_angle/(segangle)) + ((2*math.pi/(3*segangle))*distance) - (math.pi/(6*segangle))

        if(index not in bin_min_points):
            bin_min_points[index] = i
        else:
            if(bin_min_points[index][2]>i[2]):
                bin_min_points[index] = i
        if(index not in bin_all_points):
            bin_all_points[index] = []
        bin_all_points[index].append(i)

    lidarpoint = [0, 0, -heightlidar]

    # print(minhori*180/math.pi)
    # print(maxhori*180/math.pi)
    # print()
    # print(bin_min_points)
    # print()
    # print()
    # for i in bin_min_points:
    #     print(i)

    # print()
    # print()
    # for i in bin_min_points:
    #     if(i<=0):
    #         print(i, end = " ")
    startatring = 0
    for i in range(0+int((2*math.pi/(3*segangle))*startatring), int((2*math.pi/(3*segangle))*(startatring+1)), 2):
        propogate(lidarpoint, bin_min_points, i, i+1, startatring)

    # visu()
    ground = []
    noground = []
    for i in bin_all_points:
        if(i>200):
            continue
        coeffs = []
        if(i%2==0):
            coeffs = plane[(i, i+1)]
        else:
            coeffs = plane[(i-1, i)]
        denominator = (coeffs[0]**2 + coeffs[1]**2 +coeffs[2]**2)**0.5
        for j in bin_all_points[i]:
            distance = abs((coeffs[0]*j[0] + coeffs[1]*j[1] + coeffs[2]*j[2] + coeffs[3])/denominator)
            if(distance <= 0.07):
                ground.append(j)
            else:
                noground.append(j)

    
    # creating a PointCloud2 msg from the list and publishing it to /lidar_coordinate if you want to visualise in rviz
    cloud_ros = pcl.PointCloud()
    cloud_ros.from_list(noground)
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "rslidar"  # Set the appropriate frame ID
    dummy = [(point[0], point[1], point[2]) for point in cloud_ros]
    pc2_msg = point_cloud2.create_cloud_xyz32(header, dummy)
    pub.publish(pc2_msg)
    rospy.loginfo('Publishing on /no_ground_cloud')


if __name__ == "__main__":
    rospy.init_node("lu")
    rospy.Subscriber("/rslidar_points", PointCloud2, main)

    # Spin to keep the node alive
    rospy.spin()

