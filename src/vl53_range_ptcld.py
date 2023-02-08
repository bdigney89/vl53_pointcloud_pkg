#!/usr/bin/env python3

# works bd tdo , clean it up
# rosrun tf static_transform_publisher 0 0 0 0 0 0 fixed_frame map 30
#
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray 
from sensor_msgs import point_cloud2
#from sensor_msgs import point_cloud

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import  PointCloud2, PointField
from std_msgs.msg import Header
import struct
from geometry_msgs.msg import Point32

import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose




class vl53_get_ptcl:

    def __init__(self):
        pub_topic_name ="/vl53_ptcld"
        sub_topic_name ="/vl53_range1"

        self.pub = rospy.Publisher(pub_topic_name, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(sub_topic_name, Float32MultiArray, self.ptcl_callback)
        self.ptcl_out= PointCloud2()

    def ptcl_callback(self, msg):
        temp = Float32MultiArray()
        temp=  msg

        points = []
        i=0
        lim = 8
        fov = np.pi/4.0
        half_fov = fov/2.0
        resolution = 8.0
        delta_fov = half_fov/(7.0)

        for h in range(lim):
            for w in range(lim):
                vec_magnitude = temp.data[h*lim+w]/1000.   # in meters
               
                theta = -fov + w*delta_fov
                alpha = -fov + h*delta_fov
                x =  vec_magnitude*(np.sin(alpha))
                y =  vec_magnitude *(np.cos(alpha)*np.cos(theta))
                z = vec_magnitude *(np.cos(alpha)*np.sin(theta))
                r = int(200)  #(x * 255.0) # why not add colour
                g = int(200)  #y * 255.0)
                b = int(200)  #z * 255.0)
                a = 255
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x, y, z, rgb]
                points.append(pt)
               
                

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    # PointField('rgb', 12, PointField.UINT32, 1),
                    PointField('rgba', 12, PointField.UINT32, 1),
                ]
        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.get_time
        self.ptcl_out= point_cloud2.create_cloud(header, fields, points)
        self.ptcl_out.header.stamp = rospy.Time.now()
        
        self.pub.publish(self.ptcl_out)

if __name__ == '__main__':
    node_name ="vl53_range_ptcld"
    rospy.init_node(node_name)
    vl53_get_ptcl()
    rospy.spin()
