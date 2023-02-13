#!/usr/bin/env python3

# works bd tdo , clean it up
# rosrun tf static_transform_publisher 0 0 0 0 0 0 fixed_frame map 30
#
import rospy

from std_msgs.msg import Float32MultiArray 
from sensor_msgs import point_cloud2


from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import  PointCloud2, PointField
from std_msgs.msg import Header
import struct

import numpy as np


class vl53_get_ptcl:

    def __init__(self):
        pub_topic_name ="/vl53_ptcld"
        sub_topic_name ="/vl53_range1"

        self.pub = rospy.Publisher(pub_topic_name, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(sub_topic_name, Float32MultiArray, self.ptcl_callback)
        self.ptcl_out= PointCloud2()
        self.pitch = [
            62.85,	66.50,	69.40,	71.08,	71.08,	69.40,	66.50,	62.85,
            66.50,	70.81,	75.05,	77.50,	77.50,	75.05,	70.81,	66.50,
            69.40,	75.05,	78.15,	81.76,	81.76,	78.15,	75.05,	69.40,
            71.08,	77.50,	81.76,	86.00,	86.00,	81.76,	77.50,	71.08,
            71.08,	77.50,	81.76,	86.00,	86.00,	81.76,	77.50,	71.08,
            69.40,	75.05,	78.15,	81.76,	81.76,	78.15,	75.05,	69.40,
            66.50,	70.81,	75.05,	77.50,	77.50,	75.05,	70.81,	66.50,
            62.85,	66.50,	69.40,	71.08,	71.08,	69.40,	66.50,	62.85  
        ]
        self.yaw=[
            135.00, 125.40, 113.20, 98.13,  81.87, 66.80, 54.60, 45.00,
            144.60, 135.00, 120.96, 101.31, 78.69, 59.04, 45.00, 35.40,
            156.80, 149.04, 135.00, 108.45, 71.55, 45.00, 30.96, 23.20,
            171.87, 168.69, 161.55, 135.00, 45.00, 18.45, 11.31,  8.13,
            188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
            203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
            215.40, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
            225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00  
        ]
        self.sin_of_pitch = []
        self.cos_of_pitch= []
        self.sin_of_yaw= []
        self.cos_of_yaw= []

        for i in range(64):
            self.sin_of_pitch.append(np.sin((self.pitch[i])*np.pi/180.0))
            self.cos_of_pitch.append(np.cos((self.pitch[i])*np.pi/180.0))
            self.sin_of_yaw.append(np.sin((self.yaw[i])*np.pi/180.0))
            self.cos_of_yaw.append(np.cos((self.yaw[i])*np.pi/180.0)) 


    def ptcl_callback(self, msg):
        temp = Float32MultiArray()
        temp=  msg

        points = []
        i=0
        lim = 8
        fov = np.pi/4.0
        half_fov = fov/2.0
        resolution = 8.0
        #delta_fov = half_fov/(7.0)
        delta_fov = (np.pi/2.)/(7.0)

        for i in range(64):  
            vec_magnitude = (temp.data[i]/self.sin_of_pitch[i])/1000.0
            x = vec_magnitude*self.cos_of_yaw[i]* self.cos_of_pitch[i]
            y= vec_magnitude*self.sin_of_yaw[i]*self.cos_of_pitch[i]
            z= temp.data[i]/1000.0
            r = int(250)  #(x * 255.0) # why not add colour
            g = int(250)  #y * 255.0)
            b = int(250)  #z * 255.0)
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
