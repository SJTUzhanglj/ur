#!/usr/bin/env python  
import rospy
from math import atan2, asin
import math
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

def fromQuat(x, y, z, w):
    roll = atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = asin(2*(w*y-z*x))
    yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw

if __name__ == '__main__':
    rospy.init_node('camera_in_object')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(30.0)
    cam_trans_pub = rospy.Publisher('/object_pose_stamped', geometry_msgs.msg._PoseStamped.PoseStamped, queue_size=1)
    pose_stamped = geometry_msgs.msg._PoseStamped.PoseStamped()
    pose_stamped.header.frame_id = 'camera_frame'
    while not rospy.is_shutdown():
        #print tfBuffer.can_transform('base', 'object', rospy.Time())
        try:            
            trans = tfBuffer.lookup_transform('camera_frame', 'object', rospy.Time())
            r,p,y = fromQuat(trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)
            print (r/math.pi*180, p/math.pi*180, y/math.pi*180)
            #print trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        #pub_pose.header.stamp = rospy.Time.now()
        #pub_pose.transform = trans.transform
        #tfm = tf2_msgs.msg.TFMessage([pub_pose])
        #cam_in_obj_pub.publish(tfm)
        
        #pose_stamped.header.stamp = rospy.Time.now()
        #pose_stamped.pose.position = trans.transform.translation
        """
        sign + or -  ???
        """
        #pose_stamped.pose.orientation = trans.transform.rotation

        #cam_trans_pub.publish(pose_stamped)


