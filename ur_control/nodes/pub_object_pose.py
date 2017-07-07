#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('object_pose')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(30.0)
    obj_pose_pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)
    pub_pose = geometry_msgs.msg.TransformStamped()
    pub_pose.header.frame_id = "world"
    pub_pose.child_frame_id = "object"
    pub_pose.transform.rotation.w = 1.0
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'object_frame', rospy.Time())
            
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        #object orientation is set accordance with the camera coordinate system
        pub_pose.header.stamp = rospy.Time.now()
        pub_pose.transform.translation.x = trans.transform.translation.x
        pub_pose.transform.translation.y = trans.transform.translation.y
        pub_pose.transform.translation.z = trans.transform.translation.z
        tfm = tf2_msgs.msg.TFMessage([pub_pose])
        obj_pose_pub.publish(tfm)
