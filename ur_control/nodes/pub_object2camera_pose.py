#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('camera_in_object')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(30.0)
    cam_in_obj_pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)
    pub_pose = geometry_msgs.msg.TransformStamped()
    pub_pose.header.frame_id = "object"
    pub_pose.child_frame_id = "camera_in_object"
    cam_trans_pub = rospy.Publisher('/camera_pose_stamped', geometry_msgs.msg._PoseStamped.PoseStamped, queue_size=1)
    pose_stamped = geometry_msgs.msg._PoseStamped.PoseStamped()
    pose_stamped.header.frame_id = 'object'
    while not rospy.is_shutdown():
        #print tfBuffer.can_transform('base', 'object', rospy.Time())
        try:            
            trans = tfBuffer.lookup_transform('object', 'camera_frame', rospy.Time())
            #print trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        pub_pose.header.stamp = rospy.Time.now()
        pub_pose.transform = trans.transform
        tfm = tf2_msgs.msg.TFMessage([pub_pose])
        cam_in_obj_pub.publish(tfm)
        
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position = pub_pose.transform.translation
        """
        sign + or -  ???
        """
        pose_stamped.pose.orientation.x = -pub_pose.transform.rotation.x
        pose_stamped.pose.orientation.y = -pub_pose.transform.rotation.y
        pose_stamped.pose.orientation.z = -pub_pose.transform.rotation.z
        pose_stamped.pose.orientation.w = -pub_pose.transform.rotation.w
        cam_trans_pub.publish(pose_stamped)


