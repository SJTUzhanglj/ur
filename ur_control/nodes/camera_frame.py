#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math

class CameraFrameBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "ee_link"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "camera_frame"
            t.transform.translation.x = 0.0738894
            t.transform.translation.y = 0.0689687
            t.transform.translation.z = 0.0554053
            rx = 0.68755
            ry = -0.36382809
            rz = 0.60264708
            rw = -math.sqrt(1.0-rx**2-ry**2-rz**2)
            t.transform.rotation.x = 0.68755
            t.transform.rotation.y = -0.36382809
            t.transform.rotation.z = 0.60264708
            t.transform.rotation.w = rw

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('camera_frame_broadcaster')
    tfb = CameraFrameBroadcaster()

    rospy.spin()
