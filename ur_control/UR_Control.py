#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 13:07:12 2017

@author: sjtu
"""

#!/usr/bin/env python

## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import _PoseStamped
from math import cos, sin, sqrt, pi, atan2, asin,atan
import numpy as np
## END_SUB_TUTORIAL
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float32
from End2Cam import End2Cam

class Pose:
    def __init__(self, group_pose = None):
        if group_pose is not None:
            self.px = group_pose.position.x
            self.py = group_pose.position.y
            self.pz = group_pose.position.z
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            ox = group_pose.orientation.x
            oy = group_pose.orientation.y
            oz = group_pose.orientation.z
            ow = group_pose.orientation.w
            self.fromQuat(ox, oy, oz, ow)            
        else:
            self.px = 0.0
            self.py = 0.0
            self.pz = 0.0
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
    def print_me(self):
        print "position:"
        print "x={}".format(self.px)
        print "y={}".format(self.py)
        print "z={}".format(self.pz)
        print "RPY:"
        print "roll={}".format(self.roll/pi*180.0)
        print "pitch={}".format(self.pitch/pi*180.0)
        print "yaw={}".format(self.yaw/pi*180.0)
    
#    @staticmethod
#    def Axis2Quat(ax, ay, az, angle):
#      angle *= 0.5
#      sinangle = sin(angle)
#      norm = sqrt(ax*ax, ay*ay, az*az)
#      
#      x = ax / norm * sinangle
#      y = ay / norm * sinangle
#      z = az / norm * sinangle
#      w = cos(angle)
#      return x, y, z, w
    
    def toQuat(self):
        p = self.pitch / 2.0
        y = self.yaw / 2.0
        r = self.roll /2.0
        
        print (p, y, r)
        
        sinp = sin(p)
        siny = sin(y)
        sinr = sin(r)
        cosp = cos(p)
        cosy = cos(y)
        cosr = cos(r)
      
        ox = sinr * cosp * cosy - cosr * sinp * siny
        oy = cosr * sinp * cosy + sinr * cosp * siny
        oz = cosr * cosp * siny - siny * sinp * cosy
        ow = cosr * cosp * cosy + sinr * sinp * siny
        return ox, oy, oz, ow
    
    def fromQuat(self, x, y, z, w):
        self.roll = atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        self.pitch = asin(2*(w*y-z*x))
        self.yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        
#    def toMatrix(self):
#        cosy = cos(self.yaw)
#        cosp = cos(self.pitch)
#        cosr = cos(self.roll)
#        siny = sin(self.yaw)
#        sinp = sin(self.pitch)
#        sinr = sin(self.roll)
#        return [[cosr*cosy-sinr*cosp*siny, -sinr*cosy-cosr*cosp*siny, sinp*siny],
#                [cosr*siny+sinr*cosp*cosy, -sinr*siny+cosr*cosp*cosy, -sinp*cosy],
#                [sinr*sinp,                cosr*sinp,                 cosp]]
       
    @staticmethod
    def Quat2Matrix(x, y, z, w):
        return [[1-2*(y*y+z*z), 2*(x*y-z*w),  2*(x*z+y*w)],
                [2*(x*y+z*w),   1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),   2*(y*z+x*w),  1-2*(x*x+y*y)]]
        

        
def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True
    """
    q = np.empty((4, ), dtype=np.float32)
    M = np.array(matrix, dtype=np.float32, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / sqrt(t * M[3, 3])
    return q

class UR_Control:
    def __init__(self, node_name = 'ur_control'):
        self.node = node_name
          ## First initialize moveit_commander and rospy.
        print "============ Initialize:"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur_control',
                      anonymous=True)   
        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        self.robot = moveit_commander.RobotCommander()
        ## We can get a list of all the groups in the robot
        print "============ Robot Groups:"
        print self.robot.get_group_names()
        ## Instantiate a PlanningSceneInterface object.  This object is an interface
        ## to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()
        ## Instantiate a MoveGroupCommander object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the left
        ## arm.  This interface can be used to plan and execute motions on the left
        ## arm.
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_max_velocity_scaling_factor = 0.1
        self.group.set_max_acceleration_scaling_factor = 0.2
        ## We create this DisplayTrajectory publisher which is used below to publish
        ## trajectories for RVIZ to visualize.
        self.display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        ##
        ## We can get the name of the reference frame for this robot
        print "============ Reference frame: %s" % self.group.get_planning_frame()
        
        ## We can also print the name of the end-effector link for this group
        print "============ end_effector frame: %s" % self.group.get_end_effector_link()

        self.current_pose = self.get_current_pose(display = True)
        #print self.current_pose.roll/pi*180,self.current_pose.pitch/pi*180,self.current_pose.yaw/pi*180
        self.current_pose.print_me()
        ## We can also print the name of the end-effector link for this group
        print "============ RPY: %s" % self.group.get_current_rpy()
        
        print "============Init camera topics start=============="
        self.init_camera_topics()
    
    def init_camera_topics(self):
        self.camera_pose_in_obj = _PoseStamped.PoseStamped()
        self.camera_pose_in_obj_sub = rospy.Subscriber("camera_pose_stamped", 
                                                       _PoseStamped.PoseStamped, 
                                                       self.msgCallback, 
                                                       queue_size=1)
        self.rate = rospy.Rate(30.0)
        self.world_frame = "world"                                             
        self.tf_buffer_ = tf2_ros.Buffer()                                    
        self.tf2_ = tf2_ros.TransformListener(self.tf_buffer_)
        self.c2e = np.linalg.inv(End2Cam())
        print "============Init camera topics done!============"
        
        
    def msgCallback(self, cb_data):
        self.camera_pose_in_obj = cb_data
            
      
    def get_current_pose(self, display = False):
        pose = self.group.get_current_pose().pose
        
        if display:
            print "============ Printing robot pose"
            print pose
        
        return Pose(group_pose = pose)

#    def plan_next_pose(self, next_pose):
#        pose_target = geometry_msgs.msg.Pose()
#        ox, oy, oz, ow = next_pose.toQuat()
#        pose_target.orientation.w = ow
#        pose_target.orientation.x = ox
#        pose_target.orientation.y = oy
#        pose_target.orientation.z = oz
#        pose_target.position.x = next_pose.px
#        pose_target.position.y = next_pose.py
#        pose_target.position.z = next_pose.pz
#        self.group.set_pose_target(pose_target)
#        self.group.go(wait = True)
    def plan_next_pose(self, pose_target):
        self.group.set_pose_target(pose_target)
        self.group.go(wait = True)
    
    def get_current_rpy_in_object(self):
        ori_in_obj = self.camera_pose_in_obj.pose.orientation
        pos_in_obj = self.camera_pose_in_obj.pose.position
        current_rpy_in_object = Pose()
        current_rpy_in_object.px = pos_in_obj.x
        current_rpy_in_object.py = pos_in_obj.y
        current_rpy_in_object.pz = pos_in_obj.z
        current_rpy_in_object.fromQuat(ori_in_obj.x, ori_in_obj.y,ori_in_obj.z,ori_in_obj.w)
        return current_rpy_in_object
    

        
    def transform_end_pose_in_world(self,pose):
        o2c_trans = np.zeros([4,4])
        w2o_trans = np.zeros_like(o2c_trans)
        c2e_trans = np.zeros_like(o2c_trans)
        
        """
        object -> camera
        """
        x,y,z,w = pose.toQuat()
        o2c_trans[:3,:3] = pose.Quat2Matrix(x,y,z,w)
        o2c_trans[:3,3] = np.array([pose.px, pose.py, pose.pz])
        o2c_trans[3,3] = 1.0

        """
        world -> object
        """
        trans = self.tf_buffer_.lookup_transform("world", "object", rospy.Time())
        #print trans
        ori = trans.transform.rotation
        pos = trans.transform.translation
        w2o_trans[:3,:3] = pose.Quat2Matrix(ori.x, ori.y, ori.z, ori.w)
        w2o_trans[:3, 3] = np.array([pos.x, pos.y, pos.z])
        w2o_trans[3, 3] = 1.0
        
        """
        camera -> end
        """
        c2e_trans = self.c2e
        w2c_trans = np.dot(w2o_trans, o2c_trans)
        w2e_trans = np.dot(w2c_trans, c2e_trans)
        return w2e_trans
        
    
    def get_current_spherical_pos(self):
        pos_in_obj = self.camera_pose_in_obj.pose.position
        polar = atan(pos_in_obj.x/pos_in_obj.y)
        azimuth = atan(sqrt(pos_in_obj.x**2+pos_in_obj.y**2)/pos_in_obj.z)
        return azimuth, polar

        


if __name__=='__main__':
    ctr = UR_Control()
    rospy.sleep(1.0)
    current_rpy = ctr.get_current_rpy_in_object()
    current_rpy.print_me()
    
    mod_rpy = current_rpy

    dist = sqrt(mod_rpy.px**2+mod_rpy.py**2+mod_rpy.pz**2)
    mod_rpy.pitch = 0.0
    #mod_rpy.px += 0.05
    #mod_rpy.roll -= 10.0/180.0 * pi
    #mod_rpy.yaw += 2.0/180.0*pi
    mod_rpy.roll = -180.0/180.0*pi
    mod_rpy.yaw = -90.0/180.0*pi
    mod_rpy.px = 0.0
    mod_rpy.py = 0.0
    w2e_trans = ctr.transform_end_pose_in_world(mod_rpy)
    print "=================ensure the in_plane rotation is zero!==========="
    print w2e_trans[:3, 3]
    print quaternion_from_matrix(w2e_trans)
    print "-----------------------current pose is------------"
    print ctr.tf_buffer_.lookup_transform("world", "ee_link", rospy.Time())
    
    next_pose = geometry_msgs.msg.Pose()
    next_ori = quaternion_from_matrix(w2e_trans)
    next_pose.orientation.x = float(next_ori[0])
    next_pose.orientation.y = float(next_ori[1])
    next_pose.orientation.z = float(next_ori[2])
    next_pose.orientation.w = float(next_ori[3])
    next_pos = w2e_trans[:3,3]
    next_pose.position.x = float(next_pos[0])
    next_pose.position.y = float(next_pos[1])
    next_pose.position.z = float(next_pos[2])
    ctr.plan_next_pose(next_pose)
    
    
    az, po = ctr.get_current_spherical_pos()
    print az/pi*180.0, po/pi*180.0
    #rospy.spin()
    #next_pose = ctr.current_pose
    #print ("roll={}, pitch={}, yaw={}".format(next_pose.roll/pi*180, next_pose.pitch/pi*180, next_pose.yaw/pi*180))
    #next_pose.py -= 0.30
    #next_pose.pitch = 0
    #next_pose.pitch = 60.0/180.0*pi
    #next_pose.roll = 0
    #ctr.plan_next_pose(next_pose)
    
    #cur_pose = ctr.current_pose
    
    #print ("After move roll={}, pitch={}, yaw={}".format(cur_pose.roll/pi*180, cur_pose.pitch/pi*180, cur_pose.yaw/pi*180))


