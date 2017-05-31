#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import cos, sin, sqrt, pi, atan2, asin
## END_SUB_TUTORIAL

from std_msgs.msg import String

def Axis2Quat(ax, ay, az, angle):
  angle *= 0.5
  sinangle = sin(angle)
  norm = sqrt(ax*ax, ay*ay, az*az)
  
  x = ax / norm * sinangle
  y = ay / norm * sinangle
  z = az / norm * sinangle
  w = cos(angle)
  return x, y, z, w
  
def Euler2Quat(pitch, yaw, roll):
  p = pitch * pi / 2.0
  y = yaw * pi / 2.0
  r = roll * pi /2.0
  
  sinp = sin(p)
  siny = sin(y)
  sinr = sin(r)
  cosp = cos(p)
  cosy = cos(y)
  cosr = cos(r)
  
  x = sinr * cosp * cosy - cosr * sinp * siny
  y = cosr * sinp * cosy + sinr * cosp * siny
  z = cosr * cosp * siny - siny * sinp * cosy
  w = cosr * cosp * cosy + sinr * sinp * siny
  return x, y, z, w
  
def Quat2Euler(x, y, z, w):
  roll = atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
  pitch = asin(2*(w*y-z*x))
  yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
  return roll, pitch, yaw


def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()


  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()


  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("manipulator")

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  #print "============ Waiting for RVIZ..."
  #rospy.sleep(10)
  #print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()



  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  #print "============ Printing robot state"
  #print robot.get_current_state()
  #print "============"
  
  print "============ Printing robot pose"
  cur_pose = group.get_current_pose().pose
  print cur_pose
  x, y, z, w = cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w
  roll, pitch, yaw = Quat2Euler(x, y, z, w)
  print cur_pose
  print ("roll={}, pitch={}, yaw={}".format(roll/pi*180, pitch/pi*180, yaw/pi*180))
  print "============"
  
  

  
  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation = cur_pose.orientation
  
  x, y, z, w = cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w
  roll, pitch, yaw = Quat2Euler(x, y, z, w)
  roll += 180/180*pi
  tx, ty, tz, tw = Euler2Quat(pitch, yaw, roll)
  pose_target.orientation.w = tw
  pose_target.orientation.x = tx
  pose_target.orientation.y = ty
  pose_target.orientation.z = tz
  pose_target.position.x = cur_pose.position.x
  pose_target.position.y = cur_pose.position.y
  pose_target.position.z = cur_pose.position.z
  group.set_pose_target(pose_target)
  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()
  group.go(wait=True)
  
  """
  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)

  
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again).
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Waiting while plan1 is visualized (again)..."
  rospy.sleep(5)


  ## Moving to a pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Moving to a pose goal is similar to the step above
  ## except we now use the go() function. Note that
  ## the pose goal we had set earlier is still active 
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is 
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  # Uncomment below line when working with a real robot
  # group.go(wait=True)

  ## Planning to a joint-space goal 
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.

  group.clear_pose_targets()
  """
  """
  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan
  group_variable_values[0] = 1.0
  group.set_joint_value_target(group_variable_values)

  plan2 = group.plan()

  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(5)
  """
  """
  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints 
  ## for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

#  # second move down
  wpose.position.z -= 0.02
  waypoints.append(copy.deepcopy(wpose))
#
#  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.005,        # eef_step
                               0.0)         # jump_threshold
                               
  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(5)
  group.go(wait=True)
  #print "============ Visualizing plan3"
  #display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  #display_trajectory.trajectory_start = robot.get_current_state()
  #display_trajectory.trajectory.append(plan3)
  #display_trajectory_publisher.publish(display_trajectory);

  #print "============ Waiting while plan3 is visualized (again)..."
  #rospy.sleep(5)
  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  #collision_object = moveit_msgs.msg.CollisionObject()
  """


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"
  

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

