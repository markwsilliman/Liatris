#!/usr/bin/env python

#
# Inverse Kinematics
#

import math
import rospy
import argparse
import struct
import sys

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from tf import transformations
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from baxter_interface import RobotEnable, CameraController, Limb
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class IK(object):
	def __init__(self):
		self._left_arm = Limb('left')
	        self._left_arm.set_joint_position_speed(0.6)
        	self._right_arm = Limb('right')
	        self._right_arm.set_joint_position_speed(0.6)

	def neutral(self):
		self._left_arm.move_to_neutral()
		self._right_arm.move_to_neutral()
	
	def ik_calculate(self,limb,pos,rot):
	    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	    ikreq = SolvePositionIKRequest()
	    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

	    #rotation -0.5 is perp and -1.0 is parallel for rot[2]
	    
	    quat = transformations.quaternion_from_euler(rot[0], rot[1], rot[2])

	    pose = PoseStamped(
		    header=hdr,
		    pose=Pose(
		        position=Point(
		            x=pos[0], #depth 
		            y=pos[1], #lateral
		            z=pos[2], #height
		        ),
		        orientation=Quaternion(
		            quat[0],
		            quat[1],
		            quat[2],
		            quat[3],
		        ),
		    ),
		)

	    ikreq.pose_stamp.append(pose)
	    try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	    except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 1

	    # Check if result valid, and type of seed ultimately used to get solution
	    # convert rospy's string representation of uint8[]'s to int's
	    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
		                       resp.result_type)
	    if (resp_seeds[0] != resp.RESULT_INVALID):
		seed_str = {
		            ikreq.SEED_USER: 'User Provided Seed',
		            ikreq.SEED_CURRENT: 'Current Joint Angles',
		            ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
		           }.get(resp_seeds[0], 'None')
		#print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
		#      (seed_str,))
		# Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return limb_joints
	    else:
		pass
		#print pose
		#print("INVALID POSE - No Valid Joint Solution Found.")

	    return 0

	def set_left(self,x,y,z,timeout=15,wait=True):
		return self.ik_moveto_position("left",(x,y,z),(0,math.pi,math.pi*-1.0),wait,timeout)

	def set_right(self,x,y,z,timeout=15,wait=True):
		return self.ik_moveto_position("right",(x,y,z),(0,math.pi,math.pi*-1.0),wait,timeout)

	def set_right_rfid_down(self,x,y,z,offset_vertical,rot,timeout=15,wait=True):
		return self.ik_moveto_position("right",(x,y,z),(0,math.pi * (1 + offset_vertical),rot),wait,timeout)

	def set_left_down_for_pickup(self,x,y,z,offset_vertical,rot,timeout=15,wait=True):
		return self.ik_moveto_position("left",(x,y,z),(0,math.pi * (1 + offset_vertical),rot),wait,timeout)

	def set_timeout(self,limb,timeout=10):
		if limb == "left":
			return self._left_arm.set_command_timeout(timeout)
		else:
			return self._right_arm.set_command_timeout(timeout)

	def set_speed(self,limb,speed=0.3):
		if limb == "left":
			return self._left_arm.set_joint_position_speed(speed)
		else:
			return self._right_arm.set_joint_position_speed(speed)

	def ik_moveto_position(self,limb,pos=(0.657579481614,0.01981417433,0.28352386502),rot=(0,math.pi,math.pi*-1.0),wait=True,timeout=15):
		_goal = self.ik_calculate(limb,pos,rot)
		if(_goal == 0):
			return False

		if not wait:
			if limb == "left":
				self._left_arm.set_joint_positions(_goal)
			else:
				self._right_arm.set_joint_positions(_goal)
		else:
			if limb == "left":
				self._left_arm.move_to_joint_positions(_goal,timeout)
			else:
				self._right_arm.move_to_joint_positions(_goal,timeout)

		return True

	def get_force(self,limb):
		if limb == "left":
			return self._left_arm.endpoint_effort()['force']
		else:
			return self._right_arm.endpoint_effort()['force']

	def get_pose(self,limb):
		if limb == "left":
			return self._left_arm.endpoint_pose()['position']
		else:			
			return self._right_arm.endpoint_pose()['position']