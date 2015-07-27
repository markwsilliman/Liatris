#!/usr/bin/env python

#
# Controls the state (open/close) of the robot's grippers
#

import random
import numpy as np
import math
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, Range
from baxter_core_msgs.msg import ITBState
from baxter_interface import (
	RobotEnable,
	AnalogIO,
	DigitalIO,
	Gripper
)

class Robot_Gripper(object):

	def __init__(self):
		self._gripper = Gripper('left')
		self._gripper.calibrate()
		#right gripper isn't calibrated so it won't drop the RFID reader
		#self._gripper_right = Gripper('right')
		#self._gripper_right.calibrate()

	def close(self,left=True):
		if left:
			self._gripper.close(block=True)
		else:
			self._gripper_right.close(block=True)

	def open(self,left=True):
		if left:
			self._gripper.open(block=True)
		else:
			self._gripper_right.open(block=True)

	def gripping(self,left=True):
		if left:
			return self._gripper.gripping()
		else:
			return self._gripper_right.gripping()