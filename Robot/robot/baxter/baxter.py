#!/usr/bin/env python
#In the spirit of decoupling Liatris logic from Rethink Roboticc's specific functions for Baxter, all Baxter code is here.  The function names are generalized to the liatris_robot class.  Hopefully this will allow other robots to be integrated into the Liatris project.

import math
import random
import rospy
import argparse
import struct
import sys
import numpy
import cv2
import tf
import copy
from ik import IK
import threading
import Tkinter as tk
import json
import moveit_commander
import time
import urllib, json
import os.path
import httplib
from gripper import Robot_Gripper

from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
	ObjectColor,
    Grasp,
    GripperTranslation,
)


from tf import transformations
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
	Polygon,
	Vector3,
    Vector3Stamped
)

import yaml


from trajectory_msgs.msg import(
    JointTrajectory,
    JointTrajectoryPoint
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Range

class Robot(object):
    def __init__(self):
		#Setup inverse kinematics
		self.ik = IK()
		self.gripper = Robot_Gripper()