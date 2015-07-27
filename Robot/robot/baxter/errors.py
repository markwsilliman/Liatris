#!/usr/bin/env python

#
# Standardized way of throwing exceptions
#

class Robot_Error(object):

	def error(self,msg):
		raise Exception("Abbe Error: " + msg)