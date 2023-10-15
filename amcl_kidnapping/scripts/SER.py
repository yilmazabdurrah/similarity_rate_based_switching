#!/usr/bin/env python
######################################################
# Author : Abdurrahman Yilmaz yilmazabdurrah@itu.edu.tr
# Date   : 12 April 2022
# Version: v04
# Purpose: To determine current energy of AMR and SER (similar energy regions) for global localization process
######################################################
#################### import ##########################
import rospy
import roslib
import random
import math
import tf
import numpy as np
from std_msgs.msg import Int32, String, Float32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_conjugate
from tf.transformations import quaternion_multiply
from random import random
import yaml # If required run to install package pyyaml (Hint: $ pip install pyyaml)

############# definitions of variables #################

############# definitions of classes ##################

class WakeUp:

	#############################################################################
	def __init__(self):
	
		rospy.init_node('similar_energy_region')
		self.nodename = rospy.get_name()
		rospy.loginfo("-Info- %s started" % self.nodename)
        
		# variables
		## variables for settings of wake-up
		self.energy = 0.0
		self.energy_filt = 0.0
		self.ser_lb_ub = 01100 # 01 is lb, 100 is ub, first two lb, last three ub
		self.zone = -1
		self.list_energy = []
		self.list_zone_center_x = []
		self.list_zone_center_y = []
		self.list_zone_radius = []
        	self.then = rospy.Time.now()
        	
		# zone list for particle distribution
		with open('/home/roboticlab02/amcl_kidnapping_ws/src/amcl_kidnapping/scripts/zone_list.yaml') as fh:
            		self.zone_list = yaml.load(fh, Loader=yaml.FullLoader)
	
		self.sorted_zone_list = yaml.dump(self.zone_list)

		print('')
		print("The following zones are defined for global localization: ")		
		print(self.sorted_zone_list)
		
		for i in range(0, len(self.zone_list)):
			self.list_zone_radius.append(self.zone_list[i]['zone_radius'])
			self.list_zone_center_x.append(self.zone_list[i]['zone_center_x'])
			self.list_zone_center_y.append(self.zone_list[i]['zone_center_y'])	

		#### parameters #######
		self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the topics
		
		self.maxR_energy = rospy.get_param("/max_range_energy", 10.0) # the max range value will be used for energy value computations in meters
		# Please note that ~param_name: private param, /param_name: global param, param_name: relative param
		print("Max Range for energy computation is assigned to " + str(self.maxR_energy) + " meters")
		
		self.window_energy = rospy.get_param("/window_length_energy", 10) # the number of energy values will be used for current energy value computation
		# Please note that ~param_name: private param, /param_name: global param, param_name: relative param
		print("Window length for current energy computation filter is assigned to " + str(self.window_energy))

		self.ser_dev_radius = rospy.get_param("/ser_deviation_radius", 5) # the ser region bounds are defined based on this deviation radius %, integer value expected
		# Please note that ~param_name: private param, /param_name: global param, param_name: relative param
		print("SER deviation radius is assigned to " + str(self.ser_dev_radius) + "%")
		
		self.t_delta = rospy.Duration(1.0/self.rate)
		self.t_next = rospy.Time.now() + self.t_delta

		# subscriptions
		rospy.Subscriber('/laser_all', LaserScan, self.laserScan_callback)
		rospy.Subscriber('/zone', Int32, self.zone_callback)
		self.EnergyPub = rospy.Publisher("current_energy", Float32, queue_size=10)
		self.EnergyFiltPub = rospy.Publisher("current_energy_filt", Float32, queue_size=10)
		self.SerLbUbPub = rospy.Publisher("ser_lb_ub", Int32, queue_size=10)
		self.ZoneCirclePub = rospy.Publisher("zone_circle", Point, queue_size=10)
   
	#############################################################################
	def spin(self):
	
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()
		if rospy.is_shutdown():
			rospy.loginfo("-Info- %s stopped by user request" % self.nodename)

	#############################################################################
    	def update(self):
    		
		self.maxR_energy_new = rospy.get_param("/max_range_energy", 10.0) # the max range value will be used for energy value computations in meters
		if self.maxR_energy_new != self.maxR_energy:
			self.maxR_energy = self.maxR_energy_new
			rospy.logwarn("Max Range for energy computation is changed to %.2f meters, check the validity of the energy map in use!" % self.maxR_energy)
		
		self.window_energy_new = rospy.get_param("/window_length_energy", 10) # the number of energy values will be used for current energy value computation
		if self.window_energy_new != self.window_energy:
			self.window_energy = self.window_energy_new
			rospy.logwarn("Window length for current energy computation filter is changed to %d" % self.window_energy)

		self.ser_dev_radius_new = rospy.get_param("/ser_deviation_radius", 5) # the ser region bounds are defined based on this deviation radius %, integer value expected
		if self.ser_dev_radius_new != self.ser_dev_radius:
			self.ser_dev_radius = self.ser_dev_radius_new
			rospy.logwarn("SER deviation radius is changed to %d percent" % self.ser_dev_radius)

		energy = Float32()
		energy.data = self.energy
		self.EnergyPub.publish(energy)
		
		energy_filt = Float32()
		energy_filt.data = self.energy_filt
		self.EnergyFiltPub.publish(energy_filt)
		
		ser_center = round(self.energy_filt*100)
		if ser_center-self.ser_dev_radius >= 0:
			lb = ser_center-self.ser_dev_radius
		else:
			lb = 1
		if ser_center+self.ser_dev_radius <= 100:
			ub = ser_center+self.ser_dev_radius
		else:
			ub = 100
		self.ser_lb_ub = 1000*(lb) + ub
		
		ser_lb_ub = Int32()
		ser_lb_ub.data = self.ser_lb_ub
		self.SerLbUbPub.publish(ser_lb_ub)

		zone_circle = Point()
		if self.zone == -1:
			zone_circle.x = 0.0
			zone_circle.y = 0.0
			zone_circle.z = -1.0
		else:
			zone_circle.x = self.list_zone_center_x[self.zone]
			zone_circle.y = self.list_zone_center_y[self.zone]
			zone_circle.z = self.list_zone_radius[self.zone]
		self.ZoneCirclePub.publish(zone_circle)

	#############################################################################
	def laserScan_callback(self, msg):
	
		laserData = msg
		laserRanges = laserData.ranges
		enSum = 0.0
		for i in range(len(laserRanges)):
			if laserRanges[i-1] <= self.maxR_energy:
				enSum = enSum + 1 - laserRanges[i-1]/self.maxR_energy
		
		if len(laserRanges) > 0:
			self.energy = enSum/len(laserRanges)
		else:
			self.energy = 0.0

		if len(self.list_energy) < self.window_energy:
			self.list_energy.append(self.energy)
		else:
			self.list_energy.append(self.energy)
			del self.list_energy[0]	
		if len(self.list_energy) > 1:
			self.energy_mean = np.mean(self.list_energy)
		else:
			self.energy_mean = self.list_energy[0]
		self.energy_std = np.std(self.list_energy)
		self.list_energy_filt = [i for i in self.list_energy if abs(i-self.energy_mean) <= 2*self.energy_std]		
		self.energy_filt = np.mean(self.list_energy_filt)
	#############################################################################
	def zone_callback(self, msg):
		if msg.data >= 0 and msg.data <= len(self.zone_list):
			self.zone = msg.data
		else:
			self.zone = -1

	#############################################################################

if __name__ == '__main__':
    
	try:
        	WakeUpClass = WakeUp()
        	WakeUpClass.spin()
    	except rospy.ROSInterruptException:
        	pass	
	
