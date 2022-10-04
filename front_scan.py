#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

class FeatureExtracter(Node):
	def __init__(self):
		super().__init__('feature_extracter')
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 
		self.scan_idx = 0
		self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_policy)
		self.publisher_ = self.create_publisher(LaserScan, '/front_scan', 10)


	def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max):
		angle_step = (angle_max - angle_min) / len(ranges)
		angle = angle_min
		cartesian_points = []
		for range in ranges:
			x = range * np.cos(angle)
			y = range * np.sin(angle)
			angle += angle_step
			cartesian_points.append([x,y])

		#points_np = np.array(cartesian_points)
		#print(points_np)
		#plt.figure()
		#plt.scatter(points_np[:,0], points_np[:,1])
		#plt.show()
        
		return cartesian_points
		
        
        #return points_np

	def plots(self, ranges, cartesian_points):
		r = 0
		plt.figure()
		rads = np.arange(0, (2*np.pi), 2*np.pi/360)
		plt.axes(projection='polar')
		for i in rads:
			plt.polar(i, ranges[r], 'g.')
			r+=1
		
		points_np = np.array(cartesian_points)
		#print(points_np)
		plt.figure()
		plt.scatter(points_np[:,0], points_np[:,1])
		plt.show()
		
	#def corners(self, cartesian_points):
	#	corners = []
	#	idx = []
	#	return[]
	#	r = 0
	#	for i in range (0, 351):
	#		s = np.array(points[i])
	#		m = np.array(points[i+
				
	def scan_callback(self,msg):

		ranges = [] # -60, +60 degree interval for front_scan
		angle = msg.angle_min
		for r in msg.ranges:
			x = r * np.cos(angle)
			y = r * np.sin(angle)
			if (angle >= 300.0 * msg.angle_increment) or (angle <= 60.0 * msg.angle_increment):   
				ranges.append(r)
			else:
				ranges.append(0.0)   
			angle += msg.angle_increment
			
		scan = LaserScan()
		scan.header.stamp = msg.header.stamp
		scan.header.frame_id = msg.header.frame_id
		scan.angle_min = msg.angle_min
		scan.angle_max = msg.angle_max
		scan.angle_increment = msg.angle_increment
		scan.time_increment = msg.time_increment
		scan.range_min = msg.range_min
		scan.range_max = msg.range_max 
		scan.ranges = ranges
        
		self.publisher_.publish(scan)
		#self.polar_to_cartesian_coordinate(ranges, msg.angle_min, msg.angle_max) #call with ranges instead of msg.ranges
		cartesian_points = self.polar_to_cartesian_coordinate(ranges, msg.angle_min, msg.angle_max)
		self.plots(ranges, cartesian_points)

		self.scan_idx += 1
		print("Publish feature scan message idx", self.scan_idx)
        

def main(args=None):
	rclpy.init(args=args)
	feature_extracter = FeatureExtracter()
	rclpy.spin(feature_extracter) 
	feature_extracter.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
