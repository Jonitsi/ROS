import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

class FeatureExtracter(Node):
	def __init__(self):
		super().__init__('feature_extracter')
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1) 
		self.scan_idx = 0
		self.subscription_laserscan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_policy)
		self.subscription_odometry = self.create_subscription(Odometry, '/odom', self.scan_callback, qos_profile=qos_policy)
		self.map = np.zeros([100,100])
		self.odometry = odometry()
		self.map.fill(-1)

	def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max):
		angle_step = (angle_max - angle_min) / len(ranges)
		angle = 0
		points = []
		for range in ranges:
			x = range * np.cos(angle)
			y = range * np.sin(angle)
			angle += angle_step
			points.append([x,y])
		
		return points

	def bresenham_points (p0, p1):
	
		point_list = []  # We will fill this list with all points in between p0 and p1

		x0, y0 = p0[0], p0[1]
		x1, y1 = p1[0], p1[1]

		dx = abs(x1-x0)
		dy = abs(y1-y0)
		if x0 < x1:
			sx = 1
		else:
			sx = -1

		if y0 < y1:
        		sy = 1
		else:
			sy = -1

		err = dx-dy
    
	while True:
		#print("{}, {}".format(x0, y0))
		point_list.append([x0, y0])
		if x0 == x1 and y0 == y1:
			break # This means we have finished, so we break the loop
            
		e2 = 2*err
		if e2 > -dy:
			# overshot in the y direction
			err = err - dy
			x0 = x0 + sx
		if e2 < dx:
			# overshot in the x direction
			err = err + dx
			y0 = y0 + sy
    
	point_list.pop(0)
	point_list.pop(len(point_list) -1)
	
	def odom_callback(self, odom):
		self.odom = 

	def scan_callback(self, msg):
		
		scan = LaserScan()
		scan.header.stamp = msg.header.stamp
		scan.header.frame_id = msg.header.frame_id
		scan.angle_min = msg.angle_min
		scan.angle_max = msg.angle_max
		scan.angle_increment = msg.angle_increment
		scan.time_increment = msg.time_increment
		scan.range_min = msg.range_min
		scan.range_max = msg.range_max 
		scan.ranges = msg.ranges'
		
		cartesian_points = self.polar_to_cartesian_coordinate(msg.ranges, msg.angle_min, msg.angle_max)
		
def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()	
