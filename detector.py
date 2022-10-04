import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt

class FeatureExtracter(Node):
	def __init__(self):
		super().__init__('feature_extracter')
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1) 
		self.scan_idx = 0
		self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_policy)
		self.publisher_ = self.create_publisher(LaserScan, '/feature_scan', 10)
		self.corner_publisher = self.create_publisher(LaserScan, '/detect_corners', 10)
		self.line_publisher = self.create_publisher(LaserScan, '/detect_lines', 10)

		
		
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
	#	plt.show()
		
	def find_closest(self, array, value):
		array = np.asarray(array)
		index = (np.abs(array - value)).argmin()
		return index
		
	def detect_corners(self, cartesian_points):
		corners_array = []
		index = []
		output = []
		r = 0
		for i in range(0,351):
		
			a = np.array(cartesian_points[i])
			b = np.array(cartesian_points[i+4])
			c = np.array(cartesian_points[i+8])
			ba = a - b
			bc = c - b
			
			cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
			angle = np.degrees(np.arccos(cosine_angle))
			
			if angle < 100:
				corners_array.append(angle)
				index.append(i)
			elif len(corners_array) > 0 and angle > 120:
				output.append(index[self.find_closest(corners_array, 90)])
				corners_array = []
				index = []
		
		return output
		
			
	def detect_lines(self, cartesian_points):
		
		return_index = set()
		for i in range(45):
			lineA = []
			lineB = []
			for j in range(8):
				lineA.append(cartesian_points[i*8+j])
				if i == 44:
					lineB.append(cartesian_points[j])
				else:
					lineB.append(cartesian_points[(i+1)*8+j])
			
			lineA = np.array(lineA)
			lineB = np.array(lineB)
			lineA_c = np.polyfit(lineA[:,0], lineA[:,1], 1)
			lineB_c = np.polyfit(lineB[:,0], lineB[:,1], 1)
			if abs(lineA_c[0] - lineB_c[0]) < 0.2 and abs (lineA_c[1] - lineB_c[1]) < 0.2:
				return_index.add(i*8)
				if i == 44:
					return_index.add(0)
				else:
					return_index.add((i+1)*8)
			else:
				print(i, abs(lineA_c[0] - lineB_c[0]))
				print(i, abs(lineA_c[1] - lineB_c[1]))
		
		return return_index
		
		
	def scan_callback(self,msg):

		scan = LaserScan()
		scan.header.stamp = msg.header.stamp
		scan.header.frame_id = msg.header.frame_id
		scan.angle_min = msg.angle_min
		scan.angle_max = msg.angle_max
		scan.angle_increment = msg.angle_increment
		scan.time_increment = msg.time_increment
		scan.range_min = msg.range_min
		scan.range_max = msg.range_max 
		scan.ranges = msg.ranges
		
		cartesian_points = self.polar_to_cartesian_coordinate(msg.ranges, msg.angle_min, msg.angle_max)
		corners = self.detect_corners(cartesian_points)
		lines = self.detect_lines(cartesian_points)
		
		corner_ranges = [0.0 for j in range(360)]
		for i in corners:
			for j in range(9):
				corner_ranges[i+j] = msg.ranges[i+j]
		line_ranges = [0.0 for j in range(360)]
		for i in lines:
			for j in range(8):
				line_ranges[i+j] = msg.ranges[i+j]
		
		self.plots(msg.ranges, cartesian_points)
		self.detect_corners(cartesian_points)
		self.detect_lines(cartesian_points)
		
		self.publisher_.publish(scan)
		self.scan_idx += 1
		print("Publish feature scan message idx", self.scan_idx)
		
		scan.ranges = corner_ranges
		self.corner_publisher.publish(scan)
		
		scan.ranges = line_ranges
		self.line_publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
