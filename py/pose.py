#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import sys

class PoseNode(Node):
	def __init__(self):
		super().__init__("Pub_Node")
		self.pose_publisher = self.create_publisher(PoseStamped, "/do_pose", 10)
		self.pose_timer = self.create_timer(10.0, self.publish_pose)
		self.yaw=	[0.0, 0.0, 0.0,  0.0, 3.14/2.0-0.2, 3.14/(-2.0)+0.2, 0.0]
		self.pitch=	[0.0, 0.2, -0,2, 0.0, 0.2,      -0.2,        0.0]
		self.currentPose = 0;

	def publish_pose(self):
		#self.currentPose = 3
		z =self.yaw[self.currentPose]
		y =self.pitch[self.currentPose]
		#z = 1.5
		# y = 0.0
		goal = PoseStamped()
		goal.header.frame_id = "/base_link"
		d = Duration(seconds=1)
		t = self.get_clock().now() + d
		goal.header.stamp = t.to_msg()
		
		(goal.pose.orientation.x,
		goal.pose.orientation.y,
		goal.pose.orientation.z,
		goal.pose.orientation.w) = quaternion_from_euler(0.0, y, z)
		print("%d Pitch %f Yaw %f  (%f,%f,%f,%f)" %(self.currentPose,
				y,z, 
				goal.pose.orientation.x,
				goal.pose.orientation.y,
				goal.pose.orientation.z,
				goal.pose.orientation.w						
		))
		(roll,pitch,yaw)=euler_from_quaternion(
			[goal.pose.orientation.x,
			goal.pose.orientation.y,
			goal.pose.orientation.z,
			goal.pose.orientation.w])
		print("%d Euler %f, %f, %f" %(self.currentPose, roll, pitch, yaw))
		self.pose_publisher.publish(goal)
		
		self.currentPose += 1
		if (self.currentPose >= len(self.yaw)):
			self.currentPose = 0
			
	def pose(self, y, z):
		#self.currentPose = 3
		goal = PoseStamped()
		goal.header.frame_id = "/base_link"
		d = Duration(seconds=1)
		t = self.get_clock().now() + d
		goal.header.stamp = t.to_msg()
		
		(goal.pose.orientation.x,
		goal.pose.orientation.y,
		goal.pose.orientation.z,
		goal.pose.orientation.w) = quaternion_from_euler(0.0, y, z)
		print("%d Pitch %f Yaw %f  (%f,%f,%f,%f)" %(self.currentPose,
				y,z, 
				goal.pose.orientation.x,
				goal.pose.orientation.y,
				goal.pose.orientation.z,
				goal.pose.orientation.w						
		))
		(roll,pitch,yaw)=euler_from_quaternion(
			[goal.pose.orientation.x,
			goal.pose.orientation.y,
			goal.pose.orientation.z,
			goal.pose.orientation.w])
		print("%d Euler %f, %f, %f" %(self.currentPose, roll, pitch, yaw))
		self.pose_publisher.publish(goal)
		
		self.currentPose += 1
		if (self.currentPose >= len(self.yaw)):
			self.currentPose = 0

def main(args=None, pitch=0.0, yaw=0.0):

	rclpy.init(args=args)
	node = PoseNode()
	#rclpy.spin(node)
	node.pose(pitch, yaw)
	
	rclpy.shutdown()

if __name__ == "__main__":
	if len(sys.argv) != 3:
		print("Usage pose.py <pitch> <yaw>")
	else:
		pitch = float(sys.argv[1])
		yaw = float(sys.argv[2])
		main(None, pitch, yaw)