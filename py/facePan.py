#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import cv2
import time

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import ColorRGBA


class PoseNode(Node):
	def __init__(self):
		super().__init__("Pub_Node")
		self.pose_publisher = self.create_publisher(PoseStamped, 	"/do_pose", 10)
		self.eyes_publisher = self.create_publisher(ColorRGBA, 		"/do_eyes", 10)
		
		cascasdepath = "haarcascade_frontalface_default.xml"
		self.face_cascade = cv2.CascadeClassifier(cascasdepath)

		self.video_capture = cv2.VideoCapture(0)
		self.width  = self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)   
		self.height = self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT) 
		
		self.camWidRad = 1.15192
		self.camHeightRad = 0.71
		self.moves = 0
		
		self.camYaw = 0.0
		self.camPitch = 0.0
		
		self.patrolPos = 0
		self.patrolPitch = [0.2, 0.6, 0.2, 0.0, 0.2, 0.,  0.2,  0.6]
		self.patrolYaw =   [0.0, 0.6, 1.0,  1.0, 0.0, -0.6, -1.0, -0.6]
		self.detectTimeout = 10
		
		self.maxYaw = 1.0
		self.minYaw = -1.0
		self.maxPitch = 0.4
		self.minPitch = -0.1
		
	def faceRads(self):
		camYaw   = 0.0
		camPitch = 0.0
		ret, image = self.video_capture.read()
		if not ret:
		    return
		
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = self.face_cascade.detectMultiScale(
		    gray,
		    scaleFactor = 1.2,
		    minNeighbors = 5,
		    minSize = (30,30)
		    )

		#print("The number of faces found = ", len(faces))
		num_faces = len(faces)
		
		for (x,y,w,h) in faces:
		    cv2.rectangle(image, (x,y), (x+w, y+h), (0, 255, 0), 2)
		    fx = x+(w/2)
		    fy = y+(h/2)
		    camYaw = (self.camWidRad - (fx/self.width)*self.camWidRad) - (self.camWidRad/2)
		    camPitch = (self.camHeightRad - (fy/self.height) * self.camHeightRad) - (self.camHeightRad/2) 
		    #reducing pitch to 80%
		    camPitch = camPitch * 0.8
		    #print("Face at %d, %d.  Yaw %f"%(fx,fy, camYaw))
		
		#cv2.imshow("Faces found", image)
		#if cv2.waitKey(1) & 0xFF == ord('q'):
		#    return (False, 0,0)
		
		if (num_faces == 0):
			return (False, 0,0)
		
		return (True, camPitch, camYaw)

	def publish_pose(self):
		num_faces = 0
		deltaPitch=0.0
		deltaYaw=0.0
		
		if (self.moves != 0):
			(detect, deltaPitch, deltaYaw) = self.faceRads()
			if (not detect):
				return
		
		#if (self.moves > 10):
		#	return
		
		self.camPitch = self.camPitch + deltaPitch
		self.camYaw = self.camYaw + deltaYaw

		goal = PoseStamped()
		goal.header.frame_id = "/base_link"
		d = Duration(seconds=0.2)
		t = self.get_clock().now() + d
		goal.header.stamp = t.to_msg()
		
		(goal.pose.orientation.x,
		goal.pose.orientation.y,
		goal.pose.orientation.z,
		goal.pose.orientation.w) = quaternion_from_euler(0.0, self.camPitch, self.camYaw)
		print("Pitch %f Yaw %f  (%f,%f,%f,%f)" %(
				self.camPitch,
				self.camYaw, 
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
		print("Euler %f, %f, %f" %( roll, pitch, yaw))
		self.pose_publisher.publish(goal)
		self.moves = self.moves + 1
	
	def pose(self, pitch, yaw, sec):
		if (pitch > self.maxPitch):
			y= self.maxPitch
		elif (pitch < self.minPitch):
			y= self.minPitch
		else:
			y= pitch
			
		if (yaw > self.maxYaw):
			z = self.maxYaw
		elif (yaw < self.minYaw):
			z = self.minYaw
		else:
			z = yaw
		
		
		goal = PoseStamped()
		goal.header.frame_id = "/base_link"
		d = Duration(seconds=sec)
		t = self.get_clock().now() + d
		goal.header.stamp = t.to_msg()
		
		(goal.pose.orientation.x,
		goal.pose.orientation.y,
		goal.pose.orientation.z,
		goal.pose.orientation.w) = quaternion_from_euler(0.0, y, z)
		#print("Pitch %f Yaw %f  (%f,%f,%f,%f)" %(
		#		y,z, 
		#		goal.pose.orientation.x,
		#		goal.pose.orientation.y,
		#		goal.pose.orientation.z,
		#		goal.pose.orientation.w						
		#))
		(roll,pitch,yaw)=euler_from_quaternion(
			[goal.pose.orientation.x,
			goal.pose.orientation.y,
			goal.pose.orientation.z,
			goal.pose.orientation.w])
		#print("Euler %f, %f, %f" %(roll, pitch, yaw))
		self.pose_publisher.publish(goal)
		return(y, z)

	def eyes(self, r, g, b):
		eyesMsg = ColorRGBA()
		eyesMsg.r = r;
		eyesMsg.g = g;
		eyesMsg.b = b;
		
		self.eyes_publisher.publish(eyesMsg)


		
	def go(self, startPitch, startYaw):
		pitch = startPitch
		yaw = startYaw
		deltaPitch = 0.0
		deltaYaw = 0.0
		self.detectTime = time.time()
		
		self.eyes(1.0, 1.0, 1.0)
		
		
		detect = False
		while (not detect):
			(detect, deltaPitch, deltaYaw) = self.faceRads()
			if (not detect):
				if (time.time() - self.detectTime  > self.detectTimeout):
					deltaPich = 0.0
					deltaYaw = 0.0
					(detect, pitch, yaw) = self.patrol()

		
		
		while True:
			delta = max(abs(deltaPitch), abs(deltaYaw))
			if (delta > 0.001):
				yaw = yaw + deltaYaw
				pitch = pitch + deltaPitch
				#print("delta %f yaw %f"%(deltaYaw, yaw))
				(pitch, yaw) = self.pose(pitch, yaw, 0.5)
				print("Face %.3f, %.3f"%(pitch, yaw))
				self.eyes(0.0, 1.0, 0.0)
				self.detectTime = time.time()
			else:
				#print("Time %d", time.time() - self.detectTime )
				if (time.time() - self.detectTime  > self.detectTimeout): 
					deltaPich = 0.0
					deltaYaw = 0.0
					(detect, pitch, yaw) = self.patrol()
					continue
				
			deltaPitch = 0.0
			deltaYaw = 0.0
			for i in range(20):
				(detect, y, z) = self.faceRads()
				if (detect):
					deltaPitch = y
					deltaYaw = z
			
		
	def patrol(self):
		self.eyes(0.0, 0.0, 1.0)
		i = self.patrolPos
		(y,z)=self.pose(self.patrolPitch[i], self.patrolYaw[i], 1.2)
		self.detectTime = time.time()
		self.patrolPos = self.patrolPos + 1
		if (self.patrolPos >= len(self.patrolYaw)):
			self.patrolPos = 0
		print("Patrol %.3f, %.3f"%(y,z))
		return (True, y, z)



def main(args=None):
	rclpy.init(args=args)
	node = PoseNode()
	#rclpy.spin(node)
	
	node.pose(0.6, 0.0, 1.0)
	time.sleep(2)
	
	node.go(0.6, 0.0)
	
	
	rclpy.shutdown()

if __name__ == "__main__":
	main()