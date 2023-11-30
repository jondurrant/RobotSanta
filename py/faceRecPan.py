#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import cv2
import time
import pickle
import face_recognition

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import ColorRGBA


class PoseNode(Node):
	def __init__(self):
		super().__init__("Pub_Node")
		self.pose_publisher = self.create_publisher(PoseStamped, 	"/do_pose", 10)
		self.eyes_publisher = self.create_publisher(ColorRGBA, 		"/do_eyes", 10)
		
		cascasdepath = "haarcascade_frontalface_alt2.xml"
		self.face_cascade = cv2.CascadeClassifier(cascasdepath)

		self.video_capture = cv2.VideoCapture(0)
		self.scale = 1.0
		self.width  = int(self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH) * self.scale) 
		self.height = int(self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT) * self.scale)
		print("DIM (%d, %d)"%(self.width, self.height))
		
		self.camWidRad = 1.15192
		self.camHeightRad = 0.71
		self.moves = 0
		
		self.camYaw = 0.0
		self.camPitch = 0.0
		
		self.patrolPos = 0
		self.patrolPitch = [0.2, 0.6, 0.2, 0.3, 0.2, 0.2,  0.3,  0.6]
		self.patrolYaw =   [0.0, 0.6, 1.0,  1.0, 0.0, -0.6, -1.0, -0.6]
		self.detectTimeout = 10
		
		self.maxYaw = 1.0
		self.minYaw = -1.0
		self.maxPitch = 0.4
		self.minPitch = -0.1
		
		self.data = pickle.loads(open('face_enc', "rb").read())
		
	def faceRads(self):
		camYaw   = 0.0
		camPitch = 0.0
		ret, image = self.video_capture.read()
		if not ret:
		    return (False, 0,0)
		   
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		
		faces = self.face_cascade.detectMultiScale(
		    gray,
		    scaleFactor = 1.2,
		    minNeighbors = 5,
		    minSize = (60,60)
		    )

		#print("The number of faces found = ", len(faces))
		num_faces = len(faces)
		
		for (x,y,w,h) in faces:
		    fx = x+(w/2)
		    fy = y+(h/2)
		    camYaw = (self.camWidRad - (fx/self.width)*self.camWidRad) - (self.camWidRad/2)
		    camPitch = (self.camHeightRad - (fy/self.height) * self.camHeightRad) - (self.camHeightRad/2) 
		    #reducing pitch to 80%
		    camPitch = camPitch * 0.8
		    #print("Face at %d, %d.  Yaw %f"%(fx,fy, camYaw))
		    return (True, camPitch, camYaw)
		
		return (False, 0,0)

	def faceDetect(self):
		ret, image = self.video_capture.read()
		if not ret:
		    return False
		
		rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		encodings = face_recognition.face_encodings(rgb)
		for encoding in encodings:
			matches = face_recognition.compare_faces(self.data["encodings"], encoding)
			if True in matches:
				return True
		return False
	
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
				self.detectTime = time.time()
				
				if (self.faceDetect()):
					self.eyes(0.0, 0.0, 1.0)
				else:
					self.eyes(0.0, 1.0, 0.0)
			else:
				#print("Time %d", time.time() - self.detectTime )
				if (time.time() - self.detectTime  > self.detectTimeout): 
					deltaPich = 0.0
					deltaYaw = 0.0
					(detect, pitch, yaw) = self.patrol()
					continue
				
			deltaPitch = 0.0
			deltaYaw = 0.0
			for i in range(7):
				(detect, y, z) = self.faceRads()
				if (detect):
					deltaPitch = y
					deltaYaw = z

			
		
	def patrol(self):
		self.eyes(1.0, 1.0, 1.0)
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