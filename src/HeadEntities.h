/*
 * HeadEntities.h
 *
 *  Created on: 3 Nov 2023
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_HEADENTITIES_H_
#define FIRMWARE_SRC_HEADENTITIES_H_

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include "uRosEntities.h"
#include "uRosBridge.h"

extern "C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/color_rgba.h>
}
#if CPPUTEST_USE_NEW_MACROS
   #undef new
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#if CPPUTEST_USE_NEW_MACROS
   #include "CppUTest/MemoryLeakDetectorNewMacros.h"
#endif

#include "Head.h"
#include "EyesAgent.h"

class HeadEntities :  public uRosEntities {
public:
	HeadEntities(Head *pH,  EyesAgent *pE);
	virtual ~HeadEntities();

	/***
	 * Create the entities (Publishers)
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Return the number of entities
	 * @return
	 */
	virtual uint getCount();

	/***
	 * Return the number of handles needed by the executor
	 * @return
	 */
	virtual uint getHandles();


	/***
	 * Add subscribers, guards and timers to the executor
	 * @param executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);

	/***
	 * Call back on a publish to show msg has been completed.
	 * Can be used to free up allocated memory
	 * @param msg - ROS Msg
	 * @param args - Arguments passed into the publish step
	 * @param status -
	 */
	virtual void pubComplete(void *msg, void *args, uRosPubStatus status);

protected:
	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);

private:
	/***
	 * Setup the Ros Msg Structure;
	 */
	void setupRosMsgs();

	/***
	 * Convert Quaternion to Euler
	 * @param q - Quaternion
	 * @return Euler in vector: Roll, Pitch, Yaw
	 */
	Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q);

	/***
	 * Handle move request
	 * @param msg
	 */
	void handlePose(geometry_msgs__msg__PoseStamped * pPoseMsg);


	/***
	 * Handle eye colour
	 * RGB 0 to 1.
	 * @param pEyesMsg
	 */
	void handleEyes(std_msgs__msg__ColorRGBA * pEyesMsg);

	Head *				pHead = NULL;
	EyesAgent *		pEyes = NULL;

	rcl_subscription_t 												xSubPose;
	uRosSubContext_t   											xSubPoseContext;
	geometry_msgs__msg__PoseStamped 	xPoseMsg;

	rcl_subscription_t 												xSubEyes;
	uRosSubContext_t   											xSubEyesContext;
	std_msgs__msg__ColorRGBA						xEyesMsg;


	rcl_publisher_t xPubJoint;
	uint xCount = 0;

	sensor_msgs__msg__JointState xJointStateMsg;
	const char *pPitchName = "pitch_to_base";
	const char *pYawName = "yaw_to_base";

};

#endif /* FIRMWARE_SRC_HEADENTITIES_H_ */
