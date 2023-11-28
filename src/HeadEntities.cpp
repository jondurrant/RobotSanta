/*
 * HeadEntities.cpp
 *
 *  Created on: 3 Nov 2023
 *      Author: jondurrant
 */

#include "HeadEntities.h"

using namespace Eigen;

HeadEntities::HeadEntities(Head *pH, EyesAgent * pE) {
	pHead = pH;
	pEyes = pE;
	setupRosMsgs();

}

HeadEntities::~HeadEntities() {
	// TODO Auto-generated destructor stub
}

/***
* Create the entities (Publishers)
* @param node
* @param support
*/
void HeadEntities::createEntities(rcl_node_t *node, rclc_support_t *support){
	rclc_publisher_init_default(
			&xPubJoint,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			"joint_states");

	rclc_subscription_init_default(
		  &xSubPose,
		  node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
		  "do_pose");

	rclc_subscription_init_default(
			  &xSubEyes,
			  node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
			  "do_eyes");

	xCount = 3;
}

/***
* Destroy the entities
* @param node
* @param support
*/
void HeadEntities::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	xCount = 0;
	rcl_publisher_fini(&xPubJoint, 			node);
	rcl_subscription_fini(&xSubPose, 	node);
	rcl_subscription_fini(&xSubEyes, 	node);
}

/***
* Return the number of entities
* @return
*/
uint HeadEntities::getCount(){
	return xCount;
}

/***
* Return the number of handles needed by the executor
* @return
*/
uint HeadEntities::getHandles(){
	return 2;
}


/***
* Add subscribers, guards and timers to the executor
* @param executor
*/
void HeadEntities::addToExecutor(rclc_executor_t *executor){
	buildContext(&xSubPoseContext, NULL);
	rclc_executor_add_subscription_with_context(
			executor,
			&xSubPose,
			&xPoseMsg,
			uRosEntities::subscriptionCallback,
			&xSubPoseContext,
			ON_NEW_DATA);

	buildContext(&xSubEyesContext, NULL);
	rclc_executor_add_subscription_with_context(
				executor,
				&xSubEyes,
				&xEyesMsg,
				uRosEntities::subscriptionCallback,
				&xSubEyesContext,
				ON_NEW_DATA);
}

/***
* Call back on a publish to show msg has been completed.
* Can be used to free up allocated memory
* @param msg - ROS Msg
* @param args - Arguments passed into the publish step
* @param status -
*/
void HeadEntities::pubComplete(void *msg, void *args, uRosPubStatus status){

}


/***
* Handle subscription msg
* @param msg
* @param localContext
*/
void HeadEntities::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){
	if (context == &xSubPoseContext){
		geometry_msgs__msg__PoseStamped *pPoseMsg;
		pPoseMsg = (geometry_msgs__msg__PoseStamped *) msg;
		handlePose(pPoseMsg);
	} else if(context == &xSubEyesContext){
		std_msgs__msg__ColorRGBA * pEyesMsg;
		pEyesMsg = (std_msgs__msg__ColorRGBA  *) msg;
		handleEyes(pEyesMsg);
	}

}

/***
 * Handle eye colour
 * RGB 0 to 1.
 * @param pMsg
 */
void HeadEntities::handleEyes(std_msgs__msg__ColorRGBA * pEyesMsg){
	uint8_t r;
	uint8_t g;
	uint8_t b;

	r = 	ceil(pEyesMsg->r * 255.0);
	g = 	ceil(pEyesMsg->g * 255.0);
	b = 	ceil(pEyesMsg->b * 255.0);

	if (pEyes != NULL){
		pEyes->setColour(r,g,b);
	}
}

/***
 * Handle move request
 * @param msg
 */
void HeadEntities::handlePose(geometry_msgs__msg__PoseStamped * pPoseMsg){
		int targetPos;

		//Calculate target angle
		geometry_msgs__msg__Quaternion *q = &pPoseMsg->pose.orientation;

		//X = Roll
		//Y = Pitch
		//Z = Yaw

		//Roll pitch and yaw in Radians
		Quaterniond quad(
				q->w,
				q->x,
				q->y,
				q->z
				);
		quad.normalize();
		Vector3d euler = quaternionToEuler(quad);
		//printf("Quad %f,%f,%f,%f\n", quad.x(), quad.y(), quad.z(), quad.w());
		double yaw = euler[2];
		double pitch = euler[1];
		printf("Rol %f Pitch %f Yaw %f\n", euler[0],  pitch,  yaw);

		//Calc difference between timestamp and now
		int64_t now = rmw_uros_epoch_nanos();
		uint32_t sec = now / 1000000000;
		uint32_t nanosec = now % 1000000000;
		double dif = pPoseMsg->header.stamp.sec - sec;
		double ndif = pPoseMsg->header.stamp.nanosec % 1000000000;
		ndif = (ndif - nanosec) / 1000.0 / 1000.0 /1000.0;
		//If timestamp in past the 0 time
		if (dif < 0.0){
			dif = 0.0;
		}
		if ((dif == 0.0) && (ndif < 0.0)){
			dif = 0.0;
		} else {
			dif = dif + ndif;
		}
		uint ticks = pdMS_TO_TICKS(dif * 1000);
		//printf("Time dif %f (Ticks %d)\n", dif, ticks);

		if (pHead != NULL){
			pHead->moveRad(  pitch,  yaw,  ticks);
		}
}


/***
 * Setup the Ros Msg Structure;
 */
void HeadEntities::setupRosMsgs(){
	//Setup the joint state msg
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.position, 2);
	xJointStateMsg.position.data[0] = 0;
	xJointStateMsg.position.data[1] = 0;
	xJointStateMsg.position.size = 2;
	xJointStateMsg.position.capacity = 2;
	rosidl_runtime_c__String__Sequence__init(&xJointStateMsg.name, 2);
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[0], pPitchName)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[1], pYawName)){
		printf("ERROR: Joined assignment failed\n");
	}
	xJointStateMsg.name.size=2;
	xJointStateMsg.name.capacity=2;

	//Setup sub msgs
	geometry_msgs__msg__PoseStamped__init(&xPoseMsg);
	std_msgs__msg__ColorRGBA__init(&xEyesMsg);
}


/***
 * Convert Quaternion to Euler
 * @param q - Quaternion
 * @return Euler in vector: Roll, Pitch, Yaw
 */
Eigen::Vector3d HeadEntities::quaternionToEuler(const Eigen::Quaterniond& q) {
    Vector3d angles;    //roll pitch  yaw
    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}
