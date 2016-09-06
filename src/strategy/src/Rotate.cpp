#include <ros/ros.h>
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <unistd.h>

#include "strategy/Rotate.h"

using namespace std;

Rotate::Rotate() :
	debug_(false), 
	state(kROTATING_START),
	verticalLineFound(false) {

	nh_ = ros::NodeHandle("~");
	nh_.getParam("cmd_vel_topic_name", cmdVelTopicName_);
	ROS_INFO("[Rotate] PARAM cmd_vel_topic_name: %s", cmdVelTopicName_.c_str());

	nh_.getParam("debug_rotate", debug_);
	ROS_INFO("[Rotate] PARAM debug_rotate: %s", debug_ ? "TRUE" : "false");

	nh_.getParam("imu_topic_name", imuTopicName_);
	ROS_INFO("[Rotate] PARAM imu_topic_name: %s", imuTopicName_.c_str());

	nh_.getParam("line_detector_topic_name", lineDetectorTopicName_);
	ROS_INFO("[Rotate] PARAM line_detector_topic_name: %s", lineDetectorTopicName_.c_str());

	currentStrategyPub_ = nh_.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy_ = strategyHasntStarted;
	imuSub_ = nh_.subscribe(imuTopicName_.c_str(), 1, &Rotate::imuTopicCb, this);
	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>(cmdVelTopicName_.c_str(), 1);
	lineDetectorSub_ = nh_.subscribe(lineDetectorTopicName_.c_str(), 1, &Rotate::lineDetectorTopicCb, this);
}

Rotate& Rotate::Singleton() {
	static Rotate singleton_;
	return singleton_;
}

double normalizeEuler(double yaw) {
	double result = yaw;
	while (result > 360) result -= 360;
	while (result < 0) result += 360;
	return result;
}

void Rotate::imuTopicCb(const sensor_msgs::Imu& msg) {
	// yaw_ = normalizeEuler(tf::getYaw(msg.orientation));
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_ = normalizeEuler(57.29577951308229 * yaw);
	if (debug_) {
		ROS_INFO("[Rotate] yaw: %7.4f", yaw_);
	}
}

string Rotate::name() {
	return string("Rotate");
}

void Rotate::lineDetectorTopicCb(const line_detector::line_detector& msg) {
	horizontalLineFound = msg.horizontalToLeft || msg.horizontalToRight;
	horizontalToLeft = msg.horizontalToLeft;
	horizontalToRight = msg.horizontalToRight;
	verticalCurveA = msg.verticalCurveA;
	verticalCurveB = msg.verticalCurveB;
	verticalCenter = verticalCurveA + (verticalCurveB * 320);
	verticalLineFound = msg.verticalToBottom;
	lineDetectorMsgReceived = true;
	if (debug_) {
		ROS_INFO("[Rotate] toLeft: %s, toRight: %s, (hb: %d, hl: %d, len: %d), up: %s, down: %s (vb: %d, vl: %d, len: %d), a: %7.4f, b: %7.4f, vc: %7.4f",
			msg.horizontalToLeft ? "TRUE" : "false", msg.horizontalToRight ? "TRUE" : "false",
			msg.horizontalBottom, msg.horizontalLeft, msg.horizontalLength,
			msg.verticalToTop ? "TRUE" : "false", msg.verticalToBottom ? "TRUE" : "false",
			msg.verticalBottom, msg.verticalLeft, msg.verticalYlength,
			verticalCurveA, verticalCurveB, verticalCenter);
	}
}

void Rotate::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != lastReportedStrategy_) {
		lastReportedStrategy_ = strategy;
		currentStrategyPub_.publish(msg);
		ROS_INFO("[Rotate] strategy: %s", strategy.c_str());
	}
}

StrategyFn::RESULT_T Rotate::tick() {
	RESULT_T result = FATAL;
	bool keepRotating = false;
	geometry_msgs::Twist cmdVel;
	double verticalError;

	if (strategyContext.needToFollowLine) {
		if (debug_) {
			ROS_INFO("[Rotate] Still following line");
		}

		result = SUCCESS;
		return result;
	}

	if (!strategyContext.needToRotateLeft180 && !strategyContext.needToRotateRight180) {
		if (debug_) {
			ROS_INFO("[Rotate] no need to rotate");
		}

		result = SUCCESS;
		return result;
	}


	switch (state) {
		case kROTATING_START:
			startYaw_ = yaw_;
			if (strategyContext.needToRotateLeft180) {
				state = kROTATING_LEFT;
				result = RUNNING;
				goalYaw_ = normalizeEuler(startYaw_ + 90);
				if (debug_) ROS_INFO("[Rotate] start of rotate left, startYaw_: %7.4f, goalYaw_: %7.4f", startYaw_, goalYaw_);
			} else if (strategyContext.needToRotateRight180) {
				state = kROTATING_RIGHT;
				result = RUNNING;
				goalYaw_ = normalizeEuler(startYaw_ - 90);
				if (debug_) ROS_INFO("[Rotate] start of rotate right, startYaw_: %7.4f, goalYaw_: %7.4f", startYaw_, goalYaw_);
			} else {
				result = FATAL;
				if (debug_) ROS_INFO("[Rotate] BAD GOAL--neither kROTATING_LEFT nor kROTATING_RIGHT");
			}

			break;

		case kROTATING_LEFT:
			verticalError = verticalCenter - 161;
			if (verticalError < 0) verticalError = -verticalError;
			keepRotating = (verticalLineFound && (verticalError > 5));
			if (!keepRotating && !verticalLineFound) {
				if (startYaw_ > goalYaw_) {
					keepRotating = (yaw_ > (goalYaw_ + 90)) || (yaw_ < goalYaw_);
				} else {
					keepRotating = yaw_ < goalYaw_;
				}
			}

			if (debug_) {
				ROS_INFO("[Rotate] kROTATING_LEFT, keepRotating: %s, startYaw_: %7.4f, goalYaw_: %7.4f, yaw_: %7.4f, verticalLineFound: %s, verticalCenter: %7.4f, verticalError: %7.4f",
					     keepRotating ? "TRUE" : "false",
					     startYaw_,
					     goalYaw_,
					     yaw_,
					     verticalLineFound ? "TRUE" : "false",
					     verticalCenter,
					     verticalError);
			}

			if (keepRotating) {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = 1.0;
				cmdVelPub_.publish(cmdVel);
				result = RUNNING;
			} else {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = SUCCESS;
				strategyContext.needToRotateLeft180 = false;
				if (debug_) {
					ROS_INFO("[Rotate] end of rotate left, startYaw_: %7.4f, yaw_: %7.4f", startYaw_, yaw_);
				}
			}

			break;

		case kROTATING_RIGHT:
			if (startYaw_ < goalYaw_) {
				keepRotating = (yaw_ < (goalYaw_ - 90)) || (yaw_ > goalYaw_);
			} else {
				keepRotating = yaw_ > goalYaw_;
			}

			if (keepRotating) {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = -1.0;
				cmdVelPub_.publish(cmdVel);
				result = RUNNING;
				if (debug_) {
					ROS_INFO("[Rotate] rotating right, startYaw_: %7.4f, goalYaw_: %7.4f, yaw_: %7.4f", startYaw_, goalYaw_, yaw_);
				}
			} else {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = SUCCESS;
				strategyContext.needToRotateRight180 = false;
				if (debug_) {
					ROS_INFO("[Rotate] end of rotate right, startYaw_: %7.4f, yaw_: %7.4f", startYaw_, yaw_);
				}
			}

			break;
			
	}

	return result;
}

StrategyContext& Rotate::strategyContext = StrategyContext::Singleton();

const string Rotate::strategyHasntStarted = "Rotate: Strategy hasn't started";
const string Rotate::strategySuccess = "Rotate: SUCCESS";
