#ifndef __ROTATE
#define __ROTATE

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include "strategy/StrategyContext.h"
#include "strategy/StrategyFn.h"
#include <string>

class Rotate : public StrategyFn {
private:
	static const string strategyHasntStarted;
	static const string strategySuccess;

	typedef enum {
		kROTATING_START,
		kROTATING_LEFT,
		kROTATING_RIGHT
	} STATE;
	
	STATE state;

	// Print debug info?
	bool debug_;

	// Topic to publish robot movements.
	ros::Publisher cmdVelPub_;

	// Topic name containing cmd_vel message.
	string cmdVelTopicName_;

	// Topic to publish current strategy.
	ros::Publisher currentStrategyPub_;

	double goalYaw_;

	// Subscriber to IMU message.
	ros::Subscriber imuSub_;

	// Topic name containing imu message.
	string imuTopicName_;

	// To help log strategy only when it changes.
	string lastReportedStrategy_;

	// ROS node handle.
	ros::NodeHandle nh_;

	// Yaw at start of turn.
	double startYaw_;

	// From IMU callback.
	double yaw_;

	static StrategyContext& strategyContext;

	Rotate();

	// Process IMU messages.
	void imuTopicCb(const sensor_msgs::Imu& msg);

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

public:
	StrategyFn::RESULT_T tick();

	string name();

	static Rotate& Singleton();

};

#endif