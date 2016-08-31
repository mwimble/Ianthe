#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <unistd.h>

#include "strategy/GotoCrossing.h"

using namespace std;

GotoCrossing::GotoCrossing() :
	atLine_(false), 
	debug_(false), 
	horizontalLineFound(false), 
	sawHorizontalLine(false),
	state(kLOOKING_FOR_HORIZONTAL_LINE_START),
	verticalLineFound(false) {

	ros::param::param<std::string>("cmd_vel_topic_name", cmdVelTopicName_, "/cmd_vel");
	ROS_INFO("[GotoCrossing] PARAM cmd_vel_topic_name: %s", cmdVelTopicName_.c_str());

	ros::param::param<bool>("debug_goto_crossing", debug_, "false");
	ROS_INFO("[GotoCrossing] PARAM debug_goto_crossing: %s", debug_ ? "TRUE" : "false");

	ros::param::param<std::string>("line_detector_topic_name", lineDetectorTopicName_, "/lineDetect");
	ROS_INFO("[GotoCrossing] PARAM line_detector_topic_name: %s", lineDetectorTopicName_.c_str());

	currentStrategyPub_ = nh_.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy_ = strategyHasntStarted;
	lineDetectorSub_ = nh_.subscribe(lineDetectorTopicName_.c_str(), 1, &GotoCrossing::lineDetectorTopicCb, this);
	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

GotoCrossing& GotoCrossing::Singleton() {
	static GotoCrossing singleton_;
	return singleton_;
}

void GotoCrossing::lineDetectorTopicCb(const line_detector::line_detector& msg) {
	// horizontalLineWidth = (msg.horizontalLowerLeftY - msg.horizontalUpperRightY) / 2;
	// horizontalLineY = msg.horizontalUpperRightY + horizontalLineWidth;
	// horizontalLineLength = msg.horizontalUpperRightX - msg.horizontalLowerLeftX;
	// horizontalLineFound = (horizontalLineWidth >= kMIN_HORIZONTAL_LINE_WIDTH) &&
	//                       (horizontalLineWidth <= kMAX_HORIZONTAL_LINE_WIDTH) &&
	//                       (horizontalLineLength >= kMIN_HORIZONTAL_LINE_LENGTH);
	// if (debug_) {
	// 	ROS_INFO("[GotoCrossing] horizontalLineFound: %s, horizontalLineY: %d, horizontalLineWidth: %d, horizontalLineLength: %d, llx: %d, lly: %d, urx: %d, ury: %d",
	// 	         horizontalLineFound ? "TRUE" : "false",
	// 	         horizontalLineY,
	// 	         horizontalLineWidth,
	// 	         horizontalLineLength,
	// 	         msg.horizontalLowerLeftX,
	// 	         msg.horizontalLowerLeftY,
	// 	         msg.horizontalUpperRightX,
	// 	         msg.horizontalUpperRightY);
	// }
	horizontalLineFound = msg.horizontalToLeft || msg.horizontalToRight;
	verticalLineFound = msg.verticalToBottom;
}

string GotoCrossing::name() {
	return string("GotoCrossing");
}

void GotoCrossing::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != lastReportedStrategy_) {
		lastReportedStrategy_ = strategy;
		currentStrategyPub_.publish(msg);
		ROS_INFO("[GotoCrossing] strategy: %s", strategy.c_str());
	}
}

StrategyFn::RESULT_T GotoCrossing::tick() {
	RESULT_T result = FATAL;
	geometry_msgs::Twist cmdVel;

	switch (state) {
		case kLOOKING_FOR_HORIZONTAL_LINE_START:
			publishCurrentStragety(strategyLookingForHorizontalLineStart);
			if (!verticalLineFound) {
				ROS_INFO("[GotoCrossing] Vertical line not found");
				result = FAILED;
			} else if (sawHorizontalLine) {
				state = kLOOKING_FOR_HORIZONTAL_LINE_END;
				// Move until not found horizontal line.
				cmdVel.linear.x = 0.1;
				cmdVel.linear.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = RUNNING;
			} else {
				// Move until found horizontal line.
				geometry_msgs::Twist cmdVel;
				cmdVel.linear.x = 0.1;
				cmdVel.linear.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = RUNNING;
			}

			break;

		kLOOKING_FOR_HORIZONTAL_LINE_END:
			publishCurrentStragety(strategyLookingForHorizontalLineEnd);
			if (!sawHorizontalLine) {
				// Move until no longer see horizontal line.
				geometry_msgs::Twist cmdVel;
				cmdVel.linear.x = 0.1;
				cmdVel.linear.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = RUNNING;
			} else {
				// Done.
				publishCurrentStragety(strategySuccess);
				// Stop.
				geometry_msgs::Twist cmdVel;
				cmdVel.linear.x = 0.0;
				cmdVel.linear.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = SUCCESS;
			}

			break;
	}

	return result;
}

StrategyContext& GotoCrossing::strategyContext = StrategyContext::Singleton();

const string GotoCrossing::strategyHasntStarted = "GotoCrossing: Strategy hasn't started";
const string GotoCrossing::strategyLookingForHorizontalLineEnd = "GotoCrossing: Looking for horizontal line end";
const string GotoCrossing::strategyLookingForHorizontalLineStart = "GotoCrossing: Looking for horizontal line start";
const string GotoCrossing::strategySuccess = "GotoCrossing: SUCCESS";
