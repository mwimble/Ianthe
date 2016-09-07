#ifndef __GOTO_CROSSING
#define __GOTO_CROSSING

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <line_detector/line_detector.h>
#include <nav_msgs/Odometry.h>
#include "strategy/StrategyContext.h"
#include "strategy/StrategyFn.h"
#include <string>

class GotoCrossing : public StrategyFn {
private:
	static const int kMAX_HORIZONTAL_LINE_WIDTH = 60;
	static const int kMIN_HORIZONTAL_LINE_WIDTH = 15;
	static const int kMIN_HORIZONTAL_LINE_LENGTH = 160;
	static const string strategyHasntStarted;
	static const string strategyLookingForHorizontalLineEnd;
	static const string strategyLookingForHorizontalLineStart;
	static const string strategyMovingToCenteringPosition;
	static const string strategySuccess;

	typedef enum {
		kLOOKING_FOR_HORIZONTAL_LINE_START,
		kLOOKING_FOR_HORIZONTAL_LINE_END,
		kMOVING_TO_CENTERING_POSITION
	} STATE;

	STATE state;

	// For moving to a crossing line.
	int horizontalLineWidth;
	int horizontalLineY;
	int horizontalLineLength;

	bool horizontalLineFound;
	bool verticalLineFound;
	bool sawHorizontalLine;

	bool horizontalToLeft;
	bool horizontalToRight;
	bool lineDetectorMsgReceived;
	double verticalIntercept;
	line_detector::line_detector lastLineDetectorMsg;


	// Positioned at a turning line?
	bool atLine_;

	// Print debug info?
	bool debug_;

	// Topic to publish robot movements.
	ros::Publisher cmdVelPub_;

	// Topic name containing cmd_vel message.
	string cmdVelTopicName_;

	// Topic to publish current strategy.
	ros::Publisher currentStrategyPub_;

	// Distance traveled to when first discovering line end.
	double distanceTraveledToLineEnd_;

	// Pose at end of travel.
	geometry_msgs::Pose endingPose_;

	// Last odometry message received.
	nav_msgs::Odometry lastOdomMsg_;

	// Subscriber to line_detector message.
	ros::Subscriber lineDetectorSub_;

	// To help log strategy only when it changes.
	string lastReportedStrategy_;

	// Topic name containing line_detector message.`
	string lineDetectorTopicName_;

	// ROS node handle.
	ros::NodeHandle nh_;

	// Has any odometry message been received?
	bool odometryMessageReceived_;

	// Subscriber to Odometry message.
	ros::Subscriber odometrySub_;

	static StrategyContext& strategyContext;

	// Pose at start of travel.
	geometry_msgs::Pose startingPose_;

	GotoCrossing();

	// Process line_detector messages.
	void lineDetectorTopicCb(const line_detector::line_detector& msg);

	// Process Odometry messages.
	void odomTopicCb(const nav_msgs::Odometry& msg);

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

public:
	RESULT_T tick();

	string name();

	static GotoCrossing& Singleton();

};

#endif