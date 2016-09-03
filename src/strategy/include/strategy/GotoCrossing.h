#ifndef __GOTO_CROSSING
#define __GOTO_CROSSING

#include <ros/ros.h>
#include <ros/console.h>
#include "line_detector/line_detector.h"
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
	static const string strategySuccess;

	typedef enum {
		kLOOKING_FOR_HORIZONTAL_LINE_START,
		kLOOKING_FOR_HORIZONTAL_LINE_END
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

	// Subscriber to line_detector message.
	ros::Subscriber lineDetectorSub_;

	// To help log strategy only when it changes.
	string lastReportedStrategy_;

	// Topic name containing line_detector message.`
	string lineDetectorTopicName_;

	// ROS node handle.
	ros::NodeHandle nh_;

	static StrategyContext& strategyContext;

	GotoCrossing();

	// Process line_detector messages.
	void lineDetectorTopicCb(const line_detector::line_detector& msg);

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

public:
	RESULT_T tick();

	string name();

	static GotoCrossing& Singleton();

};

#endif