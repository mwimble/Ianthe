#ifndef __STRATEGY_CONTEXT
#define __STRATEGY_CONTEXT

#include <sys/time.h>
#include <geometry_msgs/Twist.h>

class StrategyContext {
private:
	// Singleton pattern.
	StrategyContext() {};
	StrategyContext(StrategyContext const&) {};
	StrategyContext& operator=(StrategyContext const&) {}

public:
	static StrategyContext& Singleton();

	/*
	* For fetching the precached sample.
	*/
	// geometry_msgs::Twist cmdVel;				// For use in repeating command.

	bool atGoal;								// True => Arrived at the goal.

	bool lookingForHome;						// True => Trying to return home.

	bool lookingForPrecachedSample;				// True => Trying to find the precached sample;

	// double minX;								// Used to dynamically adjust min orientation cmd_vel.

	// double minZ;								// Used to dynamically adjust min angular cmd.vel.

	// bool movingViaMidfieldCamera;				// True => Moving for a period because of the midfield camera.

	bool needToTurn180;							// True => Need to turn 180 degrees.

	// struct timeval periodStart;					// For measuring a period.

	bool precachedSampleFetched;				// True => Have picked up (not necessarily delivered) the precached sample.

	bool precachedSampleFoundNearField;			// True => Have seen the precached sample in the near field camera.

	bool precachedSampleIsVisibleNearField;		// True => Near field camera sees precached sample.

	bool precachedSampleIsVisibleMidField;		// True => Mid field camera sees precached sample.

	double startYaw;							// To compute how much of a turn has been made.

	bool waitingPauseOff;	// Waiting for pause to be released.
	bool waitingPauseOn;	// Waiting for pause to be set.a
};

#endif
