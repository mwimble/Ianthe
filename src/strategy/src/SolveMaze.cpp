#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "strategy/SolveMaze.h"

SolveMaze::SolveMaze() {

}

SolveMaze& SolveMaze::Singleton() {
	static SolveMaze singleton_;
	return singleton_;
}

string SolveMaze::name() {
	return string("SolveMaze");
}

StrategyFn::RESULT_T SolveMaze::tick() {
	RESULT_T result = SUCCESS;
	return result;
}

