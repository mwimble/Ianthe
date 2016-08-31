#include <ros/ros.h>
#include <ros/console.h>

#include "strategy/GotoCrossing.h"
#include "strategy/SolveMaze.h"
#include "strategy/StrategyContext.h"
#include "strategy/StrategyException.h"

int debug_last_message = 0; // So that we only emit messages when things change.
		
void logIfChanged(int id, const char* message) {
	if (id != debug_last_message) {
		ROS_INFO_STREAM(message);
		debug_last_message = id;
	}	
}

vector<StrategyFn*> behaviors;

int main(int argc, char** argv) {
	ros::init(argc, argv, "strategy_node");
	SolveMaze& solveMaze = SolveMaze::Singleton();
	StrategyContext& strategyContext = StrategyContext::Singleton();

	// ROS node handle.
	ros::NodeHandle nh;

	ros::Rate rate(40); // Loop rate

	behaviors.push_back(&GotoCrossing::Singleton());
	//#####behaviors.push_back(&IsHealthy::Singleton());
	// behaviors.push_back(&FetchPrecachedSample::Singleton());
	// behaviors.push_back(&GoHome::Singleton());

	strategyContext.atGoal = false;

	while (ros::ok()) {
		try { // Emplement Sequence behavior
			rate.sleep();
			ros::spinOnce();

			//ROS_INFO_STREAM("--- ---- ---- ---- Begin of strategy loop ---- ---- ---- ----");
			for(vector<StrategyFn*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
				StrategyFn::RESULT_T result = ((*it)->tick)();
				//ROS_INFO_STREAM("[_strategy_node] called tick for '" << ((*it)->name()) << " with result: " << result);
				if (result == StrategyFn::RESTART_LOOP) {
					//ROS_INFO_STREAM("[_strategy_node] function: " << ((*it)->name()) << ", RESTART_LOOP result, restarting");
					break; // throw new StrategyException("RESTART_LOOP");
				}

				if (result == StrategyFn::FATAL) {
					ROS_INFO_STREAM("[_strategy_node] function: " << ((*it)->name()) << ", FATAL result, exiting");
					return -1;
				}

				if (result == StrategyFn::RUNNING) {
					//ROS_INFO_STREAM("[_strategy_node] function " << ((*it)->name()) << ", RUNNING, restarting");
					break; // throw new StrategyException("RESTART_LOOP");
				}

				if (result == StrategyFn::SUCCESS) {
					//ROS_INFO_STREAM("[_strategy_node] function: " << ((*it)->name()) << ", SUCCESS result, continuing");
					continue;
				}

				if (result == StrategyFn::FAILED) {
					ROS_INFO_STREAM("[_strategy_node] function: " << ((*it)->name()) << ", FAILED result, aborting");
					break;
				}
			}
		} catch(StrategyException* e) {
			//ROS_INFO_STREAM("[_strategy_node] StrategyException: " << e->what());
			// Do nothing.
		}
	}

	return 0;
}

