#ifndef __SOLVE_MAZE
#define __SOLVE_MAZE

#include "strategy/StrategyFn.h"

class SolveMaze : public StrategyFn {
private:
	// Singleton pattern.
	SolveMaze();
	SolveMaze(SolveMaze const&) {};
	SolveMaze& operator=(SolveMaze const&) {}

public:
	RESULT_T tick();

	string name();

	static SolveMaze& Singleton();
	
};

#endif