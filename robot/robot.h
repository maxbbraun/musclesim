#ifndef ROBOT_H
#define ROBOT_H

#include "actuator.h"
#include "primitive.h"
#include "../simulation/simulation.h"

#define ENVIRONMENT NULL

class Robot {
	
public:
	
	Robot();
	virtual ~Robot();
	
	virtual double* getState() = 0;
	virtual void step(double* action) = 0;
	
	static Robot* getActiveRobot();    // returns an instance of the robot type you want to use
	static void deactivate();
	
private:
	
	static Robot* active_robot;
	
protected:
	
	dWorldID world;
	dSpaceID space;
	
	double* state;
	
};

class RoboticArm : public Robot {
	
public:
	
	RoboticArm();
	
	double* getState();
	void step(double* action);

private:
	
	Primitive* hand;
	
};

#endif
