#ifndef SIMULATION_H
#define SIMULATION_H

#include <ode/ode.h>
#include <iostream>
#include <vector>
using namespace std;

#define GRAVITY 9.81

#define ERP 0.2
#define CFM (1e-10)
#define MAX_CONTACTS 1    // for collision detection
#define DEFAULT_STEP 0.01
#define HAS_PLANE false

class Simulation {

public:

	static void initialize();    // first
	static void quit();    // last
	static void doStep();    // in loop
	
	static dWorldID getWorld();
	static dSpaceID getSpace();
	
	static void setStepSize(double step_size);
	static void setGravity(double x, double y, double z);    // change gravity vector
	static bool hasPlane();
	
	static int getSteps();    // total step count
	static double getTime();    // total elapsed time

private:
	
	static dWorldID world;
	static dSpaceID space;
	
	static dGeomID plane;
	
	static double step_size;
	
	static dJointGroupID contactgroup;
	static void nearCallback(void* data, dGeomID o1, dGeomID o2);
	
	static int steps;
	
};

#endif
