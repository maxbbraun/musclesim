#include <iostream>
#include <stdio.h>
using namespace std;

#include "controller/controller.h"
#include "robot/robot.h"
#include "simulation/simulation.h"
#include "simulation/display.h"

void cleanUp() {
	
	Actuator::deleteAll();
	Primitive::deleteAll();
	Simulation::quit();
	Robot::deactivate();
	Controller::deactivate();
	
}

int main(int argc, char** argv) {

	// exit normally on Ctrl-C:
	
	signal(SIGINT, exit);
	
	// clean up on exit:
	
	atexit(cleanUp);
	
	// start:
	
	cout << endl << "Artificial Muscle Simulator, Max Braun, Osaka University, Yuragi Project, 2008" << endl;
	
	// parse command line arguments:

	double step_size = DEFAULT_STEP;
	bool graphics_flag = true;
	bool verbose_flag = false;
	
	for (int i=1; i<argc; i++) {
		
		if (strcmp(argv[i], "--simple") == 0) {
			graphics_flag = false;
		} else if (strcmp(argv[i], "--verbose") == 0) {
			verbose_flag = true;
		} else if (strcmp(argv[i], "--step") == 0 && i+1 < argc) {
			step_size = atof(argv[i+1]);
			i++;
		} else {
			cout << "Unknown command line argument: " << argv[i] << endl;
			return 0;
		}
		
	}
	
	// print help:
	
	cout << endl
	<< "Command line arguments:" << endl
	<< " --simple" << "\t\t" << "Faster simulation without display or interaction" << endl
	<< " --step x" << "\t\t" << "Set ODE simulation step size to x (default: " << DEFAULT_STEP << ")" << endl
	<< " --verbose" << "\t\t" << "Print more information on startup" << endl;
	
	if (verbose_flag) {
		Actuator::setVerbose(true);
		Primitive::setVerbose(true);
		cout << endl;
	} else {
		Actuator::setVerbose(false);
		Primitive::setVerbose(false);
	}
	
	// initialize:
	
	Simulation::initialize();
	Simulation::setStepSize(step_size);
	Robot* robot = Robot::getActiveRobot();
	Controller* controller = Controller::getActiveController();

	cout << endl;

	if (graphics_flag) {
		
		// simulation in display run loop:
		
		Display::initialize(argc, argv);
		Display::mainLoop(robot, controller);
		
	} else {
	
		cout << "Use Ctrl-C to quit." << endl << endl;
		
		// simulation loop:
	
		while(true) {
		
			// control:
		
			double* state = robot->getState();
			double* action = controller->output(state);
			robot->step(action);
		
			// simulation step:

			Actuator::simulateAll();
			Simulation::doStep();
		
		}
		
	}
	
	cleanUp();
	
	return 0;
	
}