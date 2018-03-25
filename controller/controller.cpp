#include "controller.h"

Controller::Controller() {
	
	action = new double[NUMBER_OF_ACTUATORS];
	
	Target::set(-300, 0, 150);
		
	log_file.open(LOG_FILE_PATH);
	
}

Controller::~Controller() {

	log_file.close();
	
	delete[] action;
	
}

Controller* Controller::active_controller;

Controller* Controller::getActiveController() {
	
	// defines the type of robot to be used:
	
	active_controller = new ASMController();
	// active_controller = new AASMController();
	
	return active_controller;
	
}

void Controller::deactivate() {
	
	delete active_controller;
	
}

ASMController::ASMController() : Controller() {
	
	last_a_1 = 0;
	last_height = -241.167;
	
	yuragi = new gmpdf(NUMBER_OF_ACTUATORS, 400);
	
	log_file << "#\ttime [s]\theight of hand [mm]\theight of target [mm]\tactivity";
	for (int i=0; i<NUMBER_OF_ACTUATORS; i++) log_file << "\tx_" << i;
	log_file << endl;
	
}

ASMController::~ASMController() {
	
	delete yuragi;
	
}

double* ASMController::output(double* state) {
	
	double height = state[1];

	if (fmod( Simulation::getTime(), ACTUATOR_LAG ) == 0) {    // wait for actuators
	
		// calculate activity from state and target position:

		double A = height - last_height;
		double a_1 = 0.7 * last_a_1 + 5 * A * (Target::getY() > height ? 1 : -1);
		double a_2 = a_1 > -0.5 ? a_1 + 1.0 : 0.5;
		a_2 = a_2 + 10 * exp( -(Target::getY() - height) * (Target::getY() - height) / 100 );
		activity = a_2 * a_2;
	
		// get next yuragi state:
	
		double* x = yuragi->rejectSampling(activity, NULL);

		// write action (map [-1,1] to [0,1])
	
		for (int i=0; i<NUMBER_OF_ACTUATORS; i++) {
			action[i] = 1.0 * ( tanh( 3 * x[i] ) / 2.0 + 0.5 );
		}
		
		last_a_1 = a_1;
	
	}
	
	// write log:

	log_file << "\t" << Simulation::getTime() << "\t" << height << "\t" << Target::getY() << "\t" << activity;
	for (int i=0; i<NUMBER_OF_ACTUATORS; i++) log_file << "\t" << action[i];
	log_file << endl;
	
	last_height = height;
		
	return action;
	
}

AASMController::AASMController() : Controller() {
	
	goal = new double[3];
	goal[0] = Target::getX();
	goal[1] = Target::getY();
	goal[2] = Target::getZ();
	
	yuragi = new AdaptiveAttractorSelection(NUMBER_OF_ACTUATORS, 500);
	// yuragi->resetASMState(-0.3);
	yuragi->setGoal(3, goal);
	
	log_file
	<< "#\ttime [s]\tx [mm]\ty [mm]\tz [mm]\tdistance to target ("
	<< Target::getX() << ", "
	<< Target::getY() << ", "
	<< Target::getZ()
	<< ") [mm]\tactivity";
	for (int i=0; i<NUMBER_OF_ACTUATORS; i++) log_file << "\tx_" << i;
	log_file << endl;
	
}

AASMController::~AASMController() {
	
	delete goal;
	delete yuragi;
	
}

double* AASMController::output(double* state) {

	if (fmod( Simulation::getTime(), ACTUATOR_LAG ) == 0) {    // wait for actuators
	
		// calculate activity from state and target position:

		activity = yuragi->calculateActivity(state);
	
		// get next yuragi state:
	
		double* x = yuragi->nextASMState(activity);

		// write action (map [-1,1] to [0,1])
	
		for (int i=0; i<NUMBER_OF_ACTUATORS; i++) {
			action[i] = 1.0 * ( tanh( 3 * x[i] ) / 2.0 + 0.5 );
		}
		
		yuragi->saveData(Simulation::getTime());
		
	}
	
	// write log:
	
	double distance = sqrt(
		( state[0] - Target::getX() ) * ( state[0] - Target::getX() ) +
		( state[1] - Target::getY() ) * ( state[1] - Target::getY() ) +
		( state[2] - Target::getZ() ) * ( state[2] - Target::getZ() )
	); 

	log_file << "\t"
	<< Simulation::getTime() << "\t"
	<< state[0] << "\t"
	<< state[1] << "\t"
	<< state[2] << "\t"
	<< distance << "\t"
	<< activity;
	for (int i=0; i<NUMBER_OF_ACTUATORS; i++) log_file << "\t" << action[i];
	log_file << endl;
		
	return action;
	
}

double Target::x;
double Target::y;
double Target::z;

void Target::set(double x, double y, double z) {
	
	Target::x = x;
	Target::y = y;
	Target::z = z;
	
}

double Target::getX() {
	
	return Target::x;
	
}

double Target::getY() {
	
	return Target::y;
	
}

double Target::getZ() {
	
	return Target::z;
	
}

void Target::draw(GLfloat alpha) {
	
	GLfloat diffuse[] = {1.0, 0.0, 0.0, 0.5 * alpha};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
	
	// GLfloat specular[] = {0.0, 0.0, 0.0, 0.5 * alpha};
	// glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
	
	glPushMatrix();
	
		glTranslated(Target::x, Target::y, Target::z);
		glutSolidSphere(TARGET_RADIUS, TARGET_DIVISIONS, TARGET_DIVISIONS);
		
	glPopMatrix();
	
}
