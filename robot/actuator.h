#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "primitive.h"	// comment out if not present, connect*To() will accept dBodyID instead of Primitive*

#include <ode/ode.h>
#include <GLUT/glut.h>
#include <iostream>
#include <vector>
using namespace std;

#define ACTUATOR_DETAIL 30    // for cylinders and shperes

#define LINK_RADIUS 7.5    // mm
#define LINK_MASS 10    // gram
#define LINK_DENSITY 1    // gram/mm^3
#define HINGE_F_MAX 8000    // friction force for hinge joints
#define SLIDER_F_MAX 1000    // friction force for slider joints
#define HINGE_STOP (60*M_PI/180)    // maximum angle for hinge joints
#define HINGE_BOUNCE 0.2    // bouncyness of maximum angle for hinge joints
#define FORCE_CORRECTION 1000    // due to mm scale
#define MAX_AIR_PRESSURE 1.0    // minimum = 0
#define GRAVITY 9.81

#define FACTOR_A 0.00007687724845623249    // coefficients for function fit of actuator behavior
#define FACTOR_B 9.203893778016578
#define FACTOR_C 9.799574209874333

class Actuator {

public:

	Actuator(
		dWorldID world,
		dSpaceID outer_space,
		dVector3 start_at,
		dVector3 start_control,
		dVector3 end_control,
		dVector3 end_at,
		double minimum_length,
		double maximum_length,
		bool linear    // linear = 1 link
	);
	
	virtual ~Actuator();
	static void deleteAll();    // destroys all actuators
		
	static void simulateAll();    // updates forces for all actuators
	
	void setAirPressure(double new_pressure);
	double getAirPressure();
	static bool setAirPressure(int index, double new_pressure);    // set air pressure for actuator #index (starts with 0)
	static double getAirPressure(int index);    // get air pressure for actuator #index (starts with 0)
	static void resetAll();    // reset air pressure for all actuators
	
	void setColor(GLfloat red, GLfloat green, GLfloat blue);
	
	int getIndex();    // # of actuator (starts with 0)
	static int getNumberOfActuators();
	
	double getLength();    // in mm
	
	// one of the actuators is selected and can be treated differently (used in interactive mode):
	
	static bool selectActuator(int index);
	static Actuator* getSelectedActuator();

	// connect*To() will accept dBodyID instead of Primitive* (if class is not present):
	
	#ifdef PRIMITIVE_H
	
	void connectStartTo(Primitive* primitive);
	void connectEndTo(Primitive* primitive);
	
	#else
	
	void connectStartTo(dBodyID body);
	void connectEndTo(dBodyID body);
	
	#endif
	
	static void drawAll(GLfloat alpha);
	
	static void maskSelected();    // used to highlight the selected actuator
	
	static void setVerbose(bool verbose);    // whether to print information on startup
	
private:
	
	static vector<Actuator*> all_actuators;
	static Actuator* selected_actuator;
	
	dWorldID world;
	dSpaceID outer_space;
	dSpaceID space;
	dBodyID* bodies;
	dGeomID* geoms;
	dJointID* slider_joints;
	dJointID* hinge_joints;
	
	GLfloat black_red;
	GLfloat black_green;
	GLfloat black_blue;
	GLfloat metal_red;
	GLfloat metal_green;
	GLfloat metal_blue;
	
	virtual void drawChain(GLfloat alpha);
	virtual void drawCaps(GLfloat alpha);
	
	dVector3 start_at;
	dVector3 end_at;
	double expansion;
	int number_of_links;
	
	int actuator_index;
	
	inline static void getMatrix(dBodyID body, double* m);
	inline static void deCasteljau(dVector3 p0, dVector3 p1, dVector3 p2, dVector3 p3, double t, dVector3* p);
	
	inline void drawCylinder(int i, double adjust_beginning, double adjust_end);
	inline void drawCap();
	
	void simulate();
	virtual double calculateForce() = 0;
	
	static bool verbose;

protected:
	
	double maximum_length;
	double minimum_length;
		
	double air_pressure;
	
};

class Muscle : public Actuator {

public:
	
	Muscle(
		dWorldID world,
		dSpaceID outer_space,
		dVector3 start_at,
		dVector3 start_control,   // control points for Bezier curve
		dVector3 end_control,
		dVector3 end_at,
		double minimum_length,   // make sure that the distance from start to end is
		double maximum_length    // between minimum_length and maximum_length
	);
		
private:
	
	double calculateForce();

};

class LinearActuator : public Actuator {

public:
	
	LinearActuator(
		dWorldID world,
		dSpaceID outer_space,
		dVector3 start_at,
		dVector3 end_at,
		double minimum_length,   // make sure that the distance from start to end is
		double maximum_length    // between minimum_length and maximum_length
	);
		
private:
	
	double calculateForce();

};

#endif
