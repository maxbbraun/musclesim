#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <ode/ode.h>
#include <GLUT/glut.h>
#include <iostream>
#include <vector>
using namespace std;

#define PRIMITIVE_DETAIL 30    // for cylinders and shperes

class Primitive {

public:
	
	Primitive(dWorldID world, dSpaceID space, dVector3 position);
	
	virtual ~Primitive() {};
	static void deleteAll();    // destroys all primitives
	
	dBodyID getBody();    // returns the parent's body if part of a composite
	dGeomID getGeom();

	void rotate(dReal angle, dVector3 axis);    // angle in degrees
	void fix();
	
	void setColor(GLfloat red, GLfloat green, GLfloat blue);
	
	void attachBall(Primitive* primitive, dVector3 anchor);
	void attachHinge(Primitive* primitive, dVector3 anchor, dVector3 axis);
	void attachHinge(
		Primitive* primitive,
		dVector3 anchor,
		dVector3 axis,
		double low_stop,    // angles in degrees
		double high_stop
	);
	void attachUniversal(Primitive* primitive, dVector3 anchor, dVector3 axis_1, dVector3 axis_2);
	void attachUniversal(
		Primitive* primitive,
		dVector3 anchor,
		dVector3 axis_1,
		dVector3 axis_2,
		double low_stop_1,    // angles in degrees
		double high_stop_1,
		double low_stop_2,
		double high_stop_2
	);
	
	// be sure to call composite[With]Primitive[s]() before attach*() or connect*To()
	
	void compositeWithPrimitive(Primitive* child);    // combines two primitives to one composite (call only once)
	static void compositePrimitives(vector<Primitive*> primitives);	    // combines multiple primitives to one composite
	
	static void drawAll(GLfloat alpha);
	
	void getPosition(double& x, double& y, double& z);
	
	static void setVerbose(bool verbose);    // whether to print information on startup

private:
	
	static vector<Primitive*> all_primitives;
	
	inline void getMatrix(dGeomID geom, double* m);
	
	GLfloat red;
	GLfloat green;
	GLfloat blue;
	
	Primitive* parent;
	bool hasParent();
	Primitive* getParent();
	void setParent(Primitive* parent);
	void setGeom(dGeomID geom);
	void removeBody(Primitive* parent);
	void compositeWithPrimitives(vector<Primitive*> children);

protected:
	
	virtual void draw(GLfloat alpha);
	
	dWorldID world;
	dSpaceID space;
	dBodyID body;
	dGeomID geom;
	
	static bool verbose;

};

class CappedCylinder : public Primitive {
	
public:
	
	CappedCylinder(
		dWorldID world,
		dSpaceID space,
		dVector3 position,
		dReal mass,
		dReal density,
		dReal radius,
		dReal length
	);
	
	void draw(GLfloat alpha);
	
private:
	
	dReal length;
	dReal radius;
	
};

class Sphere : public Primitive {
	
public:
		
	Sphere(
		dWorldID world,
		dSpaceID space,
		dVector3 position,
		dReal mass,
		dReal density,
		dReal radius
	);
	
	void draw(GLfloat alpha);

private:
	
	dReal radius;
	
};

class Box : public Primitive {

public:
		
	Box(
		dWorldID world,
		dSpaceID space,
		dVector3 position,
		dReal mass,
		dReal density,
		dReal size_x,
		dReal size_y,
		dReal size_z
	);
	
	void draw(GLfloat alpha);

private:
	
	dReal size_x;
	dReal size_y;
	dReal size_z;
		
};

#endif