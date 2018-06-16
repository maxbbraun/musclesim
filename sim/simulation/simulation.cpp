#include "simulation.h"

void Simulation::initialize() {
  setStepSize(DEFAULT_STEP);

  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world, 0, -GRAVITY, 0);
  dWorldSetERP(world, ERP);
  dWorldSetCFM(world, CFM);

  Simulation::plane = (HAS_PLANE ? dCreatePlane(space, 0, 1, 0, 0) : 0);

  steps = 0;
}

void Simulation::quit() {
  if (contactgroup) dJointGroupDestroy(contactgroup);
  if (space) dSpaceDestroy(space);
  if (world) dWorldDestroy(world);
  dCloseODE();
}

void Simulation::doStep() {
  dSpaceCollide(space, 0, &nearCallback);
  dWorldStep(world, step_size);
  dJointGroupEmpty(contactgroup);

  steps++;
}

dWorldID Simulation::getWorld() { return world; }

dSpaceID Simulation::getSpace() { return space; }

void Simulation::setStepSize(double step_size) {
  Simulation::step_size = step_size;
}

void Simulation::setGravity(double x, double y, double z) {
  dWorldSetGravity(world, x, y, z);
}

bool Simulation::hasPlane() { return plane != 0; }

int Simulation::getSteps() { return steps; }

double Simulation::getTime() { return steps * step_size; }

dWorldID Simulation::world;
dSpaceID Simulation::space;

dGeomID Simulation::plane;

double Simulation::step_size;

dJointGroupID Simulation::contactgroup;

void Simulation::nearCallback(void* data, dGeomID o1, dGeomID o2) {
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  dContact contact;

  if (b1 && b2 && dAreConnected(b1, b2)) return;

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
    dSpaceCollide2(o1, o2, data, &nearCallback);
    // if (dGeomIsSpace(o1)) dSpaceCollide((dSpaceID)o1, data, &nearCallback);
    // if (dGeomIsSpace(o2)) dSpaceCollide((dSpaceID)o2, data, &nearCallback);

  } else {
    contact.surface.mode = 0;
    contact.surface.mu = 0.1;

    if (dCollide(o1, o2, MAX_CONTACTS, &contact.geom, sizeof(dContactGeom))) {
      dJointID c = dJointCreateContact(world, contactgroup, &contact);
      dJointAttach(c, b1, b2);
    }
  }
}

int Simulation::steps;
