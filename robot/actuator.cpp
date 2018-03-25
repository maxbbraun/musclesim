#include "actuator.h"
#include <cassert>

Actuator::Actuator(dWorldID world, dSpaceID outer_space, dVector3 start_at,
                   dVector3 start_control, dVector3 end_control,
                   dVector3 end_at, double minimum_length,
                   double maximum_length, bool linear) {
  actuator_index = all_actuators.size();
  all_actuators.push_back(this);
  selectActuator(actuator_index);

  // set parameters:

  this->world = world;
  this->outer_space = outer_space;
  this->start_at[0] = start_at[0];
  this->start_at[1] = start_at[1];
  this->start_at[2] = start_at[2];
  this->end_at[0] = end_at[0];
  this->end_at[1] = end_at[1];
  this->end_at[2] = end_at[2];
  this->minimum_length =
      minimum_length;  // will be overwritten for non-linear actuators
  this->maximum_length = maximum_length;

  air_pressure = 0;

  black_red = 0.2;
  black_green = 0.2;
  black_blue = 0.2;
  metal_red = 0.8;
  metal_green = 0.8;
  metal_blue = 0.8;

  space = dHashSpaceCreate(outer_space);

  // calculate number of links:

  if (linear) {
    number_of_links = 1;

  } else {
    number_of_links = floor(minimum_length / (2 * LINK_RADIUS) + 0.5);
    assert(number_of_links >= 2);

    this->minimum_length = number_of_links * 2 * LINK_RADIUS;
  }

  expansion = maximum_length / number_of_links;

  // allocate memory:

  bodies = new dBodyID[2 * number_of_links];
  geoms = new dGeomID[2 * number_of_links];
  slider_joints = new dJointID[number_of_links];
  hinge_joints = new dJointID[number_of_links - 1];

  // calculate distance:

  double distance = 0;
  for (int i = 0; i < number_of_links; i++) {
    dVector3 current, next;
    deCasteljau(start_at, start_control, end_control, end_at,
                (double)i / number_of_links, &current);
    deCasteljau(start_at, start_control, end_control, end_at,
                (double)(i + 1) / number_of_links, &next);

    const double difference[3] = {next[0] - current[0], next[1] - current[1],
                                  next[2] - current[2]};

    distance +=
        sqrt(difference[0] * difference[0] + difference[1] * difference[1] +
             difference[2] * difference[2]);
  }

  assert(distance <= maximum_length);
  assert(distance >= minimum_length);

  // bodies and mass:

  dMass mass;
  dMassSetSphere(&mass, LINK_DENSITY, LINK_RADIUS);
  dMassAdjust(&mass, LINK_MASS);

  for (int i = 0; i < 2 * number_of_links; i++) {
    bodies[i] = dBodyCreate(world);
    dBodySetMass(bodies[i], &mass);
  }

  for (int i = 0; i < number_of_links; i++) {
    // get direction:

    dVector3 current, next;
    deCasteljau(start_at, start_control, end_control, end_at,
                (double)i / number_of_links, &current);
    deCasteljau(start_at, start_control, end_control, end_at,
                (double)(i + 1) / number_of_links, &next);

    const double difference[3] = {next[0] - current[0], next[1] - current[1],
                                  next[2] - current[2]};

    const double distance =
        sqrt(difference[0] * difference[0] + difference[1] * difference[1] +
             difference[2] * difference[2]);

    const double direction[3] = {difference[0] / distance,
                                 difference[1] / distance,
                                 difference[2] / distance};

    // orientation:

    dQuaternion q1, q2, q;
    dQFromAxisAndAngle(q1, 0, 1, 0, acos(direction[2]));
    dQFromAxisAndAngle(q2, 0, 0, 1, atan2(direction[1], direction[0]));
    dQMultiply0(q, q2, q1);
    dBodySetQuaternion(bodies[2 * i], q);
    dBodySetQuaternion(bodies[2 * i + 1], q);

    // force and torque:

    dBodySetLinearVel(bodies[2 * i], 0, 0, 0);
    dBodySetLinearVel(bodies[2 * i + 1], 0, 0, 0);
    dBodySetAngularVel(bodies[2 * i], 0, 0, 0);
    dBodySetAngularVel(bodies[2 * i + 1], 0, 0, 0);

    // position:

    dBodySetPosition(bodies[2 * i], current[0] + direction[0] * LINK_RADIUS,
                     current[1] + direction[1] * LINK_RADIUS,
                     current[2] + direction[2] * LINK_RADIUS);
    dBodySetPosition(bodies[2 * i + 1], next[0] - direction[0] * LINK_RADIUS,
                     next[1] - direction[1] * LINK_RADIUS,
                     next[2] - direction[2] * LINK_RADIUS);

    // geom:

    geoms[2 * i] = dCreateSphere(space, LINK_RADIUS);
    dGeomSetBody(geoms[2 * i], bodies[2 * i]);
    geoms[2 * i + 1] = dCreateSphere(space, LINK_RADIUS);
    dGeomSetBody(geoms[2 * i + 1], bodies[2 * i + 1]);

    // set slider joints:

    slider_joints[i] = dJointCreateSlider(world, 0);
    dJointAttach(slider_joints[i], bodies[2 * i], bodies[2 * i + 1]);
    dJointSetSliderAxis(slider_joints[i], direction[0], direction[1],
                        direction[2]);
    dJointSetSliderParam(slider_joints[i], dParamLoStop, distance - expansion);
    dJointSetSliderParam(slider_joints[i], dParamHiStop,
                         distance - 2 * LINK_RADIUS);
    dJointSetSliderParam(slider_joints[i], dParamVel, 0);
    dJointSetSliderParam(slider_joints[i], dParamFMax, SLIDER_F_MAX);
    dJointSetSliderParam(slider_joints[i], dParamBounce, 0);
  }

  // set hinge joints:

  for (int i = 0; i < number_of_links - 1; i++) {
    // get direction:

    dVector3 current, next;
    deCasteljau(start_at, start_control, end_control, end_at,
                (double)i / number_of_links, &current);
    deCasteljau(start_at, start_control, end_control, end_at,
                (double)(i + 1) / number_of_links, &next);

    const double difference[3] = {next[0] - current[0], next[1] - current[1],
                                  next[2] - current[2]};

    const double distance =
        sqrt(difference[0] * difference[0] + difference[1] * difference[1] +
             difference[2] * difference[2]);

    const double direction[3] = {difference[0] / distance,
                                 difference[1] / distance,
                                 difference[2] / distance};

    hinge_joints[i] = dJointCreateHinge(world, 0);
    dJointAttach(hinge_joints[i], bodies[2 * i + 1], bodies[2 * i + 2]);

    dJointSetHingeAnchor(hinge_joints[i], next[0], next[1], next[2]);

    // cross product between up vector and direction:

    double hinge_axis[3];
    if (direction[0] || direction[2]) {
      hinge_axis[0] = direction[2];
      hinge_axis[1] = 0;
      hinge_axis[2] = -direction[0];
    } else {
      hinge_axis[0] = 1;
      hinge_axis[1] = 0;
      hinge_axis[2] = 0;
    }
    dJointSetHingeAxis(hinge_joints[i], hinge_axis[0], hinge_axis[1],
                       hinge_axis[2]);

    dJointSetHingeParam(hinge_joints[i], dParamVel, 0);
    dJointSetHingeParam(hinge_joints[i], dParamFMax, HINGE_F_MAX);
    dJointSetHingeParam(hinge_joints[i], dParamLoStop, -HINGE_STOP);
    dJointSetHingeParam(hinge_joints[i], dParamHiStop, HINGE_STOP);
    dJointSetHingeParam(hinge_joints[i], dParamBounce, HINGE_BOUNCE);
  }

  // feedback:

  if (Actuator::verbose) {
    cout << (linear ? "Linear actuator" : "Actuator") << " created: "
         << "min. = " << this->minimum_length << " (" << minimum_length
         << " intended), " << number_of_links << " link"
         << (number_of_links != 1 ? "s" : "") << ", "
         << "max. = " << this->maximum_length << ", "
         << "mass = " << 2 * number_of_links * LINK_MASS << endl;
  }
}

Actuator::~Actuator() {
  dSpaceDestroy(space);
  delete[] bodies;
  delete[] geoms;
  delete[] slider_joints;
  delete[] hinge_joints;
}

void Actuator::deleteAll() {
  for (vector<Actuator*>::iterator ii = all_actuators.begin();
       ii != all_actuators.end(); ++ii) {
    delete *ii;
  }
}

void Actuator::simulateAll() {
  for (vector<Actuator*>::iterator ii = all_actuators.begin();
       ii != all_actuators.end(); ++ii) {
    (*ii)->simulate();
  }
}

void Actuator::setAirPressure(double new_pressure) {
  air_pressure =
      new_pressure > 0
          ? (new_pressure <= MAX_AIR_PRESSURE ? new_pressure : MAX_AIR_PRESSURE)
          : 0;
}

double Actuator::getAirPressure() { return air_pressure; }

bool Actuator::setAirPressure(int index, double new_pressure) {
  Actuator* actuator;

  try {
    actuator = all_actuators.at(index);
  } catch (...) {
    return false;
  }

  actuator->setAirPressure(new_pressure);

  return true;
}

double Actuator::getAirPressure(int index) {
  Actuator* actuator;

  try {
    actuator = all_actuators.at(index);
  } catch (...) {
    return -1;
  }

  return actuator->getAirPressure();
}

void Actuator::resetAll() {
  for (vector<Actuator*>::iterator ii = all_actuators.begin();
       ii != all_actuators.end(); ++ii) {
    (*ii)->setAirPressure(0);
  }
}

void Actuator::setColor(GLfloat red, GLfloat green, GLfloat blue) {
  this->metal_red = red;
  this->metal_green = green;
  this->metal_blue = blue;
}

int Actuator::getIndex() { return actuator_index; }

int Actuator::getNumberOfActuators() { return all_actuators.size(); }

double Actuator::getLength() {
  double sum = 0;
  for (int i = 0; i < number_of_links; i++) {
    const dReal* current_position = dBodyGetPosition(bodies[2 * i]);
    const dReal* next_position = dBodyGetPosition(bodies[2 * i + 1]);
    const double distance = sqrt((next_position[0] - current_position[0]) *
                                     (next_position[0] - current_position[0]) +
                                 (next_position[1] - current_position[1]) *
                                     (next_position[1] - current_position[1]) +
                                 (next_position[2] - current_position[2]) *
                                     (next_position[2] - current_position[2]));
    sum += distance;
  }

  return sum + number_of_links * 2 * LINK_RADIUS;
}

bool Actuator::selectActuator(int index) {
  try {
    selected_actuator = all_actuators.at(index);
  } catch (...) {
    return NULL;
  }
  return selected_actuator;
}

Actuator* Actuator::getSelectedActuator() { return selected_actuator; }

#ifdef PRIMITIVE_H

void Actuator::connectStartTo(Primitive* primitive) {
  dJointID joint = dJointCreateBall(world, 0);
  dJointAttach(joint, bodies[0], primitive ? primitive->getBody() : 0);
  dJointSetBallAnchor(joint, start_at[0], start_at[1], start_at[2]);
}

void Actuator::connectEndTo(Primitive* primitive) {
  dJointID joint = dJointCreateBall(world, 0);
  dJointAttach(joint, bodies[2 * number_of_links - 1],
               primitive ? primitive->getBody() : 0);
  dJointSetBallAnchor(joint, end_at[0], end_at[1], end_at[2]);
}

#else

void Actuator::connectStartTo(dBodyID body) {
  dJointID joint = dJointCreateBall(world, 0);
  dJointAttach(joint, bodies[0], body);
  dJointSetBallAnchor(joint, start_at[0], start_at[1], start_at[2]);
}

void Actuator::connectEndTo(dBodyID body) {
  dJointID joint = dJointCreateBall(world, 0);
  dJointAttach(joint, bodies[2 * number_of_links - 1], body);
  dJointSetBallAnchor(joint, end_at[0], end_at[1], end_at[2]);
}

#endif

void Actuator::drawAll(GLfloat alpha) {
  for (vector<Actuator*>::iterator ii = all_actuators.begin();
       ii != all_actuators.end(); ++ii) {
    // GLfloat black_specular[] = {0.0, 0.0, 0.0, alpha};
    // glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black_specular);

    (*ii)->drawChain(alpha);

    // GLfloat metal_specular[] = {1.0, 1.0, 1.0, alpha};
    // glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, metal_specular);

    (*ii)->drawCaps(alpha);
  }
}

void Actuator::maskSelected() {
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glColor4f(1.0, 1.0, 1.0, 1.0);
  selected_actuator->drawChain(1.0);
  selected_actuator->drawCaps(1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

void Actuator::setVerbose(bool verbose) { Actuator::verbose = verbose; }

void Actuator::drawChain(GLfloat alpha) {
  GLfloat black_diffuse[] = {black_red, black_green, black_blue, alpha};
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black_diffuse);

  /*double m[16];
  for (int i=0; i<number_of_links; i++) {
          glPushMatrix();
                  getMatrix(bodies[2*i+1], m);
                  glMultMatrixd(m);
                  glutSolidSphere(LINK_RADIUS, ACTUATOR_DETAIL,
  ACTUATOR_DETAIL);
                  // glutSolidCube(2*LINK_RADIUS);
          glPopMatrix();
          glPushMatrix();
                  getMatrix(bodies[2*i], m);
                  glMultMatrixd(m);
                  glutSolidSphere(LINK_RADIUS, ACTUATOR_DETAIL,
  ACTUATOR_DETAIL);
                  // glutSolidCube(2*LINK_RADIUS);
          glPopMatrix();
  }*/

  if (number_of_links == 1) {
    drawCylinder(0, LINK_RADIUS, LINK_RADIUS);
  } else {
    drawCylinder(0, LINK_RADIUS, 0);
    glPushMatrix();
    double m[16];
    getMatrix(bodies[1], m);
    glMultMatrixd(m);
    glTranslated(0, 0, LINK_RADIUS);
    glutSolidSphere(LINK_RADIUS, ACTUATOR_DETAIL, ACTUATOR_DETAIL);
    glPopMatrix();
    for (int i = 1; i < number_of_links - 1; i++) {
      drawCylinder(i, 0, 0);
      glPushMatrix();
      double m[16];
      getMatrix(bodies[2 * i + 1], m);
      glMultMatrixd(m);
      glTranslated(0, 0, LINK_RADIUS);
      glutSolidSphere(LINK_RADIUS, ACTUATOR_DETAIL, ACTUATOR_DETAIL);
      glPopMatrix();
    }
    drawCylinder(number_of_links - 1, 0, LINK_RADIUS);
  }
}

void Actuator::drawCaps(GLfloat alpha) {
  GLfloat metal_diffuse[] = {metal_red, metal_green, metal_blue, alpha};
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, metal_diffuse);

  double m[16];

  glPushMatrix();
  getMatrix(bodies[0], m);
  glMultMatrixd(m);
  drawCap();
  glPopMatrix();

  glPushMatrix();
  getMatrix(bodies[2 * number_of_links - 1], m);
  glMultMatrixd(m);
  glRotated(180, 1, 0, 0);
  drawCap();
  glPopMatrix();
}

vector<Actuator*> Actuator::all_actuators;
Actuator* Actuator::selected_actuator;
bool Actuator::verbose;

inline void Actuator::getMatrix(dBodyID body, double* m) {
  const dReal* p = dBodyGetPosition(body);
  const dReal* r = dBodyGetRotation(body);
  m[0] = r[0];
  m[1] = r[4];
  m[2] = r[8];
  m[3] = 0;
  m[4] = r[1];
  m[5] = r[5];
  m[6] = r[9];
  m[7] = 0;
  m[8] = r[2];
  m[9] = r[6];
  m[10] = r[10];
  m[11] = 0;
  m[12] = p[0];
  m[13] = p[1];
  m[14] = p[2];
  m[15] = 1;
}

inline void Actuator::deCasteljau(dVector3 p0, dVector3 p1, dVector3 p2,
                                  dVector3 p3, double t, dVector3* p) {
  const int order = 3;

  dVector3 points[4];
  points[0][0] = p0[0];
  points[0][1] = p0[1];
  points[0][2] = p0[2];
  points[1][0] = p1[0];
  points[1][1] = p1[1];
  points[1][2] = p1[2];
  points[2][0] = p2[0];
  points[2][1] = p2[1];
  points[2][2] = p2[2];
  points[3][0] = p3[0];
  points[3][1] = p3[1];
  points[3][2] = p3[2];

  for (int n = order - 1; n >= 0; n--) {
    for (int i = 0; i <= n; i++) {
      points[i][0] = (1 - t) * points[i][0] + t * points[i + 1][0];
      points[i][1] = (1 - t) * points[i][1] + t * points[i + 1][1];
      points[i][2] = (1 - t) * points[i][2] + t * points[i + 1][2];
    }
  }

  (*p)[0] = points[0][0];
  (*p)[1] = points[0][1];
  (*p)[2] = points[0][2];
}

inline void Actuator::drawCylinder(int i, double adjust_beginning,
                                   double adjust_end) {
  double m1[16], m2[16];
  const dReal* current_position = dBodyGetPosition(bodies[2 * i]);
  const dReal* next_position = dBodyGetPosition(bodies[2 * i + 1]);
  const double distance = sqrt((next_position[0] - current_position[0]) *
                                   (next_position[0] - current_position[0]) +
                               (next_position[1] - current_position[1]) *
                                   (next_position[1] - current_position[1]) +
                               (next_position[2] - current_position[2]) *
                                   (next_position[2] - current_position[2]));

  glPushMatrix();
  getMatrix(bodies[2 * i], m1);
  getMatrix(bodies[2 * i + 1], m2);
  for (int j = 0; j < 16; j++) {
    m1[j] = 0.5 * (m1[j] + m2[j]);
  }
  glMultMatrixd(m1);
  glBegin(GL_QUAD_STRIP);  // cylinder
  for (double j = ACTUATOR_DETAIL; j >= 0; j--) {
    glNormal3d(cos(2 * M_PI * j / ACTUATOR_DETAIL),
               sin(2 * M_PI * j / ACTUATOR_DETAIL), 0);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS,
               -distance / 2 - LINK_RADIUS + adjust_beginning);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS,
               distance / 2 + LINK_RADIUS - adjust_end);
  }
  glEnd();
  glPopMatrix();
}

inline void Actuator::drawCap() {
  // caps:

  glTranslated(0, 0, -LINK_RADIUS);
  glBegin(GL_QUAD_STRIP);  // cone
  for (double j = ACTUATOR_DETAIL; j >= 0; j--) {
    glNormal3d(cos(2 * M_PI * j / ACTUATOR_DETAIL),
               sin(2 * M_PI * j / ACTUATOR_DETAIL), 0);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 0.3,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 0.3,
               -LINK_RADIUS / 4);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               LINK_RADIUS);
  }
  glEnd();
  glBegin(GL_QUAD_STRIP);  // cylinder
  for (double j = ACTUATOR_DETAIL; j >= 0; j--) {
    glNormal3d(cos(2 * M_PI * j / ACTUATOR_DETAIL),
               sin(2 * M_PI * j / ACTUATOR_DETAIL), 0);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               LINK_RADIUS);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               LINK_RADIUS * 2);
  }
  glEnd();
  glBegin(GL_POLYGON);  // small disc
  for (double j = ACTUATOR_DETAIL; j >= 0; j--) {
    glNormal3d(cos(2 * M_PI * j / ACTUATOR_DETAIL),
               sin(2 * M_PI * j / ACTUATOR_DETAIL), 0);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 0.3,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 0.3,
               -LINK_RADIUS / 4);
  }
  glEnd();
  glBegin(GL_POLYGON);  // bigger disc
  for (double j = 0; j <= ACTUATOR_DETAIL; j++) {
    glNormal3d(cos(2 * M_PI * j / ACTUATOR_DETAIL),
               sin(2 * M_PI * j / ACTUATOR_DETAIL), 0);
    glVertex3d(cos(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               sin(2 * M_PI * j / ACTUATOR_DETAIL) * LINK_RADIUS * 1.2,
               LINK_RADIUS * 2);
  }
  glEnd();
}

void Actuator::simulate() {
  // pressure:

  double force = calculateForce();
  force *= FORCE_CORRECTION;

  for (int i = 0; i < number_of_links; i++) {
    dJointAddSliderForce(slider_joints[i], force);
  }

  double length_ratio =
      (getLength() - minimum_length) / (maximum_length - minimum_length);

  for (int i = 0; i < number_of_links - 1; i++) {
    dJointSetHingeParam(hinge_joints[i], dParamLoStop,
                        -HINGE_STOP * length_ratio);
    dJointSetHingeParam(hinge_joints[i], dParamHiStop,
                        HINGE_STOP * length_ratio);
  }
}

Muscle::Muscle(dWorldID world, dSpaceID outer_space, dVector3 start_at,
               dVector3 start_control, dVector3 end_control, dVector3 end_at,
               double minimum_length, double maximum_length)
    : Actuator(world, outer_space, start_at, start_control, end_control, end_at,
               minimum_length, maximum_length, false) {}

double Muscle::calculateForce() {
  double length_ratio =
      (getLength() - minimum_length) / (maximum_length - minimum_length);
  double air_pressure_ratio = air_pressure / MAX_AIR_PRESSURE;
  double force =
      GRAVITY * FACTOR_C *
      (air_pressure_ratio * length_ratio +
       FACTOR_A * (1 - air_pressure_ratio) * exp(length_ratio * FACTOR_B));

  return force;
}

LinearActuator::LinearActuator(dWorldID world, dSpaceID outer_space,
                               dVector3 start_at, dVector3 end_at,
                               double minimum_length, double maximum_length)
    : Actuator(world, outer_space, start_at, start_at, end_at, end_at,
               minimum_length, maximum_length, true) {}

double LinearActuator::calculateForce() {
  double length_ratio =
      (getLength() - minimum_length) / (maximum_length - minimum_length);
  double air_pressure_ratio = air_pressure / MAX_AIR_PRESSURE;
  double force =
      GRAVITY * FACTOR_C *
      (air_pressure_ratio * length_ratio +
       FACTOR_A * (1 - air_pressure_ratio) * exp(length_ratio * FACTOR_B));

  return force;
}
