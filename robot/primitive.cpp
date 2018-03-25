#include "primitive.h"

Primitive::Primitive(dWorldID world, dSpaceID space, dVector3 position) {
  this->world = world;
  this->space = space;

  red = 0.8;
  green = 0.8;
  blue = 0.8;

  all_primitives.push_back(this);

  body = dBodyCreate(world);

  dBodySetPosition(body, position[0], position[1], position[2]);

  parent = NULL;
}

void Primitive::deleteAll() {
  for (vector<Primitive*>::iterator ii = all_primitives.begin();
       ii != all_primitives.end(); ++ii) {
    delete *ii;
  }
}

dBodyID Primitive::getBody() { return hasParent() ? parent->getBody() : body; }

dGeomID Primitive::getGeom() { return geom; }

void Primitive::rotate(dReal angle, dVector3 axis) {
  if (hasParent()) {
    getParent()->rotate(angle, axis);

    return;
  }

  dQuaternion before, after, both;
  const dReal* q = dBodyGetQuaternion(body);
  before[0] = q[0];
  before[1] = q[1];
  before[2] = q[2];
  before[3] = q[3];
  dQFromAxisAndAngle(after, axis[0], axis[1], axis[2], angle * M_PI / 180);
  dQMultiply0(both, after, before);
  dBodySetQuaternion(body, both);
}

void Primitive::fix() {
  if (hasParent()) {
    getParent()->fix();

    return;
  }

  dJointID fix = dJointCreateFixed(world, 0);
  dJointAttach(fix, body, 0);
  dJointSetFixed(fix);
}

void Primitive::setColor(GLfloat red, GLfloat green, GLfloat blue) {
  this->red = red;
  this->green = green;
  this->blue = blue;
}

void Primitive::attachBall(Primitive* primitive, dVector3 anchor) {
  dJointID joint = dJointCreateBall(world, 0);
  dJointAttach(joint, getBody(), primitive ? primitive->getBody() : 0);
  dJointSetBallAnchor(joint, anchor[0], anchor[1], anchor[2]);
}

void Primitive::attachHinge(Primitive* primitive, dVector3 anchor,
                            dVector3 axis) {
  dJointID joint = dJointCreateHinge(world, 0);
  dJointAttach(joint, getBody(), primitive ? primitive->getBody() : 0);
  dJointSetHingeAnchor(joint, anchor[0], anchor[1], anchor[2]);
  dJointSetHingeAxis(joint, axis[0], axis[1], axis[2]);
}

void Primitive::attachHinge(Primitive* primitive, dVector3 anchor,
                            dVector3 axis, double low_stop, double high_stop) {
  dJointID joint = dJointCreateHinge(world, 0);
  dJointAttach(joint, getBody(), primitive ? primitive->getBody() : 0);
  dJointSetHingeAnchor(joint, anchor[0], anchor[1], anchor[2]);
  dJointSetHingeAxis(joint, axis[0], axis[1], axis[2]);
  dJointSetHingeParam(joint, dParamLoStop, low_stop * M_PI / 180);
  dJointSetHingeParam(joint, dParamHiStop, high_stop * M_PI / 180);
}

void Primitive::attachUniversal(Primitive* primitive, dVector3 anchor,
                                dVector3 axis_1, dVector3 axis_2) {
  dJointID joint = dJointCreateUniversal(world, 0);
  dJointAttach(joint, getBody(), primitive ? primitive->getBody() : 0);
  dJointSetUniversalAnchor(joint, anchor[0], anchor[1], anchor[2]);
  dJointSetUniversalAxis1(joint, axis_1[0], axis_1[1], axis_1[2]);
  dJointSetUniversalAxis2(joint, axis_2[0], axis_2[1], axis_2[2]);
}

void Primitive::attachUniversal(Primitive* primitive, dVector3 anchor,
                                dVector3 axis_1, dVector3 axis_2,
                                double low_stop_1, double high_stop_1,
                                double low_stop_2, double high_stop_2) {
  dJointID joint = dJointCreateUniversal(world, 0);
  dJointAttach(joint, getBody(), primitive ? primitive->getBody() : 0);
  dJointSetUniversalAnchor(joint, anchor[0], anchor[1], anchor[2]);
  dJointSetUniversalAxis1(joint, axis_1[0], axis_1[1], axis_1[2]);
  dJointSetUniversalAxis2(joint, axis_2[0], axis_2[1], axis_2[2]);
  dJointSetHingeParam(joint, dParamLoStop, low_stop_1 * M_PI / 180);
  dJointSetHingeParam(joint, dParamHiStop, high_stop_1 * M_PI / 180);
  dJointSetHingeParam(joint, dParamLoStop2, low_stop_2 * M_PI / 180);
  dJointSetHingeParam(joint, dParamHiStop2, high_stop_2 * M_PI / 180);
}

void Primitive::compositeWithPrimitive(Primitive* child) {
  vector<Primitive*> children;
  children.push_back(child);
  compositeWithPrimitives(children);
}

void Primitive::compositePrimitives(vector<Primitive*> primitives) {
  Primitive* last = primitives.back();

  primitives.pop_back();

  last->compositeWithPrimitives(primitives);
}

void Primitive::setVerbose(bool verbose) { Primitive::verbose = verbose; }

void Primitive::draw(GLfloat alpha) {
  GLfloat diffuse[] = {red, green, blue, alpha};
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);

  double m[16];

  getMatrix(geom, m);
  glMultMatrixd(m);

  if (dGeomGetClass(geom) == dGeomTransformClass) {
    getMatrix(dGeomTransformGetGeom(geom), m);
    glMultMatrixd(m);
  }
}

void Primitive::drawAll(GLfloat alpha) {
  // GLfloat specular[] = {0.0, 0.0, 0.0, alpha};
  // glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);

  for (vector<Primitive*>::iterator ii = all_primitives.begin();
       ii != all_primitives.end(); ++ii) {
    (*ii)->draw(alpha);
  }
}

void Primitive::getPosition(double& x, double& y, double& z) {
  const dReal* position = dBodyGetPosition(getBody());

  x = position[0];
  y = position[1];
  z = position[2];
}

vector<Primitive*> Primitive::all_primitives;
bool Primitive::verbose;

inline void Primitive::getMatrix(dGeomID geom, double* m) {
  const dReal* p = dGeomGetPosition(geom);
  const dReal* r = dGeomGetRotation(geom);
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

bool Primitive::hasParent() { return parent != NULL; }

Primitive* Primitive::getParent() { return parent; }

void Primitive::setParent(Primitive* parent) { this->parent = parent; }

void Primitive::setGeom(dGeomID geom) { this->geom = geom; }

void Primitive::removeBody(Primitive* parent) {
  // destroy the old body and point to primitive holding the new body:

  dBodyDestroy(body);
  body = NULL;
  this->parent = parent;
}

void Primitive::compositeWithPrimitives(vector<Primitive*> children) {
  // save masses:

  vector<pair<Primitive*, dMass> > children_and_masses;

  for (vector<Primitive*>::iterator ii = children.begin(); ii != children.end();
       ++ii) {
    dMass child_mass;
    dBodyGetMass((*ii)->getBody(), &child_mass);
    pair<Primitive*, dMass> p(*ii, child_mass);
    children_and_masses.push_back(p);
  }

  // calculate new center of mass:

  dMass this_mass;
  dBodyGetMass(body, &this_mass);
  double mass_sum = this_mass.mass;

  const dReal* this_position_ptr = dGeomGetPosition(geom);
  const dReal this_position[3] = {this_position_ptr[0], this_position_ptr[1],
                                  this_position_ptr[2]};
  double position_x_sum = this_position[0] * this_mass.mass;
  double position_y_sum = this_position[1] * this_mass.mass;
  double position_z_sum = this_position[2] * this_mass.mass;

  for (vector<pair<Primitive*, dMass> >::iterator ii =
           children_and_masses.begin();
       ii != children_and_masses.end(); ++ii) {
    Primitive* child = (*ii).first;
    dMass child_mass = (*ii).second;

    dBodyGetMass(child->getBody(), &child_mass);
    mass_sum += child_mass.mass;

    const dReal* child_position = dGeomGetPosition(child->getGeom());
    position_x_sum += child_position[0] * child_mass.mass;
    position_y_sum += child_position[1] * child_mass.mass;
    position_z_sum += child_position[2] * child_mass.mass;
  }

  const dReal center_of_mass[3] = {position_x_sum / mass_sum,
                                   position_y_sum / mass_sum,
                                   position_z_sum / mass_sum};

  // new mass:

  dMass mass;
  dMassSetZero(&mass);

  // clear geoms:

  dGeomSetBody(geom, 0);
  dSpaceRemove(space, geom);

  for (vector<pair<Primitive*, dMass> >::iterator ii =
           children_and_masses.begin();
       ii != children_and_masses.end(); ++ii) {
    Primitive* child = (*ii).first;

    dGeomSetBody(child->getGeom(), 0);
    dSpaceRemove(space, child->getGeom());
    child->removeBody(this);
  }

  // move body to new center of mass:

  dBodySetPosition(body, center_of_mass[0], center_of_mass[1],
                   center_of_mass[2]);

  // get inverse body rotation:

  const dReal* body_quaternion = dBodyGetQuaternion(body);
  dQuaternion body_inverse = {body_quaternion[0], -body_quaternion[1],
                              -body_quaternion[2], -body_quaternion[3]};
  dMatrix3 body_inverse_matrix;
  dQtoR(body_inverse, body_inverse_matrix);

  // offset this position:

  dReal this_offset_x = this_position[0] - center_of_mass[0];
  dReal this_offset_y = this_position[1] - center_of_mass[1];
  dReal this_offset_z = this_position[2] - center_of_mass[2];

  dGeomSetPosition(geom,
                   body_inverse_matrix[0] * this_offset_x +
                       body_inverse_matrix[1] * this_offset_y +
                       body_inverse_matrix[2] * this_offset_z,
                   body_inverse_matrix[4] * this_offset_x +
                       body_inverse_matrix[5] * this_offset_y +
                       body_inverse_matrix[6] * this_offset_z,
                   body_inverse_matrix[8] * this_offset_x +
                       body_inverse_matrix[9] * this_offset_y +
                       body_inverse_matrix[10] * this_offset_z);

  // offset this rotation:

  dQuaternion this_quaternion;
  dGeomGetQuaternion(geom, this_quaternion);
  dQuaternion this_offset_quaternion;
  dQMultiply1(this_offset_quaternion, body_quaternion, this_quaternion);

  dGeomSetQuaternion(geom, this_offset_quaternion);

  // offset this mass rotation:

  dMatrix3 this_offset_matrix;
  dQtoR(this_offset_quaternion, this_offset_matrix);
  dMassRotate(&this_mass, this_offset_matrix);

  for (vector<pair<Primitive*, dMass> >::iterator ii =
           children_and_masses.begin();
       ii != children_and_masses.end(); ++ii) {
    Primitive* child = (*ii).first;
    dMass child_mass = (*ii).second;

    // offset child position:

    const dReal* child_position = dGeomGetPosition(child->getGeom());
    dReal child_offset_x = child_position[0] - center_of_mass[0];
    dReal child_offset_y = child_position[1] - center_of_mass[1];
    dReal child_offset_z = child_position[2] - center_of_mass[2];

    dGeomSetPosition(child->getGeom(),
                     body_inverse_matrix[0] * child_offset_x +
                         body_inverse_matrix[1] * child_offset_y +
                         body_inverse_matrix[2] * child_offset_z,
                     body_inverse_matrix[4] * child_offset_x +
                         body_inverse_matrix[5] * child_offset_y +
                         body_inverse_matrix[6] * child_offset_z,
                     body_inverse_matrix[8] * child_offset_x +
                         body_inverse_matrix[9] * child_offset_y +
                         body_inverse_matrix[10] * child_offset_z);

    // offset child rotation:

    dQuaternion child_quaternion;
    dGeomGetQuaternion(child->getGeom(), child_quaternion);

    dQuaternion child_offset_quaternion;
    dQMultiply1(child_offset_quaternion, body_quaternion, child_quaternion);

    dGeomSetQuaternion(child->getGeom(), child_offset_quaternion);

    // offset child mass rotation:

    dMatrix3 child_offset_matrix;
    dQtoR(child_offset_quaternion, child_offset_matrix);
    dMassRotate(&child_mass, child_offset_matrix);
  }

  // create geometry transforms:

  dGeomID this_transgeom = dCreateGeomTransform(space);
  dGeomTransformSetCleanup(this_transgeom, 0);
  dGeomTransformSetInfo(this_transgeom, 1);
  dGeomTransformSetGeom(this_transgeom, geom);

  dGeomSetBody(this_transgeom, body);
  setGeom(this_transgeom);

  for (vector<pair<Primitive*, dMass> >::iterator ii =
           children_and_masses.begin();
       ii != children_and_masses.end(); ++ii) {
    Primitive* child = (*ii).first;
    dMass child_mass = (*ii).second;

    dGeomID child_transgeom = dCreateGeomTransform(space);
    dGeomTransformSetCleanup(child_transgeom, 0);
    dGeomTransformSetInfo(child_transgeom, 1);
    dGeomTransformSetGeom(child_transgeom, child->getGeom());

    dGeomSetBody(child_transgeom, body);
    child->setGeom(child_transgeom);
  }

  // update mass:

  dMassAdd(&mass, &this_mass);

  for (vector<pair<Primitive*, dMass> >::iterator ii =
           children_and_masses.begin();
       ii != children_and_masses.end(); ++ii) {
    dMass child_mass = (*ii).second;

    dMassAdd(&mass, &child_mass);
  }

  dBodySetMass(body, &mass);
}

CappedCylinder::CappedCylinder(dWorldID world, dSpaceID space,
                               dVector3 position, dReal mass, dReal density,
                               dReal radius, dReal length)
    : Primitive(world, space, position) {
  this->radius = radius;
  this->length = length;

  dMass m;
  dMassSetCappedCylinder(&m, density, 3, radius, length);
  dMassAdjust(&m, mass);
  dBodySetMass(body, &m);

  geom = dCreateCCylinder(space, radius, length);

  dGeomSetBody(geom, body);

  if (Primitive::verbose) {
    cout << "Capped cylinder created: "
         << "mass = " << mass << ", "
         << "density = " << density << ", "
         << "radius = " << radius << ", "
         << "length = " << length << endl;
  }
}

void CappedCylinder::draw(GLfloat alpha) {
  glPushMatrix();

  Primitive::draw(alpha);

  dReal length, radius;
  if (dGeomGetClass(geom) == dGeomTransformClass) {
    dGeomCCylinderGetParams(dGeomTransformGetGeom(geom), &radius, &length);
  } else {
    dGeomCCylinderGetParams(geom, &radius, &length);
  }

  // cylinder:

  glBegin(GL_QUAD_STRIP);
  for (double j = 0; j <= PRIMITIVE_DETAIL; j++) {
    glNormal3d(cos(2 * M_PI * j / PRIMITIVE_DETAIL),
               sin(2 * M_PI * j / PRIMITIVE_DETAIL), 0);
    glVertex3d(cos(2 * M_PI * j / PRIMITIVE_DETAIL) * radius,
               sin(2 * M_PI * j / PRIMITIVE_DETAIL) * radius, length / 2);
    glVertex3d(cos(2 * M_PI * j / PRIMITIVE_DETAIL) * radius,
               sin(2 * M_PI * j / PRIMITIVE_DETAIL) * radius, -length / 2);
  }
  glEnd();

  // caps:

  glTranslated(0, 0, -length / 2);
  glutSolidSphere(radius, PRIMITIVE_DETAIL, PRIMITIVE_DETAIL);
  glTranslated(0, 0, length);
  glutSolidSphere(radius, PRIMITIVE_DETAIL, PRIMITIVE_DETAIL);

  glPopMatrix();
}

Sphere::Sphere(dWorldID world, dSpaceID space, dVector3 position, dReal mass,
               dReal density, dReal radius)
    : Primitive(world, space, position) {
  this->radius = radius;

  dMass m;
  dMassSetSphere(&m, density, radius);
  dMassAdjust(&m, mass);
  dBodySetMass(body, &m);

  geom = dCreateSphere(space, radius);

  dGeomSetBody(geom, body);

  if (Primitive::verbose) {
    cout << "Sphere created: "
         << "mass = " << mass << ", "
         << "density = " << density << ", "
         << "radius = " << radius << endl;
  }
}

void Sphere::draw(GLfloat alpha) {
  glPushMatrix();

  Primitive::draw(alpha);

  dReal radius;
  if (dGeomGetClass(geom) == dGeomTransformClass) {
    radius = dGeomSphereGetRadius(dGeomTransformGetGeom(geom));
  } else {
    radius = dGeomSphereGetRadius(geom);
  }

  glutSolidSphere(radius, PRIMITIVE_DETAIL, PRIMITIVE_DETAIL);

  glPopMatrix();
}

Box::Box(dWorldID world, dSpaceID space, dVector3 position, dReal mass,
         dReal density, dReal size_x, dReal size_y, dReal size_z

         )
    : Primitive(world, space, position) {
  this->size_x = size_x;
  this->size_y = size_y;
  this->size_z = size_z;

  dMass m;
  dMassSetBox(&m, density, size_x, size_y, size_z);
  dMassAdjust(&m, mass);
  dBodySetMass(body, &m);

  geom = dCreateBox(space, size_x, size_y, size_z);

  dGeomSetBody(geom, body);

  if (Primitive::verbose) {
    cout << "Box created: "
         << "mass = " << mass << ", "
         << "density = " << density << ", "
         << "size in x = " << size_x << ", "
         << "size in y = " << size_y << ", "
         << "size in z = " << size_z << endl;
  }
}

void Box::draw(GLfloat alpha) {
  glPushMatrix();

  Primitive::draw(alpha);

  dVector3 lengths;
  if (dGeomGetClass(geom) == dGeomTransformClass) {
    dGeomBoxGetLengths(dGeomTransformGetGeom(geom), lengths);
  } else {
    dGeomBoxGetLengths(geom, lengths);
  }

  glScaled(lengths[0], lengths[1], lengths[2]);
  glutSolidCube(1);

  glPopMatrix();
}
