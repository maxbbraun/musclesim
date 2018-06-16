#include "robot.h"

Robot* Robot::active_robot;

Robot* Robot::getActiveRobot() {
  // defines the type of robot to be used:

  active_robot = new RoboticArm();

  return active_robot;
}

void Robot::deactivate() { delete active_robot; }

Robot::Robot() {
  // get ODE world and space:

  world = Simulation::getWorld();
  space = Simulation::getSpace();
}

Robot::~Robot() { delete[] state; }

RoboticArm::RoboticArm() : Robot() {
  // define unit axes for rotations:

  dVector3 x_axis = {1, 0, 0};
  dVector3 y_axis = {0, 1, 0};
  dVector3 z_axis = {0, 0, 1};

  // spine:

  // dVector3 spine_position = {0, 0, 0};
  // Box* spine = new Box(
  // 	world, space,
  // 	spine_position,
  // 	80, // mass
  // 	1, // density
  // 	50, // x
  // 	1000, // y
  // 	50 // z
  // );
  //
  // spine->fix();

  // ribs:

  dVector3 rib_1_2_position = {-250 + 230, 425, 110};
  Box* rib_1_2 = new Box(world, space, rib_1_2_position,
                         10,                                   // mass
                         1,                                    // density
                         40 + 5 * tan(atan(20.0 / 30) / 2.0),  // x
                         30,                                   // y
                         5                                     // z
  );

  dVector3 rib_1_2_rest_position = {-250 + 230 - 35, 425, 100};
  Box* rib_1_2_rest =
      new Box(world, space, rib_1_2_rest_position,
              10,  // mass
              1,   // density
              sqrt(20 * 20 + 30 * 30) + 5 * tan(atan(20.0 / 30) / 2.0),  // x
              30,                                                        // y
              5                                                          // z
      );
  rib_1_2_rest->rotate(-180 / M_PI * atan(20.0 / 30), y_axis);

  rib_1_2->compositeWithPrimitive(rib_1_2_rest);
  rib_1_2->rotate(-10, x_axis);
  rib_1_2->rotate(-2, z_axis);
  rib_1_2->fix();

  dVector3 rib_3_4_position = {-250 + 225, 385, 125};
  Box* rib_3_4 = new Box(world, space, rib_3_4_position,
                         10,                                   // mass
                         1,                                    // density
                         50 + 5 * tan(atan(30.0 / 40) / 2.0),  // x
                         30,                                   // y
                         5                                     // z
  );

  dVector3 rib_3_4_rest_position = {-250 + 225 - 45, 385, 110};
  Box* rib_3_4_rest =
      new Box(world, space, rib_3_4_rest_position,
              10,  // mass
              1,   // density
              sqrt(30 * 30 + 40 * 40) + 5 * tan(atan(30.0 / 40) / 2.0),  // x
              30,                                                        // y
              5                                                          // z
      );
  rib_3_4_rest->rotate(-180 / M_PI * atan(30.0 / 40), y_axis);

  rib_3_4->compositeWithPrimitive(rib_3_4_rest);
  rib_3_4->rotate(-10, x_axis);
  rib_3_4->rotate(-2, z_axis);
  rib_3_4->fix();

  vector<Primitive*> rib_5_6_parts;

  dVector3 rib_5_6_position = {-250 + 220, 345, 140};
  Box* rib_5_6 = new Box(world, space, rib_5_6_position,
                         10,                                   // mass
                         1,                                    // density
                         60 + 5 * tan(atan(50.0 / 60) / 2.0),  // x
                         30,                                   // y
                         5                                     // z
  );
  rib_5_6_parts.push_back(rib_5_6);

  dVector3 rib_5_6_rest_1_position = {-250 + 220 - 60, 345, 140 - 25};
  Box* rib_5_6_rest_1 =
      new Box(world, space, rib_5_6_rest_1_position,
              10,  // mass
              1,   // density
              sqrt(50 * 50 + 60 * 60) + 5 * tan(atan(50.0 / 60) / 2.0),  // x
              30,                                                        // y
              5                                                          // z
      );
  rib_5_6_rest_1->rotate(-180 / M_PI * atan(50.0 / 60), y_axis);
  rib_5_6_parts.push_back(rib_5_6_rest_1);

  dVector3 rib_5_6_rest_2_position = {-250 + 220 - 60 - 35, 345, 140 - 25 - 75};
  Box* rib_5_6_rest_2 =
      new Box(world, space, rib_5_6_rest_2_position,
              10,  // mass
              1,   // density
              sqrt(100 * 100 + 10 * 10) + 5 * tan(atan(50.0 / 60) / 2.0),  // x
              30,                                                          // y
              5                                                            // z
      );
  rib_5_6_rest_2->rotate(-180 / M_PI * atan(100.0 / 10), y_axis);
  rib_5_6_parts.push_back(rib_5_6_rest_2);

  Primitive::compositePrimitives(rib_5_6_parts);
  rib_5_6->rotate(-6, x_axis);
  rib_5_6->fix();

  vector<Primitive*> rib_7_8_parts;

  dVector3 rib_7_8_position = {-250 + 220, 305, 155};
  Box* rib_7_8 = new Box(world, space, rib_7_8_position,
                         10,                                   // mass
                         1,                                    // density
                         60 + 5 * tan(atan(70.0 / 80) / 2.0),  // x
                         30,                                   // y
                         5                                     // z
  );
  rib_7_8_parts.push_back(rib_7_8);

  dVector3 rib_7_8_rest_1_position = {-250 + 220 - 70, 305, 155 - 35};
  Box* rib_7_8_rest_1 =
      new Box(world, space, rib_7_8_rest_1_position,
              10,  // mass
              1,   // density
              sqrt(70 * 70 + 80 * 80) + 5 * tan(atan(70.0 / 80) / 2.0),  // x
              30,                                                        // y
              5                                                          // z
      );
  rib_7_8_rest_1->rotate(-180 / M_PI * atan(70.0 / 80), y_axis);
  rib_7_8_parts.push_back(rib_7_8_rest_1);

  dVector3 rib_7_8_rest_2_position = {-250 + 220 - 70 - 45, 305, 155 - 35 - 85};
  Box* rib_7_8_rest_2 =
      new Box(world, space, rib_7_8_rest_2_position,
              10,  // mass
              1,   // density
              sqrt(100 * 100 + 10 * 10) + 5 * tan(atan(70.0 / 80) / 2.0),  // x
              30,                                                          // y
              5                                                            // z
      );
  rib_7_8_rest_2->rotate(-180 / M_PI * atan(100.0 / 10), y_axis);
  rib_7_8_parts.push_back(rib_7_8_rest_2);

  Primitive::compositePrimitives(rib_7_8_parts);
  rib_7_8->rotate(-5, x_axis);
  rib_7_8->fix();

  vector<Primitive*> rib_9_10_parts;

  dVector3 rib_9_10_position = {-250 + 220, 265, 170};
  Box* rib_9_10 = new Box(world, space, rib_9_10_position,
                          10,                                   // mass
                          1,                                    // density
                          60 + 5 * tan(atan(70.0 / 80) / 2.0),  // x
                          30,                                   // y
                          5                                     // z
  );
  rib_9_10_parts.push_back(rib_9_10);

  dVector3 rib_9_10_rest_1_position = {-250 + 220 - 70, 265, 170 - 35};
  Box* rib_9_10_rest_1 =
      new Box(world, space, rib_9_10_rest_1_position,
              10,  // mass
              1,   // density
              sqrt(70 * 70 + 80 * 80) + 5 * tan(atan(70.0 / 80) / 2.0),  // x
              30,                                                        // y
              5                                                          // z
      );
  rib_9_10_rest_1->rotate(-180 / M_PI * atan(70.0 / 80), y_axis);
  rib_9_10_parts.push_back(rib_9_10_rest_1);

  dVector3 rib_9_10_rest_2_position = {-250 + 220 - 70 - 45, 265,
                                       170 - 35 - 85};
  Box* rib_9_10_rest_2 =
      new Box(world, space, rib_9_10_rest_2_position,
              10,  // mass
              1,   // density
              sqrt(100 * 100 + 10 * 10) + 5 * tan(atan(70.0 / 80) / 2.0),  // x
              30,                                                          // y
              5                                                            // z
      );
  rib_9_10_rest_2->rotate(-180 / M_PI * atan(100.0 / 10), y_axis);
  rib_9_10_parts.push_back(rib_9_10_rest_2);

  Primitive::compositePrimitives(rib_9_10_parts);
  rib_9_10->rotate(-5, x_axis);
  rib_9_10->rotate(1, z_axis);
  rib_9_10->fix();

  vector<Primitive*> rib_11_12_parts;

  dVector3 rib_11_12_position = {-250 + 220, 225, 170};
  Box* rib_11_12 = new Box(world, space, rib_11_12_position,
                           10,                                   // mass
                           1,                                    // density
                           60 + 5 * tan(atan(50.0 / 60) / 2.0),  // x
                           30,                                   // y
                           5                                     // z
  );
  rib_11_12_parts.push_back(rib_11_12);

  dVector3 rib_11_12_rest_1_position = {-250 + 220 - 60, 225, 170 - 25};
  Box* rib_11_12_rest_1 =
      new Box(world, space, rib_11_12_rest_1_position,
              10,  // mass
              1,   // density
              sqrt(50 * 50 + 60 * 60) + 5 * tan(atan(50.0 / 60) / 2.0),  // x
              30,                                                        // y
              5                                                          // z
      );
  rib_11_12_rest_1->rotate(-180 / M_PI * atan(50.0 / 60), y_axis);
  rib_11_12_parts.push_back(rib_11_12_rest_1);

  dVector3 rib_11_12_rest_2_position = {-250 + 220 - 60 - 35, 225,
                                        170 - 25 - 65};
  Box* rib_11_12_rest_2 =
      new Box(world, space, rib_11_12_rest_2_position,
              10,  // mass
              1,   // density
              sqrt(80 * 80 + 10 * 10) + 5 * tan(atan(50.0 / 60) / 2.0),  // x
              30,                                                        // y
              5                                                          // z
      );
  rib_11_12_rest_2->rotate(-180 / M_PI * atan(80.0 / 10), y_axis);
  rib_11_12_parts.push_back(rib_11_12_rest_2);

  Primitive::compositePrimitives(rib_11_12_parts);
  rib_11_12->rotate(-2, x_axis);
  rib_11_12->rotate(2, z_axis);
  rib_11_12->fix();

  // scapula:

  vector<Primitive*> scapula_parts;

  dVector3 scapula_position = {-250 + 60, 330, -120};
  Box* scapula = new Box(world, space, scapula_position,
                         80,   // mass
                         1,    // density
                         70,   // x
                         170,  // y
                         10    // z
  );
  scapula_parts.push_back(scapula);

  dVector3 scapula_rest_1_position = {-250 + 60 - 35, 330, -120};
  Box* scapula_rest_1 = new Box(world, space, scapula_rest_1_position,
                                80,                              // mass
                                1,                               // density
                                5 * sqrt(34 * (17 - sqrt(93))),  // x
                                5 * sqrt(34 * (17 + sqrt(93))),  // y
                                10                               // z
  );
  scapula_rest_1->rotate(
      180 / M_PI * acos(70 / (5 * sqrt(34 * (17 - sqrt(93))))), z_axis);
  scapula_parts.push_back(scapula_rest_1);

  dVector3 scapula_rest_2_position = {-250 + 60 - 35 - 30, 330 + 80, -120};
  Box* scapula_rest_2 = new Box(world, space, scapula_rest_2_position,
                                80,  // mass
                                1,   // density
                                30,  // x
                                80,  // y
                                5    // z
  );
  scapula_rest_2->rotate(
      180 / M_PI * acos(70 / (5 * sqrt(34 * (17 - sqrt(93))))), z_axis);
  scapula_parts.push_back(scapula_rest_2);

  Primitive::compositePrimitives(scapula_parts);

  scapula->rotate(20, z_axis);
  scapula->rotate(60, y_axis);

  scapula->fix();

  // humerus:

  dVector3 humerus_position = {-250 + 40, 260, 0};
  CappedCylinder* humerus = new CappedCylinder(world, space, humerus_position,
                                               200,  // mass
                                               1,    // density
                                               13,   // radius
                                               300   // length
  );
  humerus->rotate(90, x_axis);

  dVector3 shoulder_position = {-250 + 40, 410, 0};
  Sphere* shoulder = new Sphere(world, space, shoulder_position,
                                30,  // mass
                                1,   // density
                                20   // radius
  );

  dVector3 elbow_position = {-250 + 40, 110, 0};
  Sphere* elbow = new Sphere(world, space, elbow_position,
                             30,  // mass
                             1,   // density
                             20   // radius
  );

  vector<Primitive*> humerus_parts;
  humerus_parts.push_back(humerus);
  humerus_parts.push_back(shoulder);
  humerus_parts.push_back(elbow);
  Primitive::compositePrimitives(humerus_parts);

  humerus->attachBall(ENVIRONMENT, shoulder_position);

  // radius:

  dVector3 radius_head_position = {-250 + 40, -150, 0};
  CappedCylinder* radius_head =
      new CappedCylinder(world, space, radius_head_position,
                         20,  // mass
                         1,   // density
                         6,   // radius
                         40   // length
      );
  radius_head->rotate(90, y_axis);

  dVector3 radius_position = {-250 + 20, -35, 0};
  CappedCylinder* radius = new CappedCylinder(world, space, radius_position,
                                              100,  // mass
                                              1,    // density
                                              6,    // radius
                                              230   // length
  );
  radius->rotate(90, x_axis);

  radius->compositeWithPrimitive(radius_head);

  // ulna:

  vector<Primitive*> ulna_parts;

  dVector3 ulna_head_position = {-250 + 40, 80, 0};
  CappedCylinder* ulna_head =
      new CappedCylinder(world, space, ulna_head_position,
                         20,  // mass
                         1,   // density
                         6,   // radius
                         40   // length
      );
  ulna_head->rotate(90, y_axis);
  ulna_parts.push_back(ulna_head);

  dVector3 ulna_position = {-250 + 60, -35, 0};
  CappedCylinder* ulna = new CappedCylinder(world, space, ulna_position,
                                            100,  // mass
                                            1,    // density
                                            6,    // radius
                                            230   // length
  );
  ulna->rotate(90, x_axis);
  ulna_parts.push_back(ulna);

  dVector3 ulna_rest_position = {-250 + 40, 80, -30};
  Box* ulna_rest = new Box(world, space, ulna_rest_position,
                           20,  // mass
                           1,   // density
                           16,  // x
                           8,   // y
                           60   // z
  );
  ulna_parts.push_back(ulna_rest);

  Primitive::compositePrimitives(ulna_parts);

  dVector3 coronoid_position = {-250 + 20, 80, 0};
  radius->attachBall(ulna, coronoid_position);

  dVector3 styloid_position = {-250 + 60, -150, 0};
  radius->attachBall(ulna, styloid_position);

  ulna->attachHinge(elbow, elbow_position, x_axis, -90, 0);

  // hand:

  vector<Primitive*> hand_parts;

  dVector3 pollex_position = {-250 + 0, -240.5, 0};
  Box* pollex = new Box(world, space, pollex_position,
                        10,   // mass
                        1,    // density
                        22,   // x
                        105,  // y
                        14    // z
  );
  hand_parts.push_back(pollex);

  dVector3 index_position = {-250 + 25, -257.5, 0};
  Box* index = new Box(world, space, index_position,
                       10,   // mass
                       1,    // density
                       16,   // x
                       185,  // y
                       14    // z
  );
  hand_parts.push_back(index);

  dVector3 medius_position = {-250 + 45, -267.5, 0};
  Box* medius = new Box(world, space, medius_position,
                        10,   // mass
                        1,    // density
                        16,   // x
                        205,  // y
                        14    // z
  );
  hand_parts.push_back(medius);

  dVector3 medius_rest_position = {-250 + 45, -165 - 4, 0};
  Box* medius_rest = new Box(world, space, medius_rest_position,
                             10,     // mass
                             1,      // density
                             16,     // x
                             8,      // y
                             5 * 14  // z
  );
  hand_parts.push_back(medius_rest);

  dVector3 anularius_position = {-250 + 65, -257.5, 0};
  Box* anularius = new Box(world, space, anularius_position,
                           10,   // mass
                           1,    // density
                           16,   // x
                           185,  // y
                           14    // z
  );
  hand_parts.push_back(anularius);

  dVector3 minimus_position = {-250 + 85, -255, 0};
  Box* minimus = new Box(world, space, minimus_position,
                         10,   // mass
                         1,    // density
                         16,   // x
                         140,  // y
                         14    // z
  );
  hand_parts.push_back(minimus);

  Primitive::compositePrimitives(hand_parts);

  pollex->attachHinge(radius, styloid_position, x_axis, -60, 60);

  // save hand for reaching task:

  hand = medius;

  // muscles:

  dVector3 muscle_0_start_at = {-250 + 53, 310, 0};
  dVector3 muscle_0_start_control = {-250 + 53 + 60, 310, 20};
  dVector3 muscle_0_end_control = {-250 + 230 - 35 - 80, 425, 100 + 30};
  dVector3 muscle_0_end_at = {-250 + 230 - 35, 425, 100};
  Muscle* muscle_0 =
      new Muscle(world, space, muscle_0_start_at, muscle_0_start_control,
                 muscle_0_end_control, muscle_0_end_at, 75, 300);
  muscle_0->setColor(0.75, 0.75, 0.75);  // gray

  muscle_0->connectStartTo(humerus);
  muscle_0->connectEndTo(rib_1_2);

  dVector3 muscle_1_start_at = {-250 + 53, 310 - 15, 0};
  dVector3 muscle_1_start_control = {-250 + 53 + 40, 310 - 15, 50};
  dVector3 muscle_1_end_conrol = {-250 + 220 - 90 - 80, 225, 170 - 80};
  dVector3 muscle_1_end_at = {-250 + 220 - 90, 225, 170 - 50};
  Muscle* muscle_1 =
      new Muscle(world, space, muscle_1_start_at, muscle_1_start_control,
                 muscle_1_end_conrol, muscle_1_end_at, 75, 300);
  muscle_1->setColor(1.0, 0.73, 0.8);  // pink

  muscle_1->connectStartTo(humerus);
  muscle_1->connectEndTo(rib_11_12);

  dVector3 muscle_2_start_at = {-250 + 27, 410 - 50, 0};
  dVector3 muscle_2_start_control = {-250 + 27 - 70, 410 - 50 + 50, -40};
  dVector3 muscle_2_end_control = {-250 + 40 - 20 - 50, 330 + 30, -120 + 20};
  dVector3 muscle_2_end_at = {-250 + 40 - 20, 330 + 30, -120};
  Muscle* muscle_2 =
      new Muscle(world, space, muscle_2_start_at, muscle_2_start_control,
                 muscle_2_end_control, muscle_2_end_at, 75, 200);
  muscle_2->setColor(0.73, 0.09, 0.15);  // brown

  muscle_2->connectStartTo(humerus);
  muscle_2->connectEndTo(scapula);

  dVector3 muscle_3_start_at = {-250 + 27, 260 - 50, 0};
  dVector3 muscle_3_start_control = {-250 + 27 - 30, 260 - 50 + 20, 0};
  dVector3 muscle_3_end_control = {-250 - 4 + 60, 410 + 5 - 10, -70 + 10 + 50};
  dVector3 muscle_3_end_at = {-250 - 4, 410 + 5, -70 + 10};
  Muscle* muscle_3 =
      new Muscle(world, space, muscle_3_start_at, muscle_3_start_control,
                 muscle_3_end_control, muscle_3_end_at, 75, 300);
  muscle_3->setColor(1.0, 0.0, 0.0);  // red

  muscle_3->connectStartTo(humerus);
  muscle_3->connectEndTo(scapula);

  dVector3 muscle_4_start_at = {-250 + 40, 260, -13};
  dVector3 muscle_4_start_control = {-250 + 40, 260 - 40, -13 - 30};
  dVector3 muscle_4_end_control = {-250 + 40, 80 + 4 + 80, -6 - 45};
  dVector3 muscle_4_end_at = {-250 + 40, 80 + 4, -6 - 45};
  Muscle* muscle_4 =
      new Muscle(world, space, muscle_4_start_at, muscle_4_start_control,
                 muscle_4_end_control, muscle_4_end_at, 75, 300);
  muscle_4->setColor(1.0, 0.63, 0.0);  // orange

  muscle_4->connectStartTo(humerus);
  muscle_4->connectEndTo(ulna_rest);

  dVector3 muscle_5_start_at = {-250 + 40, 260, 13};
  dVector3 muscle_5_start_control = {-250 + 40, 260 - 20, 13 + 20};
  dVector3 muscle_5_end_control = {-250 + 60, 80 - 60 + 10, -6 + 60};
  dVector3 muscle_5_end_at = {-250 + 60, 80 - 60, 6};
  Muscle* muscle_5 =
      new Muscle(world, space, muscle_5_start_at, muscle_5_start_control,
                 muscle_5_end_control, muscle_5_end_at, 90, 300);
  muscle_5->setColor(0.99, 1.0, 0.0);  // yellow

  muscle_5->connectStartTo(humerus);
  muscle_5->connectEndTo(ulna);

  dVector3 muscle_6_start_at = {-250 + 40 - 10 * sqrt(2), 110, -10 * sqrt(2)};
  dVector3 muscle_6_start_control = {-250 + 40 - 30, 110 - 20, -30};
  dVector3 muscle_6_end_control = {-250 + 20, -35 + 20, -6 - 40};
  dVector3 muscle_6_end_at = {-250 + 20, -35 + 20, -6};
  Muscle* muscle_6 =
      new Muscle(world, space, muscle_6_start_at, muscle_6_start_control,
                 muscle_6_end_control, muscle_6_end_at, 60, 200);
  muscle_6->setColor(0.0, 1.0, 0.0);  // green

  muscle_6->connectStartTo(elbow);
  muscle_6->connectEndTo(radius);

  dVector3 muscle_7_start_at = {-250 + 40, 110 + 40, 13};
  dVector3 muscle_7_start_control = {-250 + 40 - 40, 110 + 40 - 20, 13 + 40};
  dVector3 muscle_7_end_control = {-250 + 20, -35 - 60, 6 + 20};
  dVector3 muscle_7_end_at = {-250 + 20, -35 - 60, 6};
  Muscle* muscle_7 =
      new Muscle(world, space, muscle_7_start_at, muscle_7_start_control,
                 muscle_7_end_control, muscle_7_end_at, 60, 350);
  muscle_7->setColor(0.25, 1.0, 0.83);  // aquamarine

  muscle_7->connectStartTo(elbow);
  muscle_7->connectEndTo(radius);

  dVector3 muscle_8_start_at = {-250 + 60, -35, 6};
  dVector3 muscle_8_start_control = {-250 + 60, -35, 6 + 40};
  dVector3 muscle_8_end_control = {-250 + 45, -165 + 40, 25 + 10};
  dVector3 muscle_8_end_at = {-250 + 45, -165, 25};
  Muscle* muscle_8 =
      new Muscle(world, space, muscle_8_start_at, muscle_8_start_control,
                 muscle_8_end_control, muscle_8_end_at, 60, 200);
  muscle_8->setColor(0.1, 0.0, 1.0);  // blue

  muscle_8->connectStartTo(ulna);
  muscle_8->connectEndTo(medius_rest);

  dVector3 muscle_9_start_at = {-250 + 60, -35, -6};
  dVector3 muscle_9_start_control = {-250 + 60, -35, -6 - 40};
  dVector3 muscle_9_end_control = {-250 + 45, -165 + 40, -25 - 10};
  dVector3 muscle_9_end_at = {-250 + 45, -165, -25};
  Muscle* muscle_9 =
      new Muscle(world, space, muscle_9_start_at, muscle_9_start_control,
                 muscle_9_end_control, muscle_9_end_at, 60, 200);
  muscle_9->setColor(0.73, 0.0, 0.96);  // purple

  muscle_9->connectStartTo(ulna);
  muscle_9->connectEndTo(medius_rest);

  // allocate memory for the state vector:

  state = new double[3];
}

double* RoboticArm::getState() {
  double x, y, z;
  hand->getPosition(x, y, z);

  state[0] = x;
  state[1] = y;
  state[2] = z;

  return state;
}

void RoboticArm::step(double* action) {
  for (int i = 0; i < Muscle::getNumberOfActuators(); i++) {
    Actuator::setAirPressure(i, action[i]);
  }
}
