#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>
using namespace std;

#include <GLUT/glut.h>
#include <math.h>
#include <iostream>
#include "../controller/controller.h"
#include "../robot/actuator.h"
#include "../robot/primitive.h"
#include "../robot/robot.h"
#include "../simulation/simulation.h"

#define PRESSURE_STEP 0.01

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define WINDOW_LEFT 40
#define WINDOW_TOP 60

#define PHI_START (150 * M_PI / 180)
#define THETA_START (60 * M_PI / 180)
#define R_START 1000.0

#define ZOOM_SPEED 10.0
#define ROTATION_SPEED 4.0
#define ROTATION_CENTER_X -150.0
#define ROTATION_CENTER_Y 100.0
#define ROTATION_CENTER_Z 0.0

#define CAGE_RADIUS 2000
#define CAGE_DIVISIONS 30
#define PLANE_DIVISIONS (2 * CAGE_DIVISIONS)

class Display {
 public:
  static void initialize(int argc, char **argv);

  static void mainLoop(Robot *robot, Controller *controller);

 private:
  // window size and position:

  static int window_width;
  static int window_height;
  static int window_left;
  static int window_top;

  // trackball:

  static double camera_x;
  static double camera_y;
  static double camera_z;

  static int last_x;
  static int last_y;

  static double phi;
  static double theta;
  static double r;

  // callback functions:

  static void display();
  static void idle();
  static void mouse(int button, int state, int x, int y);
  static void motion(int x, int y);
  static void special(int key, int x, int y);
  static void keyboard(unsigned char key, int x, int y);
  static void reshape(int new_width, int new_height);

  // simulation:

  static Robot *robot;
  static Controller *controller;
  static bool interactive;
  static bool pause;
  static double global_alpha;
};

#endif
