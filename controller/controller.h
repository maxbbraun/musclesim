#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
using namespace std;

#include <math.h>
#include <fstream>
#include "../controller/aasm/AdaptiveAttractorSelection.h"
#include "../controller/asm/mcmc.hpp"
#include "../simulation/simulation.h"

#define LOG_FILE_PATH "log/reaching_task_data"
#define NUMBER_OF_ACTUATORS 10
#define ACTUATOR_LAG 6

class Controller {
 public:
  Controller();
  virtual ~Controller();

  virtual double* output(double* state) = 0;

  static Controller* getActiveController();  // returns an instance of the
                                             // controller type you want to use
  static void deactivate();

 private:
  static Controller* active_controller;

 protected:
  double* action;

  ofstream log_file;
};

class ASMController : public Controller {
 public:
  ASMController();
  ~ASMController();

  double* output(double* state);

 private:
  double activity;

  gmpdf* yuragi;

  double last_a_1;
  double last_height;
};

class AASMController : public Controller {
 public:
  AASMController();
  ~AASMController();

  double* output(double* state);

 private:
  double activity;

  AdaptiveAttractorSelection* yuragi;

  double* goal;
};

#include <GLUT/glut.h>

#define TARGET_RADIUS 30
#define TARGET_DIVISIONS 30

class Target {
 public:
  static void set(double x, double y, double z);

  static double getX();
  static double getY();
  static double getZ();

  static void draw(GLfloat alpha);

 private:
  static double x;
  static double y;
  static double z;
};

#endif
