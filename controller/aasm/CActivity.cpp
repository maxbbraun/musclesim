#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "CActivity.h"

//constructor
CActivity::CActivity()
{
  Debug("\nconstructor {");

  //activity
  activity = 1.0;
  //  activity = 0.0;
  //error value
  error_value = 1.0;

  //dimention of robotic state
  dim_of_robotic_state = 1;
  //set goal
  goal = new double[dim_of_robotic_state]; assert(goal);
  goal[0] = 1.0;

  //allocate previous state of robot
  pre_robotic_state = new double[dim_of_robotic_state];
  assert(pre_robotic_state);
  pre_robotic_state[0] = 0.0;

  char* filename = "./log/activity.dat";
  if((fp=fopen(filename, "w"))==NULL){
    Error("file open error (%s)...", filename);
  }

  Debug("} constructor\n");
}

//destructor
CActivity::~CActivity()
{
  fclose(fp);
  Debug("destructor\n");
}

//get activity
double CActivity::calculateActivity(double* robotic_state)
{
  Debug("calculateActivity\n");

  //calclate error value between goal and state of robot
  calcErrorValue();

  //calclate activity
  activity = calcActivity(robotic_state);
  //activity must be positive
//   if(activity<1.0){
//     activity = 1.0;
//   }
  if(activity>100.0){
    activity = 100.0;
  }
  if(activity<0.0){
    activity = 0.0;
  }
  //store previous state of robot
  memcpy(pre_robotic_state, robotic_state, sizeof(double)*dim_of_robotic_state);
  Debug("calculateActivity\n");
  return activity;
}

//change the goal
int CActivity::setGoal(int dim_of_robotic_state, const double* goal)
{
  Debug("\nsetGoal {\n");

  this->dim_of_robotic_state = dim_of_robotic_state;
  //set goal
  delete [] this->goal;
  this->goal = new double[dim_of_robotic_state]; assert(this->goal);
  memcpy(this->goal, goal, sizeof(double)*dim_of_robotic_state);

  delete [] pre_robotic_state;
  pre_robotic_state = new double[dim_of_robotic_state];
  for(int i=0; i<dim_of_robotic_state; i++){
    pre_robotic_state[i] = 0.0;
  }

  Debug("} setGoal\n");
  return 0;
}

double CActivity::getErrorValue(void)
{
  return error_value;
}

//this function calculates error value between goal and state of robot
int CActivity::calcErrorValue(void)
{
  Debug("calcErrorValue\n");

  error_value = 0.0;
  for(int i=0; i<dim_of_robotic_state; i++){
    error_value += pow(goal[i]-pre_robotic_state[i], 2.0);
  }
  Debug("calcErrorValue\n");
  return 0;
}


int CActivity::saveActivity(double time)
{
  fprintf(fp, "%f %f %f\n", time, activity, error_value);
  return 0;
}

//this function must be overriden and for calculating activity
double CActivity::calcActivity(double* robotic_state)
{
  Error("This function must be overriden (calcActivity)\n"
	"\tnow, this function return 1.0.\n");
  return 1.0;
}

