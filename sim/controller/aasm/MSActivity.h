//
//this class is for reaching task
//
#ifndef __MSACTIVITY_H__
#define __MSACTIVITY_H__

#include "CActivity.h"

class MSActivity : public CActivity
{

 public:
  //constructor
  MSActivity();

  //destructor
  ~MSActivity();

 private:

  //calclate activity
  double calcActivity(double* robotic_state);

  double calcAlpha(double* robotic_state);
  double calcBeta(double* robotic_state);

};

#endif




