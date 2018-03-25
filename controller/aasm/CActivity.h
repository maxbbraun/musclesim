#ifndef __CACTIVITY_H__
#define __CACTIVITY_H__

//output error message
#ifndef Print
#define Print(...) \
  fprintf(stdout, __VA_ARGS__);
#define Error(...) \
  fprintf(stderr, __FILE__ " (%d) : ", __LINE__); \
  fprintf(stderr, __VA_ARGS__);
//#define _DEBUG
#ifndef _DEBUG
#define Debug(...)
#else
#define Debug(...) Error(__VA_ARGS__)
#endif
#endif

class CActivity{
 public:
  //constructor
  CActivity();

  //destructor
  virtual ~CActivity();

  //calculate activity and return activity
  //  double *robotic_state : current state of robot
  //  return double         : activity
  double calculateActivity(double* robotic_state);

  //this method sets goal
  //  int dim_of_robotic_state  : dimention of robotic state
  //  const double* goal        : array of goal of some task
  int setGoal(int dim_of_robotic_state, const double* goal);

  //this function calculates error value between goal and state of robot
  //  return double         : error value
  double getErrorValue(void);

 protected:
  //dimention of target point
  int dim_of_robotic_state;
  //activity
  double activity;
  //goal
  double* goal;

  //error value between goal and state of robot
  double error_value;

  //calclate activity : this is arbitarry function
  virtual double calcActivity(double* robotic_state);

  //this function calculates error value between goal and state of robot
  //and write it and activity into file "activity.dat"
  int saveActivity(double time);

 private:
  //previous value of robotic state
  double* pre_robotic_state;

  //this function calculates error value between goal and state of robot
  int calcErrorValue(void);

  FILE* fp;

};

#endif
