#ifndef __ADAPTIVEATTORACTORSELECTION_H__
#define __ADAPTIVEATTORACTORSELECTION_H__

#include "MSYuragi.h"
#include "MSActivity.h"

class AdaptiveAttractorSelection : public MSYuragi, public MSActivity
{

 public:
  //constructor
  //  int number_of_actuators  : number of actuators,
  //                             that is dimention of state of yuragi
  //  int number_of_attractors : number of attractors
  AdaptiveAttractorSelection(int number_of_actuators, int number_of_attractors);

  //destructor
  ~AdaptiveAttractorSelection();

  //update state of attractor selection model,
  //this method wraps following 2 methods
  double* nextASMState(double* robotic_state);

  //update state with eular method
  // (this method is exactly defined in CYuragi.cpp)
  //  double activity : arbitrary activity (activity >= 1)
  //  return double*  : array of ASM's state
  //                    the range of the state is [-1, 1]
  double* nextASMState(double activity);

  //calculation of activity
  // (this method is defined in CActivity.cpp)
  //  double* robotic_state : the 3D position of the hand
  //  return double         : activity
  //
  //  double calculateActivity(double* robotic_state);

  //this method write activity and distance into "activity.dat",
  //                  state into "state.dat",
  //              and attractors into "attractor.dat" respectively
  int saveData(double time);

 private:

};

#endif
