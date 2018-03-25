#include <stdio.h>
#include <math.h>

#include "AdaptiveAttractorSelection.h"

//constructor (set parameters)
AdaptiveAttractorSelection::AdaptiveAttractorSelection
(int number_of_actuators, int number_of_attractors)
  : MSYuragi(number_of_actuators, number_of_attractors),
    MSActivity()
{
  Debug("constructor\n");
}

AdaptiveAttractorSelection::~AdaptiveAttractorSelection()
{
  Debug("destructor\n");
}

//update state of attractor selection model
double* AdaptiveAttractorSelection::nextASMState(double* robotic_state)
{
  Debug("nextASMState\n");
  return MSYuragi::nextASMState(calculateActivity(robotic_state));
}

//update state of attractor selection model
double* AdaptiveAttractorSelection::nextASMState(double activity)
{
  Debug("nextASMState\n");
  return MSYuragi::nextASMState(activity);
}

int AdaptiveAttractorSelection::saveData(double time)
{
  saveActivity(time);
  saveASMState(time);
  saveAtractors(time);

  return 0;
}


