#ifndef __CYURAGI_H__
#define __CYURAGI_H__

#include "GMixture.h"

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

//vevtor utility
int zero(int dim, double* vector);
double norm(int dim, double* vector);
double inner(int dim, double* vector1, double* vector2);

class CYuragi
{
 public:
  //constructor
  //  int dim_of_ASM_state     : dimention of ASM's state
  //  int number_of_attractors : number of attractors
  CYuragi(int dim_of_ASM_state, int number_of_attractors);

  //destructor
  virtual ~CYuragi();

  //update state with eular method
  //  double activity : arbitrary activity 
  //  return double*  : array of ASM's state
  double* nextASMState(double activity);

  //get state accoding to index
  //  int index       : index of attractor
  //  return double*  : array of ASM's state (exactly this is just attractor)
  double* nextASMStateByAattractor(int index);

  //set attractors
  //  double** attractors : array of arbitrary attractors
  int setAttractors(double** attractors);

  //get attractors
  //  return double** : array of attractors
  double** getAttractors(void);

  //calculate length |Xi - x|
  //  double* Euclid_distance : array of Euclid distance
  //                            between attractor and ASM's state
  //  return int              : index of attractor with minimum distance
  int whichAttractor(double* Euclid_distance);

  //this method resets state
  //  double* ASM_state : arbitrary state
  //                      if this argument is NULL, random number is set
  int resetASMState(double* ASM_state=NULL);

  //this method resets state
  //  double uniform_state : arbitrary state
  int resetASMState(double uniform_state);

  //this method determines whether attractors are adaptive or not
  //  bool on : true -> adaptive (default), false -> not adaptive
  int adaptiveAttractor(bool on);

  //this method return whether this class uses gaussian method or not
  //  return bool : yes or no
  bool isAdaptive(void);

 protected:

  //dimention of ASM's state
  int dim_of_ASM_state;
  //number of attractor
  int number_of_attractors;
  //attractor
  double** attractor;
  //previous state
  double* pre_ASM_state;

  //calculate function
  //  return double* : array of dynamics (f(x))
  virtual double* updateDynamics(void);

  //another version of calculating dynamics
  virtual double* updateDynamics(double activity);

  //this method write state of attractor selection model into file "state.dat"
  int saveASMState(double time);

  //this method write attractors into file "attractor.dat"
  int saveAtractors(double time);

  //Gaussian Mixture
  GMixture* gm;



 private:
  //time step
  double delta_t;
  //time const
  double tau;

  //variance of noise
  double variance;

  //whether this class uses gaussian method or not
  bool adaptive;

  //file pointors
  FILE* state_fp;
  FILE* attractor_fp;

  //calclation of dx
  double* xDot(double activity);

  //normal distribution
  double getNoise(double mean, double s2);

  //this methos sets random seed and write seed into "yuragi.log"
  int setRandSeed(unsigned int seed=0);

};

#endif


