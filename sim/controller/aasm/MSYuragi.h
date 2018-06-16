#ifndef __MSYURAGI_H__
#define __MSYURAGI_H__

#include "CYuragi.h"

class MSYuragi : public CYuragi
{
 public:
  //constructor
  //  int dim_of_ASM_state     : dimention of ASM's state
  //  int number_of_attractors : number of attractors
  //  double** attractors       : you can use arbitrary attractors
  MSYuragi(int dim_of_ASM_state, int number_of_attractors);

  //destructor
  ~MSYuragi();

  //
  double* updateDynamics(double activity);


 private:

  //array of f(x)
  double* dynamics;

  //calculate function
  //  return double* : dynamics (f(x))
  double* updateDynamics(void);

  //
  //  double* updateDynamics(double activity);


  //buffer for working
  double** error;
  double* norm_buf;
  double* gauss_buf;

};

#endif




