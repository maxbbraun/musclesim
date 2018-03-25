#include <stdio.h>
#include <math.h>

#include "MSYuragi.h"

MSYuragi::MSYuragi(int dim_of_ASM_state, int number_of_attractors)
  : CYuragi(dim_of_ASM_state, number_of_attractors)
{
  Debug("\nconstructor {");

  //attray of f(x)
  dynamics = new double[dim_of_ASM_state];
  //allocate buffer
  error = new double*[number_of_attractors];
  for(int i=0; i<number_of_attractors; i++){
    error[i] = new double[dim_of_ASM_state];
  }
  norm_buf = new double[number_of_attractors];
  gauss_buf = new double[number_of_attractors];

  Debug("} constructor\n");
}

MSYuragi::~MSYuragi(void)
{
  Debug("\ndestructor{ \n");
  Debug("} destructor\n");
}


//calculate dynamics
double* MSYuragi::updateDynamics(double activity)
{
  Debug("\nupdateDynamics {");
  //calculation norm
  for(int i=0; i<number_of_attractors; i++){
    for(int j=0; j<dim_of_ASM_state; j++){
     error[i][j] = attractor[i][j] - pre_ASM_state[j];
    }
    norm_buf[i] = norm(dim_of_ASM_state, error[i]);
    gauss_buf[i] = sqrt(activity)*pow(2*M_PI,-0.5*dim_of_ASM_state)
      *exp(-0.5*activity*pow(norm_buf[i],2.0));
  }

//    fprintf(stderr, "stat : %f %f\n", pre_ASM_state[0], pre_ASM_state[1]);
//    fprintf(stderr, "attr : %f %f\n", attract[0][0], attract[0][1]);
//    fprintf(stderr, "erro : %f %f\n", error[0][0], error[0][1]);
//    fprintf(stderr, "norm : %f %f\n", norm_buf[0], norm_buf[1]);
//    fprintf(stderr, "gaus : %f %f\n", gauss_buf[0], gauss_buf[1]);

  for(int i=0; i<dim_of_ASM_state; i++){
    dynamics[i] = 0.0;
    for(int j=0; j<number_of_attractors; j++){
      dynamics[i] += gauss_buf[j]*error[j][i]*activity;
    }
  }

  double norm_state = norm(dim_of_ASM_state, dynamics);
  for(int i=0; i<dim_of_ASM_state; i++){
    dynamics[i] = 1.0*dynamics[i]/(norm_state+1.0);
  }

  Debug("updateDynamics\n");
  return dynamics;
}


#if 1
//calculate dynamics
double* MSYuragi::updateDynamics(void)
{
  Debug("updateDynamics\n");
  //calculation norm
  double sum = 0.0;
  for(int i=0; i<number_of_attractors; i++){
    for(int j=0; j<dim_of_ASM_state; j++){
     error[i][j] = attractor[i][j] - pre_ASM_state[j];
    }
    norm_buf[i] = norm(dim_of_ASM_state, error[i]);
    gauss_buf[i] = exp(-pow(norm_buf[i],2.0));
    sum += gauss_buf[i];
  }

//    fprintf(stderr, "stat : %f %f\n", pre_ASM_state[0], pre_ASM_state[1]);
//    fprintf(stderr, "attr : %f %f\n", attract[0][0], attract[0][1]);
//    fprintf(stderr, "erro : %f %f\n", error[0][0], error[0][1]);
//    fprintf(stderr, "norm : %f %f\n", norm_buf[0], norm_buf[1]);
//    fprintf(stderr, "gaus : %f %f\n", gauss_buf[0], gauss_buf[1]);

  if(!sum){
    Error("Sum of Gaussian is zero\n");
  }

//   double min = param_K[0];
//   int ind = 0;
//   for(int i=1; i<num; i++){
//     if(param_K[i]<min){
//       min = param_K[i];
//       ind = i;
//     }
//   }

//   double sum = 0.0;
//   for(int i=0; i<num; i++){
//     sum += exp(gain*(-param_K[i]+min));
//   }
//   //soft max method
//   for(int i=0; i<num; i++){
//     param_k[i] = exp(gain*(-param_K[i]+min))/sum;
//   }

  //only
//   for(int i=0; i<num; i++){
//     if(i==ind){
//       param_k[i] = 1.0;
//     }else{
//       param_k[i] = 0.0;
//     }
//   }

  for(int i=0; i<dim_of_ASM_state; i++){
    dynamics[i] = 0.0;
  }

  double tmp;
  for(int i=0; i<number_of_attractors; i++){
    if(!(norm_buf[i])){
      tmp = 1.0;
    }else{
      tmp = norm_buf[i];
    }

    for(int j=0; j<dim_of_ASM_state; j++){
      //      dynamics[j] += param_k[i]*(attract[i][j] - pre_ASM_state[j]);
      dynamics[j] += gauss_buf[i]/sum*(error[i][j])/tmp;
    }
  }

  Debug("updateDynamics\n");
  return dynamics;
}
#endif

#if 0
//calculate dynamics
int CDynamics::updateDynamics(double* f_x, double* x, double activity)
{
  double** tmp = new double*[num];
  for(int i=0; i<num; i++){
    tmp[i] = new double[dim];
    for(int j=0; j<dim; j++){
      tmp[i][j] = attract[i][j] - x[j];
    }
    param_K[i] = norm(tmp[i]);
  }
  for(int i=0; i<num; i++){
    delete [] tmp[i];
  }
  delete [] tmp;

  //soft max method
  for(int i=0; i<num; i++){
    param_k[i] = exp(-activity*(gain*pow(param_K[i],2.0)));
  }

  //f(x)を計算する
  memset(f_x, 0, sizeof(double)*dim);
  //  zero(f_x);
  double tmp2;
  for(int i=0; i<num; i++){
    if(!(param_K[i])){
      tmp2 = 1.0;
    }else{
      tmp2 = param_K[i];
    }
    for(int j=0; j<dim; j++){
      //アトラクタと状態との差の和をとる
      //      f_x[j] += param_k[i]*(attract[i][j] - x[j]);
      f_x[j] += param_k[i]*(attract[i][j] - x[j])/tmp2;
    }
  }

  return 0;
}
#endif


