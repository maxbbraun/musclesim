#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>//for time
#include <stdlib.h>//for srand and rand

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

GMixture::GMixture(int dimention, int number)
{
  //parameters
  //
  //default : kmeans off
  kmeans_method = false;
  //dimention of data
  this->dimention = dimention;
  //number of cluster
  this->number = number;
  //learning coefficient
  learning = 0.001;
  //variance of gaussian
  variance = pow(1.0/((double)number+1.0),2.0);
  //update step
  update_step = 1000;

  updated = true;

  //allocate
  //
  //<weight> of each cluster
  ex_w = new double[number];
  //centroid and <weight*data>
  centroid = new double*[number];
  ex_wd = new double*[number];
  for(int i=0; i<number; i++){
    centroid[i] = new double[dimention];
    ex_wd[i] = new double[dimention];
  }

  //initialize centroids of random number
  randomCentroid();

  //initialize
  for(int i=0; i<number; i++){
    memcpy(ex_wd[i], centroid[i], sizeof(double)*dimention);
    //initial weight is 1.0
    ex_w[i] = 1.0;
  }
}

GMixture::~GMixture()
{
  delete [] ex_w;

  for(int i=0; i<number; i++){
    delete [] ex_wd[i];
    delete [] centroid[i];
  }
  delete [] ex_wd;
  delete [] centroid;
}

//this method sets arbitrary centroid
int GMixture::setCentroid(double** centroids)
{
  for(int i=0; i<number; i++){
    memcpy(centroid[i], centroids[i], sizeof(double)*dimention);
    memcpy(ex_wd[i], centroids[i], sizeof(double)*dimention);
    //weight
    ex_w[i] = 1.0;
  }
  return 0;
}

//this method returns pointer of centroids
double** GMixture::getCentroid(void)
{
  return centroid;
}

double** GMixture::getGridCentroid(int zero, int marge)
{
  //initialize centroid from -1.0 to 1.0
  layoutGrid(centroid, dimention, number, zero ,marge);

  //copy
  for(int i=0; i<number; i++){
    memcpy(ex_wd[i], centroid[i], sizeof(double)*dimention);
    ex_w[i] = 1.0;
  }

  return centroid;
}

unsigned int GMixture::setRandSeed(unsigned int seed)
{
  //seed of random number
  char* file = "log/gmixture.log";
  FILE* fp;
  if((fp=fopen(file, "a"))==NULL){
    Error("file open error (%s)...\n", file);
    exit(EXIT_FAILURE);
  }
  unsigned t;
  if(!seed){
    t = (unsigned)time(NULL);
  }else{
    t = seed;
  }
  fprintf(fp, "seed of random number is %d\n", t);
  fclose(fp);
  srandom(t);
  return t;
}

double** GMixture::update(double* sample)
{
  //weight equal 1.0
  return update(1.0, sample);
}

double** GMixture::update(double weight, double* sample)
{
  //count up
  count++;

  //gaussian mixture
  double* g = new double[number];
  double sum_g = 0.0;
  for(int i=0; i<number; i++){
    g[i] = gauss(sample, i);
    sum_g += g[i];
  }

  if(sum_g!=0.0){
    for(int i=0; i<number; i++){
      g[i] /= sum_g;
    }
  }

  if(kmeans_method){
    //k-means method
    int ind = 0;
    double max = g[ind];
    for(int i=1; i<number; i++){
      if(g[i]>max){
	max = g[i];
	ind = i;
      }
    }
    for(int i=0; i<number; i++){
      g[i] = ((i==ind)?1.0:0.0);
    }
  }

  for(int i=0; i<number; i++){
    //update <weight>
    ex_w[i] = (1.0-learning*g[i])*ex_w[i] + learning*g[i]*weight;
    //update <weight*data>
    for(int j=0; j<dimention; j++){
      ex_wd[i][j] = (1.0-learning*g[i])*ex_wd[i][j]+learning*g[i]*weight*sample[j];
    }
  }

  //gaussian mixture
  if(!(count%=update_step)){
    //    fprintf(stderr, "attractors are updated.\n");
    updated = true;
    for(int i=0; i<number; i++){
      if(ex_w[i]!=0.0){
	for(int j=0; j<dimention; j++){
	  centroid[i][j] = ex_wd[i][j]/ex_w[i];
	}
      }
    }
  }

  delete [] g;

#if _DEBUG
  //debug
  for(int i=0; i<num; i++){
    fprintf(stderr, "g[%d] = %f\n", i, g[i]);
  }
  fprintf(stderr, "sum_g = %f\n", sum_g);
  for(int i=0; i<num; i++){
    fprintf(stderr, "ex_w[%d] = %f\n", i, ex_w[i]);
  }
  fprintf(stderr, "centroid:\n");
  for(int i=0; i<num; i++){
    for(int j=0; j<dim; j++){
      fprintf(stderr, "%f ", this->centroid[i][j]);
    }
    fprintf(stderr, "\n");
  }
#endif

  return centroid;
}

bool GMixture::centroidUpdated(void)
{
  bool ret = updated;
  updated = false;
  return ret;
}

//this method calculate random centroid
int GMixture::randomCentroid(void)
{
  //seed of random number
  setRandSeed(0);
  //random number from -1.0 to 1.0  
  for(int i=0; i<number; i++){
    for(int j=0; j<dimention; j++){
      centroid[i][j] = (double)random()/(double)RAND_MAX*2.0-1.0;
    }
  }
  return 0;
}

int GMixture::kmeansOn(bool on)
{
  kmeans_method = on;
  return 0;
}
//calclation gaussian distribution
double GMixture::gauss(double* sample, int index)
{
  double tmp=0.0;
  for(int i=0; i<dimention; i++){
    tmp += pow(sample[i]-centroid[index][i], 2.0);
  }
  double ret = exp(-tmp/(2*variance))/pow(2*M_PI, dimention/2.0)/sqrt(variance);

  return ret;
}
//recursive call for get centroid
int GMixture::grid(double** centroid, double len,
		   int nmax, int n, int dim, int marge)
{
  if(dim==-1){
    return 1;
  }
  int buf1=0;
  int buf2=0;
  int buf3=0;
  //now dimention
  for(int i=0; i<nmax; i++){
    //increment
    buf2 = (int)floor((double)buf1/pow(n,dim));
    //-1.0 ~ 1.0
    centroid[i][dim] = len*(2.0*(double)buf2+1.0-(double)marge)-1.0;
    buf1++;
    //count
    if(buf2==n-1){
      buf3++;
    }
    //reset
    if(buf3==(int)pow(n,dim)){
      buf1=0;
      buf2=0;
      buf3=0;
    }
  }
  grid(centroid, len, nmax, n, dim-1, marge);
  return 1;
}
//get centroid of grid
int GMixture::layoutGrid(double** centroid, int dim, int num,
			 int zero, int marge)
{
  //number of each dimention
  int n = (int)floor(pow(num,1.0/(double)dim));
  //max number of lattice centroids
  int nmax = (int)pow(n,dim);
  //harf length of a lattice
  double len = 1.0/(double)(n-marge);
  //recursive call
  grid(centroid, len, nmax, n, dim-1, marge);
  //centroids except lattice centroids randomized
  for(int i=nmax; i<num; i++){
    for(int j=0; j<dim; j++){
      centroid[i][j] = (double)rand()/(double)RAND_MAX*2.0-1.0;
    }
  }
  //zero centroid can't be allowed
  if(!zero){
    int c;
    if(!((c=(nmax-1))%2)){
      for(int i=0; i<dim; i++){
	centroid[c/2][i]
	  = (double)rand()/(double)RAND_MAX*2.0-1.0;
      }
    }
  }
  return 1;
}


//change parameters
int GMixture::setParam(int param, double value)
{
//   switch(param){
//   case GUpdateStep:
//     upstep = (int)value;
//     break;
//   case GLearning:
//     learning = value;
//     break;
//   case GVariance:
//     variance = value;
//     break;
//   default:
//     Error("In GMixture.cpp at setParam : argument error...\n"
// 	  "\tparam = %d, value = %f\n", param, value);
//     return 1;
//  }
  return 0;
}

//get parameters
double GMixture::getParam(int param)
{
//   switch(param){
//   case GLearning:
//     return learning;
//   case GVariance:
//     return variance;
//   case GUpdateStep:
//     return upstep;
//   default:
//     Error("In GMixture.cpp at getParam : argument error...\n"
// 	  "\tparam = %d\n", param);
//     return 1;
//   }
  return 0;
}

