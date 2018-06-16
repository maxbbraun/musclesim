#include <cmath>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "util.hpp"


// random variable functions #######################################
#define DOUBLE_BIT 31
/* random value and some matrix initialization */
int init_random()
{
  unsigned int s;
  time_t t;
  t=time(&t);  s=abs(t); srandom(s);
  return s;
}

int init_random_seed(unsigned int s)
{
  srandom(s);
  return s;
}
#undef DOUBLE_BIT
int randi()
{
  int rnd;
  rnd=(int)random();
  return rnd;
}


int randin(int n)
{
  double r=n*rand01();
  return floor(r);
}


double rand01()
{
  double rnd;
  rnd=((double)random())/((double)RAND_MAX);
  return rnd;
}

double rand11()
{
  return 2.0*rand01()-1.0;
}

double randn()
{
  double fac,rsq,v1,v2;
  static double NDR=0.0;
  static int    NDR_FLAG=0;

  if(NDR_FLAG==0)
    {
      do{
	v1=rand11(); v2=rand11(); rsq=v1*v1+v2*v2;
      } while(rsq >=1.0 || rsq==0.0);
      fac=sqrt(-2.0*log(rsq)/rsq);
      NDR = v1*fac;
      NDR_FLAG =1;
      return v2*fac;
    }
  else
    {
      NDR_FLAG=0;
      return NDR;
    }
}

// ##################################################################
// multinomial_distribution
// ##################################################################
int multinomial_distribution(double *p, int n)
{
  int i;
  double s, r=rand01();
  s=*p++;
  for(i=0;i<n;i++)
    {
      if(s > r) break;
      s+=*p++;
    }
  return i;
}


int multinomial_distribution_exp(double *p, int n)
{
  double maxp=-300;
  double *pp;   pp=new double [n];
  int idx;


  for(int i=0;i<n;i++) { if(maxp< p[i]) maxp=p[i]; }
  for(int i=0;i<n;i++) 
    {
      if(-20<(p[i]-maxp)) pp[i]=exp(p[i]-maxp); 
      else pp[i]=0.0;
    }

  idx=multinomial_distribution_norm(pp, n);
  delete [] pp;
  return idx;
}

int multinomial_distribution_norm(double *p, int n)
{
  double sump=0.0;
  double *pp; pp=new double [n];
  int idx;

  for(int i=0;i<n;i++) sump+=p[i];
  for(int i=0;i<n;i++) pp[i]=p[i]/sump;

  idx=multinomial_distribution(pp, n);
  delete [] pp;
  return idx;
}


double multinomial_norm_pdf(int idx, double *p, int n)
{
  double sum=0.0;

  for(int i=0; i<n; i++)  sum+=p[i];
  return p[idx]/sum;
}

double multinomial_exp_pdf(int idx, double *p, int n)
{
  double maxp=-300;
  double *pp;   pp=new double [n];
  double prob;

  for(int i=0;i<n;i++) { if(maxp< p[i]) maxp=p[i]; }
  for(int i=0;i<n;i++) 
    {
      if(-20<(p[i]-maxp)) pp[i]=exp((p[i]-maxp));
      else pp[i]=0;
    }
  prob=multinomial_norm_pdf(idx, pp, n);

  delete [] pp;
  return prob;
}

// ##################################################################
// Gaussian_distribution
// ##################################################################
// log of Gaussian 
double logGaussian(double *x, double *mu, double beta, int dim)
{
  double Z, L=0, l;
  double *px, *pm;
  px=x; pm=mu;
  
  Z=log(pow(beta/(2*PI), dim/2));

  for(int i=0; i<dim; i++)
    {
      l=*px++ - *pm++;
      L+=l*l;
    }
  return -0.5*beta*L+Z;
}

// Probability density functions
double gaussian_pdf(double x, double m, double beta)
{
  double dx, l;
  dx=x-m; l=0.5*beta*dx*dx;
  if(l>100) return 0;
  else  return sqrt(beta/(2*PI))*exp(-l);
}

// some useful functions ############################################
double threshold(double x)
{
  if(x>0) return x;
  else    return 0.0;
}
double sign(double x)
{
  if(x>0) return 1.0;
  else    return 0.0;
}
double sigmoid(double x)
{
  double ex;
  ex=exp(x);
  return ex/(1+ex);
}

double limit11(double x)
{
  if(x>1.0) return 1.0;
  if(x<-1.0) return -1.0;
  return x;
}

double limit11ignore(double x)
{
  if(x>1.0) return 0.0;
  if(x<-1.0) return -0.0;
  return x;
}

// for quick sort
int compare_double(const double *a, const double *b)
{
  double r;
  r=*a - *b;
  if(r<0.0) return -1;
  if(r>0.0) return 1;
  return 0;
}




// ******************************************************************
ostream &setup(ostream &stream)
{
  // stream.width(10);
  stream.width(4);
  stream.precision(3);

  return stream;
}


// ******************************************************************
