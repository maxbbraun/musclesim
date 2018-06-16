#include <cmath>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "mcmc.hpp"
#include "util.hpp"


mcmc::mcmc(int dim)
{
  N=dim;
  s=new double [dim];
  ts=new double [dim];
  dF=new double [dim];
  DT=0.1;
  for(int i=0; i<dim; i++) s[i]=0.01*randn()-0.5;
}


mcmc::~mcmc()
{
  delete [] s;
  delete [] ts;
  delete [] dF;
}


void mcmc::printState()
{
  // cout << "S(" << N << ")";
  for(int i=0; i<N; i++)   cout << "\t" << s[i];
  // cout << endl;
}

void mcmc::printState(int t)
{
  cout << t;
  for(int i=0; i<N; i++)   cout << "\t" << s[i];
  cout << endl;
}

int mcmc::setState(double *S)
{
  int i;

  for(i=0; i<N; i++) s[i]=S[i];
  return i;
}

double *mcmc::rejectSampling(double a, double *S)
{
  for(int i=0; i<N; i++) ts[i]=s[i]+0.1*randn();

  if(Fval(a, ts)/Fval(a, s)>rand01())
    for(int i=0; i<N; i++) s[i]=ts[i];
  return s;
}


double *mcmc::biasedSampling(double a, double *S)
{
  double *df=Fgrad(a, s);
  for(int i=0; i<N; i++) s[i]+=DT*(df[i]*a+0.1*randn());
  return s;
}

// Gaussian mixture with variable width
gmpdf::gmpdf(int dim, int Num, double *Mu): mcmc(dim)
{
  num=Num;
  mu = new double [dim*num];
  for(int i=0; i<dim*num; i++)   mu[i]=Mu[i];
}


gmpdf::gmpdf(int dim, int Num): mcmc(dim)
{
  num=Num;
  mu = new double [dim*num];
  double norm, val;

  for(int j=0; j<num; j++)
    {
      norm=0.0;
      for(int i=0; i<dim; i++)
	{
	  val=(double)randin(3)-1.0;
	  mu[dim*j+i]=val;
	  norm+=val;
	}
      if(norm!=0.0)
	for(int i=0; i<dim; i++)
	  mu[dim*j+i]/=norm;
    }
}




gmpdf::~gmpdf()
{
  delete [] mu;
}

double gmpdf::gaussianDensity(double *x, double *m, double a)
{
  double d=0.0;
  for(int i=0; i<N; i++) d+=(x[i]-m[i])*(x[i]-m[i]);
  return pow(a/(2*3.14),N/2)*exp(-0.5*a*d);
}

double *gmpdf::gaussianGradient(double *x, double *m, double a)
{
  static double *d = new double [N];
  double w=gaussianDensity(x, m, a);

  for(int i=0; i<N; i++) d[i]=-a*(x[i]-m[i])*w;

  return d;
}



double gmpdf::Fval(double a, double *x)
{
  double u=0;
  for(int i=0; i<num; i++)
    {
      u+=gaussianDensity(x, mu+N*i, a);
    }
  return u;
}


double *gmpdf::Fgrad(double a, double *x)
{
  double *d;
  for(int j=0; j<N; j++) dF[j]=0.0;
  for(int i=0; i<num; i++)
    {
      d=gaussianGradient(x, mu+N*i, a);
      for(int j=0; j<N; j++)
	dF[j]+=d[j];
    }
  return dF;
}


// test program
//int main(int argc, char **argv)
//{
//  double *x;
//  double mu[]={1.0, 0.5, -1.0, -0.5, 0.5, -1.0, -0.5, 1.0};
//  gmpdf *yuragi = new gmpdf(2, 4, mu);
//  double a;
//
//  int T; T=atoi(argv[1]);
//  int seed; seed=atoi(argv[2]); init_random_seed(seed);
//
//  double ix[2]; ix[0]=0.2*randn(); ix[1]=0.2*randn();
//  yuragi->setState(ix);
//
//  for(int t=0; t<T; t++)
//    {
//      yuragi->printState();
//      cout << endl;
//      a=1+1*sin(3.14/360*(double)t);
//      x=yuragi->biasedSampling(3*a, x);
//      // x=yuragi->rejectSampling(10*a, x);
//    }
//}
