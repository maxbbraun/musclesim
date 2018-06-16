#include <time.h>
#include <sys/time.h>

#define PI 3.141592

// random variable generator
int init_random();
int init_random_seed(unsigned int s);
int randi();
int randin(int n);
double rand01();
double rand11();
double randn();

// for multinomial distribution
int multinomial_distribution(double *p, int n);
int multinomial_distribution_exp(double *p, int n);
int multinomial_distribution_norm(double *p, int n);

// useful functions
double threshold(double x);
double sign(double x);
double sigmoid(double x);
double logGaussian(double *x, double *mu, double beta, int dim);
double limit11(double x);
double limit11ignore(double x);


// for quick sort
// usage
// d: array storing data
// n: number of data
//   qsort(d, n, sizeof(double), 
//        (int (*)(const void*, const void*))compare_double);
int compare_double(const double *a, const double *b);



// probability density function
double gaussian_pdf(double x, double m, double beta);
double multinomial_norm_pdf(int idx, double *p, int n);
double multinomial_exp_pdf(int idx, double *p, int n);

ostream &setup(ostream &stream);

#define coutf cout << setup 
// #define coutf cout


class simpleClock
{
protected:
  double bias;
  struct timeval tv;
public:
  simpleClock()
  {
    // gettimeofday(&tv, NULL);
    // bias=(double)tv.tv_sec + (double)tv.tv_usec*1e-6;
    bias=tic();
  };
  ~simpleClock(){};

  double toc()
  {
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec*1e-6 -bias;
  }
  double tic()
  {
    gettimeofday(&tv, NULL);
    bias=(double)tv.tv_sec + (double)tv.tv_usec*1e-6;
    return bias;
  }
  int waitfor(double sec)
  {
    while(toc()<sec){} tic();
    return 1;
  }
};
