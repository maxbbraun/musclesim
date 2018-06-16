// discrete time and continuous state space system
class mcmc
{
 private:

 protected:
  int N;
  double *s, *ts, *dF;
  double DT;

 public:
  mcmc(int dim);
  virtual ~mcmc();

  void printState();
  void printState(int t);
  double *getState() {return s;};
  int setState(double *S);
  int dimensionOfState(){ return N; };

  double *rejectSampling(double a, double *S);
  double *biasedSampling(double a, double *S);

  virtual double Fval(double a, double *x)=0;
  virtual double *Fgrad(double a, double *x)=0;
};

// Gaussian mixture with variable width
class gmpdf : public mcmc
{
private:
  double *mu;
  int num;

public:
  gmpdf(int dim, int num, double *Mu);
  gmpdf(int dim, int Num);
  ~gmpdf();

  double Fval(double a, double *x);
  double *Fgrad(double a, double *x);

  double gaussianDensity(double *x, double *m, double a);
  double *gaussianGradient(double *x, double *m, double a);
};














