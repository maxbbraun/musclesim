#ifndef __GMIXTURE_H__
#define __GMIXTURE_H__

class GMixture{
 public:
  //constructor
  //  int dimention     : dimention of centroid
  //  int number        : number of centroid
  GMixture(int dimention, int number);
  //destructor
  ~GMixture();

  //this method sets arbitrary centroid
  //  double** centroids : array of arbitrary centroids
  int setCentroid(double** centroids);

  //this method returns pointer of centroids
  //  return double** : array of centroids
  double** getCentroid(void);

  //this method generates (initializes) centroids according to arguments,
  //and return pointer of centroids
  //  int zero          : if 1, zero centroid can be arrowed
  //                    : if 0, zero centroid can't be arrowed
  //  int marge         : if 1, centroid is set at marge
  //                    : if 0, centroid isn't set at marge
  double** getGridCentroid(int zero=0, int marge=0);

  //update centroid
  double** update(double* sample);
  //update centroid from weighted data
  double** update(double weight, double* sample);

  //change parameters
  int setParam(int param, double value);
  //get parameters
  double getParam(int param);

  //use k-means method
  int kmeansOn(bool on);

  bool centroidUpdated(void);

 protected:

 private:
  //dimention of data
  int dimention;
  //number of attractors
  int number;
  //learning coefficient
  double learning;
  //variance of gaussian
  double variance;
  //update step
  int update_step;
  //update count
  int count;
  //centroid
  double** centroid;
  //expected value, <weight> of each cluster
  double* ex_w;
  //expected value, <weight*data> of each cluster
  double** ex_wd;

  //wheather use kmeans method or not
  bool kmeans_method;

  bool updated;

  unsigned int setRandSeed(unsigned int seed);

  int randomCentroid(void);

  //get centroid of grid
  int layoutGrid(double** centroid, int dim, int num, int zero, int marge);
  //recursive call for get centroid
  int grid(double** centroid, double len,
	   int nmax, int n, int dim, int marge);
  //calclation gaussian distribution
  double gauss(double* sample, int index);
};

#endif
