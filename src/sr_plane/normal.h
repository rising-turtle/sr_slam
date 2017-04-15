/*
 *  Sep 14 2015, David Z
 *  
 *  normal.h define a class to generate normal distribution 
 *
 * */

#ifndef NORMAL_H
#define NORMAL_H

#include  "boost/random/mersenne_twister.hpp"
#include "boost/random/normal_distribution.hpp"
#include "boost/random/variate_generator.hpp"

#include <iostream>
#include <vector>
#include <fstream>



using namespace std;

class CNormal
{
  public:
    double normal(double mean=0, double sigma =1){
      boost::normal_distribution<double> dist(mean,sigma);
      boost::variate_generator<boost::mt19937&,\
        boost::normal_distribution<double> > gen(eng,dist);
      return gen();
    }
    boost::mt19937 eng;
};
class CSNormal : public CNormal
{
  public:
    CSNormal(double mean = 0, double sigma = 2): mean_(mean), sigma_(sigma){}
    double normal()
    {
      return CNormal::normal(mean_,sigma_);
    }

    void setSigma(double sigma){sigma_ = sigma;}
    double mean_; 
    double sigma_;
};

#endif
