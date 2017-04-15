
/*
 *  Sep. 23, 2015, David Z
 *  
 *  compute the mean and sigma of an array of data 
 *
 * */

#ifndef MEAN_SIGMA_H
#define MEAN_SIGMA_H

#include <cmath>

template<typename T>
bool compute_mu_sigma(T* in, int N, T& mu, T& sigma)
{
  if(N<=0)
  {
    mu = 0; 
    sigma = 0;
    return false;
  }

  // compute mu
  T total = 0; 
  for(int i=0; i<N; i++)
  {
    total += in[i];
  }
  mu = total/(T)(N); 

  // compute sigma
  total = 0;
  for(int i=0; i<N; i++)
  {
    total += (in[i]-mu)*(in[i]-mu);
  }
  sigma = sqrt(total/(T)N);
  return true;
}


#endif

