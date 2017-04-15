/*
 * Jun 3, 2015, David Z
 * compute the mean and sigma 
 *
 * */


#ifndef MU_SIGMA_HPP
#define MU_SIGMA_HPP

#define SQ(x) ((x)*(x))

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
    total += SQ(in[i]-mu);
  }
  sigma = sqrt(total/(T)N);
  return true;
}

#endif
