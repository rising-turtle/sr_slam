
#include  "boost/random/mersenne_twister.hpp"
#include "boost/random/normal_distribution.hpp"
#include "boost/random/variate_generator.hpp"

#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

class CNormal{
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
	double normal(double mean = 1, double sigma = 2)
	{
		return CNormal::normal(mean,sigma);
	}
};

double generateRandom()
{
  boost::normal_distribution<double> dist(5,1);
  static boost::mt19937 engine;
  boost::variate_generator<boost::mt19937&,
   boost::normal_distribution<double> > gen(engine, dist);
  double r = gen();
  return r;
}

int main()
{

int i =0;
/*while(i<10)
{
i++;
std::vector<double> v(3);
std::generate (v.begin(), v.end(), gen);
std::cout << v[0] <<" "<< v[1]<<" " <<v[2] <<std::endl;
}*/
CSNormal gN;
while(i++<10000){
	// cout<<generateRandom()<<endl;
	cout<<gN.normal()<<endl;
}

ofstream tmp("tmp2");
i = 0;
CSNormal gN2;
while(i++<10000)
{
  tmp<<gN.normal()<<endl;
}

  return 0;
}  
