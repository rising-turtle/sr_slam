/*
 * cpp_utils.h
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_

#include <string>
#include <vector>
#include <iostream>
#include <time.h>
#include <math.h>
#include <cmath>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>


typedef uint64_t TTimeStamp;

using namespace std;


TTimeStamp  getCurrentTime( );

TTimeStamp  time_tToTimestamp(const time_t &t );

double timeDifference( const TTimeStamp &t1, const TTimeStamp &t2 );



#endif /* TIMESTAMP_H_ */
