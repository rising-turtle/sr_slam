/*
 * timestamp.cpp
 *
 *  Created on: Dec 18, 2012
 *      Author: liu
 */

#include "timestamp.h"

TTimeStamp  getCurrentTime( )
{
    timespec  tim;
    clock_gettime(CLOCK_REALTIME, &tim);
	return time_tToTimestamp( tim.tv_sec ) + tim.tv_nsec/100;
}

TTimeStamp  time_tToTimestamp(const time_t &t )
{
	//10000000 is the space to save tv_nsec/100
    return (((uint64_t)t) * (uint64_t)10000000) + ((uint64_t)116444736*1000000000);
}


/*---------------------------------------------------------------
					timeDifference
  ---------------------------------------------------------------*/
double timeDifference( const TTimeStamp &t1, const TTimeStamp &t2 )
{
	return ((double)((int64_t)(t2-t1)))/10000000.0;
}




