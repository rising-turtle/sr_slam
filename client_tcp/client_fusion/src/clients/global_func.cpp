#include "global_func.h"

#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#define _WIN32_WINNT 0x0501

#include "NodeHandle.h"
#include "NormalSleep.hpp"

#include <cmath>
#include <ctime>
#include <iomanip>
#include <stdexcept>
#include <limits.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <windows.h>

bool g_has_time_asy = false;
DWORD g_dwHighDateTime = 0;
DWORD g_dwLowDateTime = 0;

double round(double val)
{    
	return floor(val + 0.5);
}


void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
	int64_t nsec_part = nsec;
	int64_t sec_part = sec;

	while (nsec_part >= 1000000000L)
	{
		nsec_part -= 1000000000L;
		++sec_part;
	}
	while (nsec_part < 0)
	{
		nsec_part += 1000000000L;
		--sec_part;
	}

	if (sec_part < 0 || sec_part > INT_MAX)
		throw std::runtime_error("Time is out of dual 32-bit range");

	sec = sec_part;
	nsec = nsec_part;
}
double getRosTime() 
{
	uint32_t sec, nsec;
#ifndef WIN32
#if HAS_CLOCK_GETTIME
	timespec start;
	clock_gettime(CLOCK_REALTIME, &start);
	sec  = start.tv_sec;
	nsec = start.tv_nsec;
#else
	struct timeval timeofday;
	gettimeofday(&timeofday,NULL);
	sec  = timeofday.tv_sec;
	nsec = timeofday.tv_usec * 1000;
#endif
#else
	static LARGE_INTEGER cpu_freq, init_cpu_time;
	static uint32_t start_sec = 0;
	static uint32_t start_nsec = 0;
	if ( ( start_sec == 0 ) && ( start_nsec == 0 ) )
	{
		QueryPerformanceFrequency(&cpu_freq);
		if (cpu_freq.QuadPart == 0) {
			printf("NoHighPerformanceTimersException()\n");
		}
		QueryPerformanceCounter(&init_cpu_time);

		if(g_has_time_asy)
		{	
			start_sec = g_dwHighDateTime;
			start_nsec = g_dwLowDateTime;
		}else
		{
			// compute an offset from the Epoch using the lower-performance timer API
			FILETIME ft;
			GetSystemTimeAsFileTime(&ft);
			LARGE_INTEGER start_li;
			start_li.LowPart = ft.dwLowDateTime;
			start_li.HighPart = ft.dwHighDateTime;
			// why did they choose 1601 as the time zero, instead of 1970?
			// there were no outstanding hard rock bands in 1601.
	#ifdef _MSC_VER
			start_li.QuadPart -= 116444736000000000Ui64;
	#else
			start_li.QuadPart -= 116444736000000000ULL;
	#endif
			start_sec = (uint32_t)(start_li.QuadPart / 10000000); // 100-ns units. odd.
			start_nsec = (start_li.LowPart % 10000000) * 100;
		}
	}
	LARGE_INTEGER cur_time;
	QueryPerformanceCounter(&cur_time);
	LARGE_INTEGER delta_cpu_time;
	delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
	// todo: how to handle cpu clock drift. not sure it's a big deal for us.
	// also, think about clock wraparound. seems extremely unlikey, but possible
	double d_delta_cpu_time = delta_cpu_time.QuadPart / (double) cpu_freq.QuadPart;
	uint32_t delta_sec = (uint32_t) floor(d_delta_cpu_time);
	uint32_t delta_nsec = (uint32_t) round((d_delta_cpu_time-delta_sec) * 1e9);

	int64_t sec_sum  = (int64_t)start_sec  + (int64_t)delta_sec;
	int64_t nsec_sum = (int64_t)start_nsec + (int64_t)delta_nsec;

	// Throws an exception if we go out of 32-bit range
	normalizeSecNSecUnsigned(sec_sum, nsec_sum);

	sec = sec_sum;
	nsec = nsec_sum;
#endif

	return (double)sec + 1e-9*(double)nsec;
}

