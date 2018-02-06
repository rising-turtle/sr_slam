#ifndef GLOBAL_FUNC_H
#define GLOBAL_FUNC_H

#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif
#include "NodeHandle.h"

extern bool g_has_time_asy; 
extern unsigned long g_dwHighDateTime;
extern unsigned long g_dwLowDateTime;
extern double getRosTime();
extern void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);
extern double round(double val);

#endif