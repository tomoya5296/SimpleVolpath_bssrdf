#pragma once
#ifdef _MSC_VER
#pragma once
#endif

#ifndef _PARALLEL_H_
#define _PARALLEL_H_

#include <atomic>
#include <functional>

#include "common.h"

enum class ParallelSchedule {
	Static = 0x01,
	Dynamic = 0x02
};

void parallel_for(int start, int end, const std::function<void(int)>& func,
	ParallelSchedule schedule = ParallelSchedule::Dynamic);

int numSystemThreads();
int getThreadID();

#endif // _PARALLEL_H_
