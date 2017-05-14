#ifndef _MEDIUM_H_
#define _MEDIUM_H_

#include "vec.h"

class Medium {
public:
	explicit Medium(const Color &sigS_ = { 0.0, 0.0, 0.0 },
		const Color &sigA_ = { 0.0, 0.0, 0.0 },
		double eta_ = 1.0)
		: sigS(sigS_)
		, sigA(sigA_)
		, eta(eta_) {
	}

	Color sigS, sigA;
	double eta;
};

#endif  // _MEDIUM_H_
