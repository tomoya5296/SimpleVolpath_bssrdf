#ifndef _RAY_H
#define _RAY_H

#include"vec.h"

struct  Ray{
	Ray(const Vec org_, const Vec &dir_)
		:org(org_)
		, dir(dir_) {

	}


	Vec org, dir;

};


#endif //_RAY_H