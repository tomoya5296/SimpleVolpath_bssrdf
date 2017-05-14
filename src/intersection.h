#ifndef _INTERSECTION_H_
#define _INTERSECTION_H_
#include<algorithm>
#include "vec.h"
#include "common.h"
#include "ray.h"

struct TRIANGLE;


	struct Hitpoint {
		double distance;
		TRIANGLE *tri;
		Vec normal;
		Vec orienting_normal;
		Vec position;
		Hitpoint() : distance(INF), normal(), orienting_normal(), position(){}
	};

	struct Intersection {
		Hitpoint hitpoint;
		int obj_id;
		int tri_num;
		Intersection():obj_id(-1),tri_num(-1)  {}
		Material Mat;
	};

	





#endif