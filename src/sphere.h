#ifndef _SPHERE_H_
#define _SPHERE_H_

#include <cmath>

#include "common.h"
#include "vec.h"
#include "ray.h"
#include "material.h"

struct Sphere {
	Sphere(double radius_, const Vec &center_, const Material &mat_)
		: radius(radius_)
		, center(center_)
		, mat(mat_) {
	}

	bool intersect(const Ray &ray, double *t) {
		Vec o2c = center - ray.org;
		const double b = Dot(o2c, ray.dir);
		const double det = b * b - Dot(o2c, o2c) + radius * radius;
		if (det >= 0.0) {
			const double sqdet = std::sqrt(det);
			const double t1 = b - sqdet;
			const double t2 = b + sqdet;
			if (t1 > EPS && t1 < *t) {
				*t = t1;
				return true;
			}
			else if (t2 > EPS && t2 < *t) {
				*t = t2;
				return true;
			}
		}
		return false;
	}

	double radius;
	Vec center;
	Material mat;
};

#endif  // _SPHERE_H_
