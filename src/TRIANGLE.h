
#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

#include <cmath>
#include "ray.h"
#include "material.h"
#include "vec.h"
#include "common.h"
#include"intersection.h"


struct TRIANGLE {



	TRIANGLE(Vec v0, Vec v1, Vec v2, Vec n0, Vec n1, Vec n2,Material &mat_) 
		: mat(mat_) {
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
		n[0] = n0;
		n[1] = n1;
		n[2] = n2;
	}




	TRIANGLE(Vec v_[3], Vec n_[3], Material &mat_)
		: mat(mat_) {
	
		v[0] = v_[0];
		v[1] = v_[1];
		v[2] = v_[2];
		n[0] = n_[0];
		n[1] = n_[1];
		n[2] = n_[2];
		
	}



	TRIANGLE()
		 {

		}




	bool intersect(const Ray &ray, Hitpoint *hitpoint) {
		//Tomas Moller method

		
		Vec edge2 = v[1] - v[0];
		Vec	edge1 = v[2] - v[0];
		


		Vec invRay = -1*ray.dir;

		double denominator =det(edge1, edge2, invRay);
		if (denominator < 0.0) {
			Vec temp = edge1;
			edge1 = edge2;
			edge2 = temp;
		}
		else if (denominator == 0.0) {
			return false;
		}
		denominator = det(edge1, edge2, invRay);


		//The denominator of Cramer's rule

		//p is point in this triangle;
		// p = v0 + u*edge1 + v*edge2;
		// p = ray.org + t*ray.dir;
		//Formula deformation;
		//(edge1*u)+(edge2*v)-(ray.dir*t)=ray.org-v0;
		//-(ray.dir*t)+(edge1*u)+(edge2*v)=ray.org-v0;
		//solution is Cramer's rule

		//Is a ray parallel for the triangle
	
		if (denominator <= 0.0) {
			return false;
		}

		Vec d = ray.org - v[0];
		double u = det(d, edge2, invRay) / denominator;

		if ((u >= 0.0) && (u <= 1.0)) {
			double v = det(edge1, d, invRay) / denominator;
			if ((v >= 0.0) && (u + v <= 1.0)) {
				double t1 = det(edge1, edge2, d) / denominator;
				if (t1 < 0.0) {
					return false;
				}
				hitpoint->tri = this;
				hitpoint->distance = t1;
				hitpoint->position = ray.org + hitpoint->distance*ray.dir;
				hitpoint->normal =normal;
				
				return true;
			}
		}
		return false;
	}


	inline double det(const Vec &vecA, const Vec &vecB, const Vec &vecC) {
		return ((vecA.x * vecB.y * vecC.z)
			+ (vecA.y * vecB.z * vecC.x)
			+ (vecA.z * vecB.x * vecC.y)
			- (vecA.x * vecB.z * vecC.y)
			- (vecA.y * vecB.x * vecC.z)
			- (vecA.z * vecB.y * vecC.x));
	}


	Vec v[3];
	Vec n[3];
	Vec normal;
	Material mat;
	float bbox[2][3];
	int obj_id;
};



#endif  // _TRIANGLE_H_
