#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "vec.h"
#include "medium.h"

static const double airIOR = 1.0;
static const double objectIOR = 1.5;

enum RefType : int {
	DIFFUSE,
	SPECULAR,
	REFRACTION,
	TRANSLUCENT,
};

class Material {
public:
	Material(const Color &Le_, const Color &ref_, RefType type_,
		const Medium &medium_ = Medium{})
		: Le(Le_)
		, ref(ref_)
		, type(type_)
		, medium(medium_) {
	}


	Material()
	 {
	}

	bool isLight() const {
		return !Le.isZero();
	}

	bool isTranslucent() const {
		return !medium.sigS.isZero() && !medium.sigA.isZero();
	}

	Color Le, ref;
	RefType type;
	Medium medium;
};

#endif  // _MATERIAL_H_
