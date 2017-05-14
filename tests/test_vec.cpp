#include <ctime>
#include <tuple>

#include <gtest/gtest.h>

#include "Vec.h"
#include "random.h"

static const int trials = 100;

using VecPair = std::tuple<Vec, Vec>;

// ----------------------------------------------------------------------------
// Fixiation classes
// ----------------------------------------------------------------------------

class VecTest : public ::testing::Test {
protected:
	Random rand;

protected:
	VecTest()
		: rand((unsigned long)time(0)) {
	}

	virtual ~VecTest() {}
};

class VecUnaryTest : public VecTest, public ::testing::WithParamInterface<Vec> {
protected:
	Vec v1;

protected:
	VecUnaryTest() {}
	virtual ~VecUnaryTest() {}

	void SetUp() {
		v1 = GetParam();
	}
};

class VecPairwiseTest : public VecTest, public ::testing::WithParamInterface<VecPair> {
protected:
	Vec v1, v2;

protected:
	VecPairwiseTest() {}
	virtual ~VecPairwiseTest() {}

	void SetUp() {
		v1 = std::get<0>(GetParam());
		v2 = std::get<1>(GetParam());
	}
};

// ----------------------------------------------------------------------------
// Test definitions
// ----------------------------------------------------------------------------
TEST_F(VecTest, VecDefaultInstance) {
	Vec v;
	ASSERT_EQ(v.x, 0.0);
	ASSERT_EQ(v.y, 0.0);
	ASSERT_EQ(v.z, 0.0);
}

TEST_F(VecTest, VecSingletInstance) {
	double x = rand.uniform(-1.0, 1.0);
	Vec v(x);
	ASSERT_EQ(v.x, x);
	ASSERT_EQ(v.y, x);
	ASSERT_EQ(v.z, x);
}

TEST_F(VecTest, TestInstance) {
	double x = rand.uniform(-1.0, 1.0);
	double y = rand.uniform(-1.0, 1.0);
	double z = rand.uniform(-1.0, 1.0);
	Vec v(x, y, z);
	ASSERT_EQ(v.x, x);
	ASSERT_EQ(v.y, y);
	ASSERT_EQ(v.z, z);
}

TEST_P(VecPairwiseTest, TestAdd) {
	Vec v3 = v1 + v2;
	ASSERT_EQ(v3.x, v1.x + v2.x);
	ASSERT_EQ(v3.y, v1.y + v2.y);
	ASSERT_EQ(v3.z, v1.z + v2.z);
}

TEST_P(VecPairwiseTest, kakezan) {
	Vec v3 = v1*v2;
	ASSERT_EQ(v3.x, v1.x * v2.x);
	ASSERT_EQ(v3.y, v1.y * v2.y);
	ASSERT_EQ(v3.z, v1.z * v2.z);
	
}

TEST_P(VecUnaryTest, MultipleScalar) {
	const double s = rand.uniform(-1.0, 1.0);
	const Vec v2 = v1 * s;
	ASSERT_EQ(v2.x, v1.x * s);
	ASSERT_EQ(v2.y, v1.y * s);
	ASSERT_EQ(v2.z, v1.z * s);

	const Vec v3 = s * v1;
	ASSERT_EQ(v3.x, v1.x * s);
	ASSERT_EQ(v3.y, v1.y * s);
	ASSERT_EQ(v3.z, v1.z * s);
}

TEST_P(VecPairwiseTest, warizan) {
	Vec v3;
	if (v2.x == 0 || v2.y == 0 || v2.z == 0) {
		ASSERT_DEATH(v1 / v2, "");
	}
	else {
		v3 = v1 / v2;
		ASSERT_EQ(v3.x, v1.x / v2.x);
		ASSERT_EQ(v3.y, v1.y / v2.y);
		ASSERT_EQ(v3.z, v1.z / v2.z);
	}
}

TEST_P(VecPairwiseTest, hikizan) {
	Vec v3 = v1 - v2;
	ASSERT_EQ(v3.x, v1.x - v2.x);
	ASSERT_EQ(v3.y, v1.y - v2.y);
	ASSERT_EQ(v3.z, v1.z - v2.z);
}

TEST_P(VecUnaryTest, ExpTest) {
	Vec v2 = Vec::exp(v1);
	ASSERT_EQ(v2.x, std::exp(v1.x));
	ASSERT_EQ(v2.y, std::exp(v1.y));
	ASSERT_EQ(v2.z, std::exp(v1.z));
}

TEST_P(VecPairwiseTest, Dot) {
	double a = Dot(v1,v2);
	ASSERT_EQ(a, v1.x*v2.x+ v1.y*v2.y + v1.z*v2.z);
}

TEST_P(VecPairwiseTest, equalequal) {
	ASSERT_EQ(v1 == v2, (v1.x==v2.x)&&(v1.y== v2.y)&&(v1.z == v2.z));
}

TEST_F(VecTest, TestAddRandom) {
	for (int i = 0; i < trials; i++) {
		Vec v1(rand.uniform(-1.0, 1.0), rand.uniform(-1.0, 1.0), rand.uniform(-1.0, 1.0));
		Vec v2(rand.uniform(-1.0, 1.0), rand.uniform(-1.0, 1.0), rand.uniform(-1.0, 1.0));
		Vec v3 = v1 + v2;
		ASSERT_EQ(v3.x, v1.x + v2.x);
		ASSERT_EQ(v3.y, v1.y + v2.y);
		ASSERT_EQ(v3.z, v1.z + v2.z);
	}
}

TEST_P(VecUnaryTest, TestNorm) {
	double nrm = std::sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
	ASSERT_EQ(v1.Length(), nrm);
}

TEST_P(VecUnaryTest, TestNormSq) {
	double nrm_sq = v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
	ASSERT_EQ(v1.LengthSquared(), nrm_sq);
}

TEST_P(VecUnaryTest,minus) {
	Vec v2 = -v1;
	ASSERT_EQ(v2.x, -v1.x);
	ASSERT_EQ(v2.y, -v1.y);
	ASSERT_EQ(v2.z, -v1.z);
}

TEST(TestVec, TestdoubleDivFails) {
	Vec v1(1.0, 2.0, 3.0);
	const double div = 0.0;
	ASSERT_DEATH(v1 / div, "");
}

std::vector<Vec> vectors = {
	Vec(0.0, 0.0, 0.0),
	Vec(0.0, 1.0, 2.0),
	Vec(-2.0, -1.0, 0.0),
	Vec(3.14, 1.59, 2.65),
	Vec(1.0e8, 1.0e8, 1.0e8),
	Vec(1.0e-8, 1.0e-8, 1.0e-8)
};

INSTANTIATE_TEST_CASE_P(, VecUnaryTest,
	::testing::ValuesIn(vectors));

INSTANTIATE_TEST_CASE_P(, VecPairwiseTest,
	::testing::Combine(::testing::ValuesIn(vectors),
		::testing::ValuesIn(vectors)));

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
