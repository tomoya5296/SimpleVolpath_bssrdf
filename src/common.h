#ifndef _COMMON_H_
#define _COMMON_H_

#include <cmath>
#include <string>

static const double PI = 4.0 * std::atan(1.0);
static const double INF = 1.0e20;
static const double EPS = 1.0e-8;

// -----------------------------------------------------------------------------
// Assertion with message
// -----------------------------------------------------------------------------

#ifndef __FUNCTION_NAME__
#if defined(_WIN32) || defined(__WIN32__)
#define __FUNCTION_NAME__ __FUNCTION__
#else
#define __FUNCTION_NAME__ __func__
#endif
#endif

#undef NDEBUG
#ifndef NDEBUG
#define Assertion(PREDICATE, ...) \
do { \
if (!(PREDICATE)) { \
std::cerr << "Asssertion \"" \
<< #PREDICATE << "\" failed in " << __FILE__ \
<< " line " << __LINE__ \
<< " in function \"" << (__FUNCTION_NAME__) << "\"" \
<< " : "; \
fprintf(stderr, __VA_ARGS__); \
std::cerr << std::endl; \
std::abort(); \
} \
} while (false)
#else  // NDEBUG
#define Assertion(PREDICATE, ...) do {} while (false)
#endif // NDEBUG

#endif  // _COMMON_H_
