// Minimal assertion helpers, so `ctest` works with no external framework.
#ifndef MTL_TEST_UTIL_HPP
#define MTL_TEST_UTIL_HPP

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace mtl::test {

inline int g_failures = 0;

inline void check(bool cond, const std::string& what, const char* file, int line) {
    if (cond) return;
    std::printf("  FAIL %s  (%s:%d)\n", what.c_str(), file, line);
    ++g_failures;
}

inline void checkNear(double a, double b, double tol, const std::string& what, const char* file,
                      int line) {
    if (std::abs(a - b) <= tol) return;
    std::printf("  FAIL %s: %.10g vs %.10g (tol %.3g)  (%s:%d)\n", what.c_str(), a, b, tol, file,
                line);
    ++g_failures;
}

inline int report(const char* name) {
    if (g_failures == 0) {
        std::printf("%s: OK\n", name);
        return 0;
    }
    std::printf("%s: %d failure(s)\n", name, g_failures);
    return 1;
}

}  // namespace mtl::test

#define CHECK(cond) ::mtl::test::check((cond), #cond, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) ::mtl::test::checkNear((a), (b), (tol), #a " ~ " #b, __FILE__, __LINE__)

#endif  // MTL_TEST_UTIL_HPP
