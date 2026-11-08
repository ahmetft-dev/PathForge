#include <pathforge/geometry/vec2.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace pathforge::geometry;

// Tolerances: all inputs are small exactly-representable values (3-4-5 triangles etc.),
// so results are exact or off by at most a few ulp; 1e-12 is generous and still tight.
constexpr double kTol = 1e-12;

TEST(Vec2, Arithmetic) {
  const Vec2 a{1.0, 2.0}, b{3.0, -1.0};
  EXPECT_EQ(a + b, (Vec2{4.0, 1.0}));
  EXPECT_EQ(a - b, (Vec2{-2.0, 3.0}));
  EXPECT_EQ(2.0 * a, (Vec2{2.0, 4.0}));
  EXPECT_EQ(a * 0.5, (Vec2{0.5, 1.0}));
}

TEST(Vec2, DotCrossNorm) {
  EXPECT_DOUBLE_EQ(dot({1, 0}, {0, 1}), 0.0);
  EXPECT_DOUBLE_EQ(dot({2, 3}, {4, 5}), 23.0);
  EXPECT_DOUBLE_EQ(cross({1, 0}, {0, 1}), 1.0);   // CCW positive
  EXPECT_DOUBLE_EQ(cross({0, 1}, {1, 0}), -1.0);  // CW negative
  EXPECT_NEAR(norm({3, 4}), 5.0, kTol);
}

TEST(Vec2, Distance) {
  EXPECT_NEAR(distance({0, 0}, {3, 4}), 5.0, kTol);
  EXPECT_NEAR(distance({-1, -1}, {2, 3}), 5.0, kTol);
  EXPECT_DOUBLE_EQ(distance({1.5, 2.5}, {1.5, 2.5}), 0.0);
}

TEST(Vec2, IsFinite) {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(isFinite({1, 2}));
  EXPECT_FALSE(isFinite({nan, 0}));
  EXPECT_FALSE(isFinite({0, inf}));
}
