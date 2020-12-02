#include <gtest/gtest.h>

#include "utils/differentiation.hpp"

const double kEpsilon = 1e-6;

template <typename Scalar>
struct l2 {
  Scalar operator()(const std::vector<Scalar>& vs) const {
    Scalar n(0.);
    for (const Scalar& v : vs) {
      n += v * v;
    }
    return n;
  }
};

TEST(Differentiation, FiniteDiffSimple) {
  std::vector<double> x{0.2, -0.5, 0.1, 123.45};
  std::vector<double> grad;
  tds::compute_gradient<tds::DIFF_NUMERICAL>(l2<double>(), x, grad);
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(grad[i], 2. * x[i], kEpsilon);
  }
}

#if USE_CERES
TEST(Differentiation, CeresSimple) {
  std::vector<double> x{0.2, -0.5, 0.1, 123.45};
  std::vector<double> grad;
  const int kDim = 4;
  tds::compute_gradient<tds::DIFF_CERES, kDim, l2>(x, grad);
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(grad[i], 2. * x[i], kEpsilon);
  }
}
#endif

TEST(Differentiation, DualSimple) {
  std::vector<double> x{0.2, -0.5, 0.1, 123.45};
  std::vector<double> grad;
  tds::compute_gradient<tds::DIFF_DUAL>(l2<TINY::TinyDual<double>>(), x, grad);
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(grad[i], 2. * x[i], kEpsilon);
  }
}

#if USE_STAN
TEST(Differentiation, StanReverseSimple) {
  std::vector<double> x{0.2, -0.5, 0.1, 123.45};
  std::vector<double> grad;
  tds::compute_gradient<tds::DIFF_STAN_REVERSE>(l2<stan::math::var>(), x, grad);
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(grad[i], 2. * x[i], kEpsilon);
  }
}

TEST(Differentiation, StanForwardSimple) {
  std::vector<double> x{0.2, -0.5, 0.1, 123.45};
  std::vector<double> grad;
  tds::compute_gradient<tds::DIFF_STAN_FORWARD>(l2<stan::math::fvar<double>>(),
                                                x, grad);
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(grad[i], 2. * x[i], kEpsilon);
  }
}
#endif
