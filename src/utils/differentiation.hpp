#pragma once

#include <cmath>
#include <limits>

// clang-format off
// Stan Math needs to be included first to get its Eigen plugins
#if USE_STAN
#include <stan/math.hpp>
#include <stan/math/fwd.hpp>
#endif
#include <ceres/autodiff_cost_function.h>
// clang-format on

#include "base.hpp"
#include "math/tiny/tiny_dual.h"
#include "math/tiny/tiny_dual_utils.h"
#include "math/eigen_algebra.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/ceres_utils.h"

namespace tds {
enum DiffMethod {
  DIFF_NUMERICAL,
  DIFF_CERES,
  DIFF_DUAL,
  DIFF_STAN_REVERSE,
  DIFF_STAN_FORWARD,
};

/**
 * Central difference for scalar-valued function `f` given vector `x`.
 */
template <DiffMethod Method, typename F, typename Scalar = double>
static std::enable_if_t<Method == DIFF_NUMERICAL, void> compute_gradient(
    F f, const std::vector<Scalar>& x, std::vector<Scalar>& dfx,
    const Scalar eps = 1e-6) {
  dfx.resize(x.size());
  const Scalar fx = f(x);
  std::vector<Scalar> left_x = x, right_x = x;
  for (std::size_t i = 0; i < x.size(); ++i) {
    left_x[i] -= eps;
    right_x[i] += eps;
    Scalar dx = right_x[i] - left_x[i];
    Scalar fl = f(left_x);
    Scalar fr = f(right_x);
    dfx[i] = (fr - fl) / dx;
    left_x[i] = right_x[i] = x[i];
  }
}

namespace {
template <int Dim, template <typename> typename F, typename Scalar>
struct CeresFunctional {
  F<Scalar> f_double;
  F<ceres::Jet<Scalar, Dim>> f_jet;

  template <typename T>
  bool operator()(const T* const x, T* e) const {
    std::vector<T> arg(x, x + Dim);
    if constexpr (std::is_same_v<T, Scalar>) {
      *e = f_double(arg);
    } else {
      *e = f_jet(arg);
    }
    return true;
  }
};
}  // namespace

/**
 * Forward-mode autodiff using Ceres' Jet implementation.
 * Note that template argument `F` must be a functional that accepts
 * double and ceres::Jet<double, Dim> scalars, as declared by a template
 * argument on F.
 */
template <DiffMethod Method, int Dim, template <typename> typename F,
          typename Scalar = double>
static std::enable_if_t<Method == DIFF_CERES, void> compute_gradient(
    const std::vector<Scalar>& x, std::vector<Scalar>& dfx) {
  assert(static_cast<int>(x.size()) == Dim);
  dfx.resize(x.size());
  typedef CeresFunctional<Dim, F, Scalar> CF;
  ceres::AutoDiffCostFunction<CF, 1, Dim> cost_function(new CF);
  Scalar fx;
  Scalar* grad = dfx.data();
  const Scalar* params = x.data();
  cost_function.Evaluate(&params, &fx, &grad);
}

/**
 * Forward-mode AD using TinyDual for scalar-valued function `f` given vector
 * `x`.
 */
template <DiffMethod Method, typename F, typename Scalar = double>
static std::enable_if_t<Method == DIFF_DUAL, void> compute_gradient(
    F f, const std::vector<Scalar>& x, std::vector<Scalar>& dfx) {
  typedef TinyDual<Scalar> Dual;
  dfx.resize(x.size());
  std::vector<Dual> x_dual(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].real() = x[i];
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].dual() = 1.;
    Dual fx = f(x_dual);
    dfx[i] = fx.dual();
    x_dual[i].dual() = 0.;
  }
}

/**
 * Reverse-mode AD using Stan Math.
 */
template <DiffMethod Method, typename F>
static std::enable_if_t<Method == DIFF_STAN_REVERSE, void> compute_gradient(
    F f, const std::vector<double>& x, std::vector<double>& dfx) {
#if USE_STAN
  dfx.resize(x.size());

  std::vector<stan::math::var> x_var(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_var[i] = x[i];
  }

  stan::math::var fx = f(x_var);
  stan::math::grad(fx.vi_);

  for (std::size_t i = 0; i < x.size(); ++i) {
    dfx[i] = x_var[i].adj();
  }
  stan::math::recover_memory();
#else
  static_assert(false,
                "Variable 'USE_STAN' must be set to use automatic "
                "differentiation functions from Stan Math.");
#endif
}

/**
 * Forward-mode AD using Stan Math.
 */
template <DiffMethod Method, typename F, typename Scalar = double>
static std::enable_if_t<Method == DIFF_STAN_FORWARD, void> compute_gradient(
    F f, const std::vector<Scalar>& x, std::vector<Scalar>& dfx) {
#if USE_STAN
  typedef stan::math::fvar<Scalar> Dual;
  dfx.resize(x.size());

  std::vector<Dual> x_dual(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].val_ = x[i];
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].d_ = 1.;
    Dual fx = f(x_dual);
    dfx[i] = fx.d_;
    x_dual[i].d_ = 0.;
  }
#else
  static_assert(false,
                "Variable 'USE_STAN' must be set to use automatic "
                "differentiation functions from Stan Math.");
#endif
}

template <DiffMethod Method, template <typename> typename F,
          typename ScalarAlgebra = EigenAlgebra>
struct GradientFunctional {
  using Scalar = typename ScalarAlgebra::Scalar;
  virtual Scalar value(const std::vector<Scalar>&) const = 0;
  virtual const std::vector<Scalar>& gradient(
      const std::vector<Scalar>&) const = 0;
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_NUMERICAL, F, ScalarAlgebra> {
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
  mutable std::vector<Scalar> gradient_;

 public:
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_NUMERICAL>(f_scalar_, x, gradient_);
    return gradient_;
  }
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_CERES, F, ScalarAlgebra> {
  static const int kDim = F<ScalarAlgebra>::kDim;
  using Scalar = typename ScalarAlgebra::Scalar;
  using ADScalar = ceres::Jet<Scalar, kDim>;
  mutable std::vector<Scalar> gradient_;

  struct CostFunctional {
    GradientFunctional* parent;

    F<ScalarAlgebra> f_scalar;
    F<TinyAlgebra<ADScalar, CeresUtils<kDim>>> f_jet;

    CostFunctional(GradientFunctional* parent) : parent(parent) {}

    template <typename T>
    bool operator()(const T* const x, T* e) const {
      std::vector<T> arg(x, x + kDim);
      if constexpr (std::is_same_v<T, Scalar>) {
        *e = f_scalar(arg);
      } else {
        *e = f_jet(arg);
      }
      return true;
    }
  };

  CostFunctional* cost_{nullptr};
  ceres::AutoDiffCostFunction<CostFunctional, 1, kDim> cost_function_;

 public:
  // CostFunctional pointer is managed by cost_function_.
  GradientFunctional()
      : cost_(new CostFunctional(this)), cost_function_(cost_) {}
  GradientFunctional(GradientFunctional& f)
      : cost_(new CostFunctional(this)), cost_function_(cost_) {}
  GradientFunctional(const GradientFunctional& f)
      : cost_(new CostFunctional(this)), cost_function_(cost_) {}
  GradientFunctional& operator=(const GradientFunctional& f) {
    if (cost_) {
      delete cost_;
    }
    cost_ = new CostFunctional(this);
    cost_function_ =
        ceres::AutoDiffCostFunction<CostFunctional, 1, kDim>(cost_);
    return *this;
  }

  Scalar value(const std::vector<Scalar>& x) const {
    return cost_->f_scalar(x);
  }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    assert(static_cast<int>(x.size()) == kDim);
    gradient_.resize(x.size());
    Scalar fx;
    Scalar* grad = gradient_.data();
    const Scalar* params = x.data();
    cost_function_.Evaluate(&params, &fx, &grad);
    return gradient_;
  }
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_DUAL, F, ScalarAlgebra> {
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
  F<TinyAlgebra<TinyDual<Scalar>, TinyDualUtils<Scalar>>> f_ad_;
  mutable std::vector<Scalar> gradient_;

 public:
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_DUAL>(f_ad_, x, gradient_);
    return gradient_;
  }
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_STAN_REVERSE, F, ScalarAlgebra> {
#if USE_STAN
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
  F<EigenAlgebraT<stan::math::var>> f_ad_;
  mutable std::vector<Scalar> gradient_;

 public:
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_STAN_REVERSE>(f_ad_, x, gradient_);
    return gradient_;
  }
#else
  static_assert(false,
                "Variable 'USE_STAN' must be set to use automatic "
                "differentiation functions from Stan Math.");
#endif
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_STAN_FORWARD, F, ScalarAlgebra> {
#if USE_STAN
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
  F<EigenAlgebraT<stan::math::fvar<Scalar>>> f_ad_;
  mutable std::vector<Scalar> gradient_;

 public:
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_STAN_FORWARD>(f_ad_, x, gradient_);
    return gradient_;
  }
#else
  static_assert(false,
                "Variable 'USE_STAN' must be set to use automatic "
                "differentiation functions from Stan Math.");
#endif
};
}  // namespace tds