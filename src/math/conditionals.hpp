#pragma once

#ifdef USE_CPPAD

#ifdef USE_CPPAD_CODEGEN
#include <cppad/cg.hpp>
#else
#include <cppad/cppad.hpp>
#endif

#endif

#include "base.hpp"
// #include "math/eigen_algebra.hpp"

// template <class T>
// static CppAD::AD<T> max(const CppAD::AD<T>& a, const CppAD::AD<T>& b) {
//   std::cout << "max\n";
//   return CppAD::CondExpGt(a, b, a, b);
// }
// template <class T>
// static CppAD::AD<T> min(const CppAD::AD<T>& a, const CppAD::AD<T>& b) {
//   std::cout << "min\n";
//   return CppAD::CondExpLt(a, b, a, b);
// }

namespace tds {

template <typename Scalar>
struct is_cppad_scalar {
  static constexpr bool value = false;
};
#ifdef USE_CPPAD

#ifdef USE_CPPAD_CODEGEN
template <typename Scalar>
struct is_cppad_scalar<CppAD::cg::CG<Scalar>> {
  static constexpr bool value = true;
};
#endif
template <typename Scalar>
struct is_cppad_scalar<CppAD::AD<Scalar>> {
  static constexpr bool value = true;
};

template <typename Scalar>
static TINY_INLINE CppAD::AD<Scalar> where_gt(
    const CppAD::AD<Scalar>& x, const CppAD::AD<Scalar>& y,
    const CppAD::AD<Scalar>& if_true, const CppAD::AD<Scalar>& if_false) {
  return CppAD::CondExpGt(x, y, if_true, if_false);
}

template <typename Scalar>
static TINY_INLINE CppAD::AD<Scalar> where_ge(
    const CppAD::AD<Scalar>& x, const CppAD::AD<Scalar>& y,
    const CppAD::AD<Scalar>& if_true, const CppAD::AD<Scalar>& if_false) {
  return CppAD::CondExpGe(x, y, if_true, if_false);
}

template <typename Scalar>
static TINY_INLINE CppAD::AD<Scalar> where_lt(
    const CppAD::AD<Scalar>& x, const CppAD::AD<Scalar>& y,
    const CppAD::AD<Scalar>& if_true, const CppAD::AD<Scalar>& if_false) {
  return CppAD::CondExpLt(x, y, if_true, if_false);
}
template <typename Scalar>
static TINY_INLINE CppAD::AD<Scalar> where_le(
    const CppAD::AD<Scalar>& x, const CppAD::AD<Scalar>& y,
    const CppAD::AD<Scalar>& if_true, const CppAD::AD<Scalar>& if_false) {
  return CppAD::CondExpLe(x, y, if_true, if_false);
}
template <typename Scalar>
static TINY_INLINE CppAD::AD<Scalar> where_eq(
    const CppAD::AD<Scalar>& x, const CppAD::AD<Scalar>& y,
    const CppAD::AD<Scalar>& if_true, const CppAD::AD<Scalar>& if_false) {
  return CppAD::CondExpEq(x, y, if_true, if_false);
}
template <typename Scalar>
static TINY_INLINE CppAD::AD<Scalar> isnan(const CppAD::AD<Scalar>& x,
                                           const CppAD::AD<Scalar>& if_true,
                                           const CppAD::AD<Scalar>& if_false) {
  return CppAD::CondExpEq(x, x, if_false, if_true);
}

#endif  // USE_CPPAD

template <typename Scalar>
static TINY_INLINE Scalar where_gt(const Scalar& x, const Scalar& y,
                                   const Scalar& if_true,
                                   const Scalar& if_false) {
  return x > y ? if_true : if_false;
}

template <typename Scalar>
static TINY_INLINE Scalar where_ge(const Scalar& x, const Scalar& y,
                                   const Scalar& if_true,
                                   const Scalar& if_false) {
  return x >= y ? if_true : if_false;
}

template <typename Scalar>
static TINY_INLINE Scalar where_lt(const Scalar& x, const Scalar& y,
                                   const Scalar& if_true,
                                   const Scalar& if_false) {
  return x < y ? if_true : if_false;
}

template <typename Scalar>
static TINY_INLINE Scalar where_le(const Scalar& x, const Scalar& y,
                                   const Scalar& if_true,
                                   const Scalar& if_false) {
  return x <= y ? if_true : if_false;
}

template <typename Scalar>
static TINY_INLINE Scalar where_eq(const Scalar& x, const Scalar& y,
                                   const Scalar& if_true,
                                   const Scalar& if_false) {
  return x == y ? if_true : if_false;
}

template <typename Scalar>
static TINY_INLINE Scalar isnan(const Scalar& x, const Scalar& if_true,
                                const Scalar& if_false) {
  return x == x ? if_false : if_true;
}

template <typename Scalar>
static TINY_INLINE Scalar clamp(const Scalar& x, const Scalar& low,
                                const Scalar& high) {
  return where_lt(x, low, low, where_gt(x, high, high, x));
}
}  // namespace tds
