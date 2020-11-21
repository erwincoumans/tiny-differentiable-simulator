#pragma once

#if USE_PAGMO
#include <pagmo/threading.hpp>
#include <pagmo/types.hpp>
#endif

#include "differentiation.hpp"
#include "parameter.hpp"

namespace tds {

template <DiffMethod Method, template <typename> typename F>
class OptimizationProblem {
 public:
  static const inline int kParameterDim = F<EigenAlgebra>::kDim;
  static_assert(kParameterDim >= 1);
  typedef std::array<EstimationParameter, kParameterDim> ParameterVector;
  typedef GradientFunctional<Method, F> CostFunctor;
  static const DiffMethod kDiffMethod = Method;

#if USE_PAGMO
  typedef pagmo::vector_double DoubleVector;
#else
  typedef std::vector<double> DoubleVector;
#endif

  const unsigned int m_dim = kParameterDim;

 protected:
  CostFunctor cost_;
  ParameterVector parameters_;

 public:
  TINY_INLINE ParameterVector& parameters() { return parameters_; }
  TINY_INLINE const ParameterVector& parameters() const { return parameters_; }

  TINY_INLINE EstimationParameter& operator[](int i) { return parameters_[i]; }
  TINY_INLINE const EstimationParameter& operator[](int i) const {
    return parameters_[i];
  }

  TINY_INLINE CostFunctor& cost() { return cost_; }
  TINY_INLINE const CostFunctor& cost() const { return cost_; }

  void set_params(const std::array<double, kParameterDim>& params) {
    parameters_ = params;
    // for (int i = 0; i < kParameterDim; ++i) {
    //   parameters_[i].value = params[i];
    // }
  }

  OptimizationProblem() = default;
  OptimizationProblem(OptimizationProblem&) = default;
  OptimizationProblem(const OptimizationProblem&) = default;
  OptimizationProblem(const ParameterVector& parameters)
      : parameters_(parameters) {}
  OptimizationProblem& operator=(const OptimizationProblem&) = default;

  virtual ~OptimizationProblem() = default;

  std::pair<DoubleVector, DoubleVector> get_bounds() const {
    DoubleVector low(kParameterDim), high(kParameterDim);
    for (int i = 0; i < kParameterDim; ++i) {
      low[i] = parameters_[i].minimum;
      high[i] = parameters_[i].maximum;
    }
    return {low, high};
  }

  TINY_INLINE DoubleVector fitness(const DoubleVector& x) const {
    return {cost_.value(x)};
  }

  TINY_INLINE DoubleVector gradient(const DoubleVector& x) const {
    return cost_.gradient(x);
  }

  bool has_gradient() const { return true; }

#if USE_PAGMO
  pagmo::thread_safety get_thread_safety() const {
    return pagmo::thread_safety::basic;
  }
#endif
};
}  // namespace tds
