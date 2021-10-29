#pragma once

#include <cstdlib>
#include <limits>
#include <string>
#include <cmath>

namespace tds {
struct EstimationParameter {
  std::string name{"unnamed_param"};
  double value{1.0};
  double minimum{-std::numeric_limits<double>::infinity()};
  double maximum{std::numeric_limits<double>::infinity()};

  // coefficient of L1 regularization for this parameter
  double l1_regularization{0.};

  // coefficient of L2 regularization for this parameter
  double l2_regularization{0.};

  EstimationParameter& operator=(double rhs) {
    value = rhs;
    return *this;
  }
  explicit operator double() const { return value; }

  double random_value() const {
    return minimum + (double(std::rand()) / RAND_MAX * (maximum - minimum));
  };

  bool has_limits() const {
    return !std::isinf(minimum) && !std::isinf(maximum);
  }

  /**
   * Compute loss contribution of this parameter given its actual value used in
   * the loss function.
   */
  template <typename Algebra>
  typename Algebra::Scalar loss(
      const typename Algebra::Scalar& parameter_value) const {
    return Algebra::abs(parameter_value) *
               Algebra::from_double(l1_regularization) +
           parameter_value * parameter_value *
               Algebra::from_double(l2_regularization);
  }

  //friend std::ostream& operator<<(std::ostream& stream, const EstimationParameter& p) {
  //  stream << p.name << " = " << p.value;
  //  return stream;
  //}
};
}  // namespace tds
