#pragma once

#include "neural_scalar.hpp"
#include "tiny_algebra.hpp"

namespace tds {
template <typename Algebra>
using NeuralAlgebra =
    TinyAlgebra<NeuralScalar<Algebra>, NeuralScalarUtils<Algebra>>;

template <typename Algebra>
struct is_neural_algebra<NeuralAlgebra<Algebra>> {
  static constexpr bool value = true;
};

template <typename Algebra>
TINY_INLINE typename Algebra::Scalar from_neural(
    const typename NeuralAlgebra<Algebra>::Scalar& value) {
  return value.evaluate();
}

template <typename Algebra>
TINY_INLINE std::vector<typename Algebra::Scalar> from_neural(
    const std::vector<typename NeuralAlgebra<Algebra>::Scalar>& values) {
  std::vector<typename Algebra::Scalar> output(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    output[i] = values[i].evaluate();
  }
  return output;
}
}  // namespace tds
