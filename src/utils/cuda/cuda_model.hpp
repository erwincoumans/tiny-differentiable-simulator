#pragma once

#include "cuda_function.hpp"

namespace tds {
template <typename Scalar> struct CudaModel {
  template<typename OtherScalar>
  friend class CudaLibrary;

protected:
  const std::string model_name_;

public:
  CudaFunction<Scalar> forward_zero;
  CudaFunction<Scalar> sparse_jacobian;

  const std::string &model_name() { return model_name_; }

private:
  CudaModel(const std::string &model_name, void *lib_handle)
      : model_name_(model_name) {
    forward_zero =
        CudaFunction<Scalar>(model_name + "_forward_zero", lib_handle);
    sparse_jacobian =
        CudaFunction<Scalar>(model_name + "_jacobian", lib_handle);
  }
};
} // namespace tds