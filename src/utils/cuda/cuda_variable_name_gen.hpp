#pragma once

namespace tds {
template <class Base>
class CudaVariableNameGenerator
    : public CppAD::cg::LangCDefaultVariableNameGenerator<Base> {
protected:
  // defines how many input indices belong to the global input
  std::size_t global_input_dim_{0};
  // name of thread-local input
  std::string local_name_;

public:
  inline explicit CudaVariableNameGenerator(
      std::size_t global_input_dim, std::string depName = "y",
      std::string indepName = "x", std::string localName = "xj",
      std::string tmpName = "v", std::string tmpArrayName = "array",
      std::string tmpSparseArrayName = "sarray")
      : CppAD::cg::LangCDefaultVariableNameGenerator<Base>(
            depName, indepName, tmpName, tmpArrayName, tmpSparseArrayName),
        global_input_dim_(global_input_dim), local_name_(std::move(localName)) {
  }

  std::size_t global_input_dim() const { return global_input_dim_; }
  const std::string &local_name() const { return local_name_; }
  const std::string &independent_name() const { return this->_indepName; }
  const std::string &dependent_name() const { return this->_depName; }

  inline std::string
  generateIndependent(const CppAD::cg::OperationNode<Base> &independent,
                      size_t id) override {
    this->_ss.clear();
    this->_ss.str("");

    if (id - 1 >= global_input_dim_) {
      // global inputs access directly independent vars starting from index 0
      this->_ss << this->local_name_ << "[" << (id - 1 - global_input_dim_)
                << "]";
    } else {
      // thread-local inputs use 'xj' (offset of input 'x')
      this->_ss << this->_indepName << "[" << (id - 1) << "]";
    }

    return this->_ss.str();
  }
};
} // namespace tds
