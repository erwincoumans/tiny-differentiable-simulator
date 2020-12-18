#pragma once

#include <numeric>

#include <cppad/cg.hpp>
#include <cppad/cg/arithmetic.hpp>
#include <cppad/cg/support/cppadcg_eigen.hpp>

#if CPPAD_CG_SYSTEM_WIN
#include <windows.h>
#endif

#include "file_utils.hpp"
#include "stopwatch.hpp"

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
        global_input_dim_(global_input_dim),
        local_name_(std::move(localName)) {}

  inline std::string generateIndependent(
      const CppAD::cg::OperationNode<Base>& independent, size_t id) override {
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

struct CudaFunctionMetaData {
  int output_dim;
  int local_input_dim;
  int global_input_dim;
  bool accumulated_output;
};

enum CudaAccumulationMethod {
  CUDA_ACCUMULATE_NONE,
  CUDA_ACCUMULATE_SUM,
  CUDA_ACCUMULATE_MEAN
};

namespace {
struct CudaFunctionSourceGen {
  std::string function_name;
  int local_input_dim;
  int global_input_dim;
  int output_dim;
  CudaAccumulationMethod acc_method;

  CudaFunctionSourceGen(const std::string& function_name, int local_input_dim,
                        int global_input_dim, int output_dim,
                        CudaAccumulationMethod acc_method)
      : function_name(function_name),
        local_input_dim(local_input_dim),
        global_input_dim(global_input_dim),
        output_dim(output_dim),
        acc_method(acc_method) {}

  void emit_header(std::ostringstream& code) const {
    code << "#include \"util.h\"\n\n";

    // meta data retrieval function
    code << "extern \"C\" {\nMODULE_API CudaFunctionMetaData " << function_name
         << "_meta() {\n";
    code << "  CudaFunctionMetaData data;\n";
    code << "  data.output_dim = " << output_dim << ";\n";
    code << "  data.local_input_dim = " << local_input_dim << ";\n";
    code << "  data.global_input_dim = " << global_input_dim << ";\n";
    code << "  data.accumulated_output = " << std::boolalpha
         << (acc_method != CUDA_ACCUMULATE_NONE) << ";\n";
    code << "  return data;\n}\n}\n";
  }

  void emit_kernel(std::ostringstream& code, std::size_t temporary_dim,
                   const std::ostringstream& body) const {
    code << "\n__global__\n";
    std::string kernel_name = function_name + "_kernel";
    std::string fun_head_start = "void " + kernel_name + "(";
    std::string fun_arg_pad = std::string(fun_head_start.size(), ' ');
    code << fun_head_start;
    code << "int num_total_threads,\n";
    code << fun_arg_pad << "Float *output,\n";
    code << fun_arg_pad << "const Float *local_input";
    if (global_input_dim > 0) {
      code << ",\n" << fun_arg_pad << "const Float *global_input";
    }
    code << ") {\n";
    code << "   const int i = blockIdx.x * blockDim.x + threadIdx.x;\n";
    code << "   if (i >= num_total_threads) {\n";
    code << "      printf(\"ERROR: thread index %i exceeded provided "
            "number of total threads %i.\\n\", i, num_total_threads);\n";
    code << "      return;\n   }\n\n";
    if (temporary_dim > 0) {
      code << "   Float v[" << temporary_dim << "];\n";
    }
    if (global_input_dim > 0) {
      code << "   const Float *x = &(global_input[0]);  // global input\n";
    }
    code << "   const Float *xj = &(local_input[i * " << local_input_dim
         << "]);  // thread-local input\n";
    code << "   Float *y = &(output[i * " << output_dim << "]);\n";

    code << "\n";

    code << body.str();

    code << "}\n\n";
  }

  void emit_allocation_functions(std::ostringstream& code) const {
    // global device memory pointers
    code << "Float* dev_" << function_name << "_output = nullptr;\n";
    code << "Float* dev_" << function_name << "_local_input = nullptr;\n";
    code << "Float* dev_" << function_name << "_global_input = nullptr;\n\n";

    // allocation function
    code << "extern \"C\" {\nMODULE_API void " << function_name
         << "_allocate(int num_total_threads) {\n";
    code << "  const size_t output_dim = num_total_threads * " << output_dim
         << ";\n";
    code << "  const size_t input_dim = num_total_threads * " << local_input_dim
         << ";\n\n";
    code << "  allocate((void**)&dev_" << function_name
         << "_output, output_dim * sizeof(Float));\n";
    code << "  allocate((void**)&dev_" << function_name
         << "_local_input, input_dim * "
            "sizeof(Float));\n";
    code << "  allocate((void**)&dev_" << function_name << "_global_input, "
         << global_input_dim << " * sizeof(Float));\n";
    code << "}\n\n";

    // deallocation function
    code << "MODULE_API void " << function_name << "_deallocate() {\n";
    code << "  cudaFreeHost(dev_" << function_name << "_output);\n";
    code << "  cudaFreeHost(dev_" << function_name << "_local_input);\n";
    code << "  cudaFreeHost(dev_" << function_name << "_global_input);\n";
    code << "  // cudaDeviceReset();\n";
    code << "}\n\n";
  }

  void emit_send_functions(std::ostringstream& code) const {
    // send thread-local inputs to GPU
    std::string fun_head_start =
        "MODULE_API bool " + function_name + "_send_local(";
    std::string fun_arg_pad = std::string(fun_head_start.size(), ' ');
    code << fun_head_start;
    code << "int num_total_threads,\n";
    code << fun_arg_pad << "const Float *input) {\n";
    code << "  const size_t input_dim = num_total_threads * " << local_input_dim
         << ";\n";
    code << "  cudaError status = cudaMemcpy(dev_" << function_name
         << "_local_input, input, "
            "input_dim * sizeof(Float), "
            "cudaMemcpyHostToDevice);\n";
    code << R"(  if (status != cudaSuccess) {
    fprintf(stderr, "Error %i (%s) while sending thread-local input data to GPU: %s.\n",
            status, cudaGetErrorName(status), cudaGetErrorString(status));
    return false;
  }
)";
    code << "  return true;\n}\n\n";

    // send global input to GPU
    code << "MODULE_API bool " + function_name + "_send_global(";
    code << "const Float *input) {\n";
    code << "  cudaError status = cudaMemcpy(dev_" << function_name
         << "_global_input, input, " << global_input_dim
         << " * sizeof(Float), "
            "cudaMemcpyHostToDevice);\n";
    code << R"(  if (status != cudaSuccess) {
    fprintf(stderr, "Error %i (%s) while sending global input data to GPU: %s.\n",
            status, cudaGetErrorName(status), cudaGetErrorString(status));
    return false;
  }
)";
    code << "  return true;\n}\n\n";
  }

  void emit_kernel_launch(std::ostringstream& code) const {
    std::string fun_head_start = "MODULE_API void " + function_name + "(";
    std::string fun_arg_pad = std::string(fun_head_start.size(), ' ');
    code << fun_head_start;
    code << "int num_total_threads,\n";
    code << fun_arg_pad << "int num_blocks,\n";
    code << fun_arg_pad << "int num_threads_per_block,\n";
    code << fun_arg_pad << "Float *output) {\n";

    code << "  const size_t output_dim = num_total_threads * " << output_dim
         << ";\n";

    std::string kernel_name = function_name + "_kernel";
    fun_head_start =
        "  " + kernel_name + "<<<num_blocks, num_threads_per_block>>>(";
    fun_arg_pad = std::string(fun_head_start.size(), ' ');
    code << fun_head_start;
    code << "num_total_threads,\n";
    code << fun_arg_pad << "dev_" << function_name << "_output,\n";
    code << fun_arg_pad << "dev_" << function_name << "_local_input";
    if (global_input_dim > 0) {
      code << ",\n"
           << fun_arg_pad << "dev_" << function_name << "_global_input";
    }
    code << ");\n";
    code << R"(
  // cudaDeviceSynchronize waits for the kernel to finish, and returns
  // any errors encountered during the launch.
  cudaDeviceSynchronize();
  cudaError status = cudaGetLastError();
  if (status != cudaSuccess) {
    fprintf(stderr, "Error %i (%s) while executing CUDA kernel: %s.\n",
            status, cudaGetErrorName(status), cudaGetErrorString(status));
    exit((int)status);
  }

  // Copy output vector from GPU buffer to host memory.
  )";
    code << "cudaMemcpy(output, dev_" << function_name
         << "_output, output_dim * sizeof(Float), cudaMemcpyDeviceToHost);\n";
    code << R"(  status = cudaGetLastError();
  if (status != cudaSuccess) {
    fprintf(stderr, "Error %i (%s) while retrieving output from kernel: %s.\n",
            status, cudaGetErrorName(status), cudaGetErrorString(status));
    exit((int)status);
  })";

    if (acc_method != CUDA_ACCUMULATE_NONE) {
      code << "\n\n  // accumulate thread-wise outputs\n";
      code << "  for (int i = 1; i < num_total_threads; ++i) {\n";
      code << "    for (int j = 0; j < " << output_dim << "; ++j) {\n";
      code << "      output[j] += output[i*" << output_dim << " + j];\n";
      code << "    }\n  }\n";
      if (acc_method == CUDA_ACCUMULATE_MEAN) {
        code << "  for (int j = 0; j < " << output_dim << "; ++j) {\n";
        code << "    output[j] /= num_total_threads;\n  }";
      }
    }

    code << "\n}\n}\n";
  }
};
}  // namespace

template <class Base>
class CudaSourceGen : public CppAD::cg::ModelCSourceGen<Base> {
  using CGBase = CppAD::cg::CG<Base>;

  std::size_t global_input_dim_{0};

  CudaAccumulationMethod jac_acc_method_{CUDA_ACCUMULATE_MEAN};

  std::vector<std::size_t> jac_local_input_sparsity_;
  std::vector<std::size_t> jac_global_input_sparsity_;
  std::vector<std::size_t> jac_output_sparsity_;

 public:
  CudaSourceGen(CppAD::ADFun<CppAD::cg::CG<Base>>& fun, std::string model)
      : CppAD::cg::ModelCSourceGen<Base>(fun, model) {}

  std::size_t& global_input_dim() { return global_input_dim_; }
  const std::size_t& global_input_dim() const { return global_input_dim_; }

  std::size_t local_input_dim() const {
    return this->_fun.Domain() - global_input_dim_;
  }
  std::size_t output_dim() const { return this->_fun.Range(); }

  std::string base_type_name() const { return this->_baseTypeName; }

  CudaAccumulationMethod& jacobian_acc_method() { return jac_acc_method_; }
  const CudaAccumulationMethod& jacobian_acc_method() const {
    return jac_acc_method_;
  }

  void set_jac_local_input_sparsity(const std::vector<std::size_t>& sparsity) {
    for (auto idx : sparsity) {
      assert(idx < local_input_dim());
    }
    jac_local_input_sparsity_ = sparsity;
  }
  void set_jac_global_input_sparsity(const std::vector<std::size_t>& sparsity) {
    for (auto idx : sparsity) {
      assert(idx < global_input_dim());
    }
    jac_global_input_sparsity_ = sparsity;
  }
  void set_jac_output_sparsity(const std::vector<std::size_t>& sparsity) {
    for (auto idx : sparsity) {
      assert(idx < output_dim());
    }
    jac_output_sparsity_ = sparsity;
  }

  const std::map<std::string, std::string>& sources() {
    auto mtt = CppAD::cg::MultiThreadingType::NONE;
    CppAD::cg::JobTimer* timer = nullptr;
    return this->getSources(mtt, timer);
  }

  std::string jacobian_source() {
    const std::string jobName = "sparse Jacobian";

    if (jac_local_input_sparsity_.empty() &&
        jac_global_input_sparsity_.empty()) {
      // assume dense Jacobian
      jac_local_input_sparsity_.resize(local_input_dim());
      std::iota(jac_local_input_sparsity_.begin(),
                jac_local_input_sparsity_.end(), 0);
      jac_global_input_sparsity_.resize(global_input_dim());
      std::iota(jac_global_input_sparsity_.begin(),
                jac_global_input_sparsity_.end(), 0);
    }
    if (jac_output_sparsity_.empty()) {
      jac_output_sparsity_.resize(1);  // output_dim());
      std::iota(jac_output_sparsity_.begin(), jac_output_sparsity_.end(), 0);
    }

    std::vector<std::size_t> rows, cols;
    for (std::size_t output_i : jac_output_sparsity_) {
      for (std::size_t input_i : jac_global_input_sparsity_) {
        rows.push_back(output_i);
        cols.push_back(input_i);
      }
      for (std::size_t input_i : jac_local_input_sparsity_) {
        rows.push_back(output_i);
        cols.push_back(input_i + global_input_dim_);
      }
    }
    this->setCustomSparseJacobianElements(rows, cols);
    this->determineJacobianSparsity();

    // size_t m = _fun.Range();
    std::size_t n = _fun.Domain();

    startingJob("'" + jobName + "'", CppAD::cg::JobTimer::GRAPH);

    CppAD::cg::CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    std::vector<CGBase> indVars(n);
    handler.makeVariables(indVars);
    if (_x.size() > 0) {
      for (size_t i = 0; i < n; i++) {
        indVars[i].setValue(_x[i]);
      }
    }

    std::vector<CGBase> jac(this->_jacSparsity.rows.size());
    bool forward = local_input_dim() + global_input_dim() <= output_dim();
    if (this->_loopTapes.empty()) {
      // printSparsityPattern(this->_jacSparsity.sparsity, "jac sparsity");
      CppAD::sparse_jacobian_work work;
      // work.color

      if (forward) {
        _fun.SparseJacobianForward(indVars, this->_jacSparsity.sparsity,
                                   this->_jacSparsity.rows,
                                   this->_jacSparsity.cols, jac, work);
      } else {
        _fun.SparseJacobianReverse(indVars, this->_jacSparsity.sparsity,
                                   this->_jacSparsity.rows,
                                   this->_jacSparsity.cols, jac, work);
      }
    } else {
      jac = prepareSparseJacobianWithLoops(handler, indVars, forward);
    }

    finishedJob();

    CppAD::cg::LanguageC<Base> langC(this->_baseTypeName);
    langC.setMaxAssignmentsPerFunction(this->_maxAssignPerFunc,
                                       &this->_sources);
    langC.setMaxOperationsPerAssignment(this->_maxOperationsPerAssignment);
    langC.setParameterPrecision(this->_parameterPrecision);
    langC.setGenerateFunction("");  // _name + "_" + FUNCTION_SPARSE_JACOBIAN

    std::ostringstream code;

    CudaVariableNameGenerator<Base> nameGen(global_input_dim_);

    handler.generateCode(code, langC, jac, nameGen, this->_atomicFunctions,
                         jobName);

    std::size_t temporary_dim = nameGen.getMaxTemporaryVariableID() + 1 -
                                nameGen.getMinTemporaryVariableID();
    if (temporary_dim == 0) {
      std::cerr << "Warning: generated code has no temporary variables.\n";
    } else {
      std::cout << "Code generated with " << temporary_dim
                << " temporary variables.\n";
    }

    std::ostringstream complete;

    CudaFunctionSourceGen generator(std::string(this->_name) + "_jacobian",
                                    local_input_dim(), global_input_dim_,
                                    rows.size(), jac_acc_method_);

    generator.emit_header(complete);
    generator.emit_kernel(complete, temporary_dim, code);
    generator.emit_allocation_functions(complete);
    generator.emit_send_functions(complete);
    generator.emit_kernel_launch(complete);

    return complete.str();
  }

  std::string jacobian_source(
      const std::vector<std::size_t>& local_indices,
      const std::vector<std::size_t>& global_indices,
      CudaAccumulationMethod acc_method = CUDA_ACCUMULATE_MEAN) {
    const std::size_t output_dim = this->_fun.Range();
    std::vector<size_t> output_indices(output_dim, 0);
    std::iota(output_indices.begin(), output_indices.end(), 0);
    return jacobian_source(local_indices, global_indices, output_indices,
                           acc_method);
  }

  std::string jacobian_source(
      const std::vector<std::size_t>& global_indices,
      CudaAccumulationMethod acc_method = CUDA_ACCUMULATE_MEAN) {
    const std::size_t input_dim = this->_fun.Domain() - global_input_dim_;
    std::vector<size_t> local_indices(input_dim, 0);
    std::iota(local_indices.begin(), local_indices.end(), 0);
    return jacobian_source(local_indices, global_indices, acc_method);
  }

  /**
   * Generate CUDA library code for the forward zero pass.
   */
  std::string zero_source() {
    const std::string jobName = "model (zero-order forward)";

    this->startingJob("'" + jobName + "'", CppAD::cg::JobTimer::GRAPH);

    CppAD::cg::CodeHandler<Base> handler;
    handler.setJobTimer(this->_jobTimer);

    if (global_input_dim_ > this->_fun.Domain()) {
      throw std::runtime_error(
          "CUDA codegen failed: global data input size must not be "
          "larger than the provided input vector size.");
    }

    const std::size_t local_input_dim = this->_fun.Domain() - global_input_dim_;
    const std::size_t output_dim = this->_fun.Range();

    std::cout << "Generating code for function with input dimension "
              << local_input_dim << " and output dimension " << output_dim
              << "...\n";

    std::vector<CGBase> indVars(local_input_dim + global_input_dim_);
    handler.makeVariables(indVars);
    if (this->_x.size() > 0) {
      for (std::size_t i = 0; i < indVars.size(); i++) {
        indVars[i].setValue(this->_x[i]);
      }
    }

    std::vector<CGBase> dep;

    if (this->_loopTapes.empty()) {
      dep = this->_fun.Forward(0, indVars);
    } else {
      /**
       * Contains loops
       */
      dep = this->prepareForward0WithLoops(handler, indVars);
    }

    this->finishedJob();

    CppAD::cg::LanguageC<Base> langC(this->_baseTypeName);
    langC.setMaxAssignmentsPerFunction(this->_maxAssignPerFunc,
                                       &this->_sources);
    langC.setMaxOperationsPerAssignment(this->_maxOperationsPerAssignment);
    langC.setParameterPrecision(this->_parameterPrecision);
    langC.setGenerateFunction("");  // this->_name + "_forward_zero");

    std::ostringstream code;
    CudaVariableNameGenerator<Base> nameGen(global_input_dim_);

    handler.generateCode(code, langC, dep, nameGen, this->_atomicFunctions,
                         jobName);

    std::size_t temporary_dim = nameGen.getMaxTemporaryVariableID() + 1 -
                                nameGen.getMinTemporaryVariableID();
    if (temporary_dim == 0) {
      std::cerr << "Warning: generated code has no temporary variables.\n";
    } else {
      std::cout << "Code generated with " << temporary_dim
                << " temporary variables.\n";
    }
    // for (const auto& var : nameGen.getTemporary()) {
    //   std::cout << "\t" << var.name << std::endl;
    // }

    CudaFunctionSourceGen generator(std::string(this->_name) + "_forward_zero",
                                    local_input_dim, global_input_dim_,
                                    output_dim, CUDA_ACCUMULATE_NONE);

    std::ostringstream complete;

    generator.emit_header(complete);
    generator.emit_kernel(complete, temporary_dim, code);
    generator.emit_allocation_functions(complete);
    generator.emit_send_functions(complete);
    generator.emit_kernel_launch(complete);

    return complete.str();
  }
};

namespace {
template <typename FunctionPtrT>
FunctionPtrT load_function(const std::string& function_name, void* lib_handle) {
#if CPPAD_CG_SYSTEM_WIN
  auto ptr =
      (FunctionPtrT)GetProcAddress((HMODULE)lib_handle, function_name.c_str());
  if (!ptr) {
    throw std::runtime_error("Cannot load symbol '" + function_name +
                             "': error code " + std::to_string(GetLastError()));
  }
#else
  auto ptr = (FunctionPtrT)dlsym(lib_handle, function_name.c_str());
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    throw std::runtime_error("Cannot load symbol '" + function_name +
                             "': " + std::string(dlsym_error));
  }
#endif
  return ptr;
}
}  // namespace

template <typename Scalar>
using DefaultCudaFunctionPtrT = void (*)(int, int, int, Scalar*);
template <typename Scalar>
using SendLocalFunctionPtrT = bool (*)(int, const Scalar*);
template <typename Scalar>
using SendGlobalFunctionPtrT = bool (*)(const Scalar*);

using MetaDataFunctionPtrT = CudaFunctionMetaData (*)();
using AllocateFunctionPtrT = void (*)(int);
using DeallocateFunctionPtrT = void (*)();

template <typename Scalar,
          typename kFunctionPtrT = DefaultCudaFunctionPtrT<Scalar>>
struct CudaFunction {
  using FunctionPtrT = kFunctionPtrT;
  std::string function_name;

 protected:
  CudaFunctionMetaData meta_data_{};
  bool is_available_{false};

  public:
  bool is_available() const { return is_available_; };

  /**
   * Global input dimension.
   */
  int global_input_dim() const { return meta_data_.global_input_dim; }
  /**
   * Input dimension per thread.
   */
  int local_input_dim() const { return meta_data_.local_input_dim; }
  /**
   * Output dimension per thread.
   */
  int output_dim() const { return meta_data_.output_dim; }
  /**
   * Determines whether the output is accumulated over all threads.
   */
  bool accumulated_output() const { return meta_data_.accumulated_output; }

  CudaFunction() = default;
  CudaFunction(const std::string& function_name, void* lib_handle)
      : function_name(function_name) {
    try {
      fun_ = load_function<FunctionPtrT>(function_name, lib_handle);
      is_available_ = true;
    } catch (const std::runtime_error& ex) {
      is_available_ = false;
    }
    if (is_available_) {
      auto meta_data_fun = load_function<MetaDataFunctionPtrT>(
          function_name + "_meta", lib_handle);
      meta_data_ = meta_data_fun();
      allocate_ = load_function<AllocateFunctionPtrT>(
          function_name + "_allocate", lib_handle);
      deallocate_ = load_function<DeallocateFunctionPtrT>(
          function_name + "_deallocate", lib_handle);
      send_global_fun_ = load_function<SendGlobalFunctionPtrT<Scalar>>(
          function_name + "_send_global", lib_handle);
      send_local_fun_ = load_function<SendLocalFunctionPtrT<Scalar>>(
          function_name + "_send_local", lib_handle);
    }
  }

  inline void allocate(int num_total_threads) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    allocate_(num_total_threads);
  }

  inline void deallocate() const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    deallocate_();
  }

  inline bool operator()(int num_total_threads, int num_blocks,
                         int num_threads_per_block, Scalar* output,
                         const Scalar* input) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(fun_);
    bool status;
    status = send_global_input(input);
    assert(status);
    if (!status) {
      return false;
    }
    status = send_local_input(&(input[meta_data_.global_input_dim]));
    assert(status);
    if (!status) {
      return false;
    }
    fun_(num_total_threads, num_blocks, num_threads_per_block, output);
    return true;
  }

  inline bool operator()(int num_total_threads, Scalar* output,
                         int num_threads_per_block = 32) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(fun_);
    int num_blocks = num_total_threads / num_threads_per_block;
    fun_(num_total_threads, num_blocks, num_threads_per_block, output);
    return true;
  }

  inline bool operator()(std::vector<std::vector<Scalar>>* thread_outputs,
                         const std::vector<std::vector<Scalar>>& local_inputs,
                         int num_threads_per_block = 32,
                         const std::vector<Scalar>& global_input = {}) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(fun_);
    bool status;
    status = send_local_input(local_inputs);
    assert(status);
    if (!status) {
      return false;
    }
    if (!global_input.empty()) {
      status = send_global_input(global_input);
      assert(status);
      if (!status) {
        return false;
      }
    }

    int num_total_threads = static_cast<int>(local_inputs.size());
    Scalar* output = new Scalar[num_total_threads * meta_data_.output_dim];

    int num_blocks = num_total_threads / num_threads_per_block;

    // call GPU kernel
    fun_(num_total_threads, num_blocks, num_threads_per_block, output);

    // assign thread-wise outputs
    std::size_t i = 0;
    if (meta_data_.accumulated_output) {
      assert(thread_outputs->size() >= 1);
      for (; i < output_dim(); ++i) {
        (*thread_outputs)[0][i] = output[i];
      }
    } else {
      assert(thread_outputs->size() >= num_total_threads);
      for (auto& thread : *thread_outputs) {
        for (Scalar& t : thread) {
          t = output[i];
          ++i;
        }
      }
    }

    delete[] output;
    return true;
  }

  inline bool send_local_input(int num_total_threads,
                               const Scalar* input) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_local_fun_);
    return send_local_fun_(num_total_threads, input);
  }
  inline bool send_local_input(
      const std::vector<std::vector<Scalar>>& thread_inputs) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_local_fun_);
    if (thread_inputs.empty() || static_cast<int>(thread_inputs[0].size()) !=
                                     meta_data_.local_input_dim) {
      assert(false);
      return false;
    }
    auto num_total_threads = static_cast<int>(thread_inputs.size());
    Scalar* input = new Scalar[thread_inputs[0].size() * num_total_threads];
    std::size_t i = 0;
    for (const auto& thread : thread_inputs) {
      for (const Scalar& t : thread) {
        input[i] = t;
        ++i;
      }
    }
    bool status = send_local_fun_(num_total_threads, input);
    delete[] input;
    return status;
  }
  inline bool send_local_input(const std::vector<Scalar>& thread_inputs) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_local_fun_);
    auto num_total_threads =
        static_cast<int>(thread_inputs.size() / meta_data_.input_dim);
    return send_local_fun_(num_total_threads, thread_inputs.data());
  }

  inline bool send_global_input(const Scalar* input) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_global_fun_);
    return send_global_fun_(input);
  }

  inline bool send_global_input(const std::vector<Scalar>& input) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_global_fun_);
    if (static_cast<int>(input.size()) != meta_data_.global_input_dim) {
      assert(false);
      return false;
    }
    return send_global_fun_(input.data());
  }

 protected:
  FunctionPtrT fun_{nullptr};
  AllocateFunctionPtrT allocate_{nullptr};
  DeallocateFunctionPtrT deallocate_{nullptr};
  SendGlobalFunctionPtrT<Scalar> send_global_fun_{nullptr};
  SendLocalFunctionPtrT<Scalar> send_local_fun_{nullptr};
};

template <typename Scalar>
struct CudaModel {
 protected:
  const std::string model_name_;
  void* lib_handle_;

  CudaFunctionMetaData meta_data_;

 public:
  CudaFunction<Scalar> forward_zero;
  CudaFunction<Scalar> sparse_jacobian;

#if CPPAD_CG_SYSTEM_WIN
  CudaModel(const std::string& model_name) : model_name_(model_name) {
    // load the dynamic library
    std::string path = model_name + ".dll";
    std::string abs_path;
    bool found = tds::FileUtils::find_file(path, abs_path);
    assert(found);
    lib_handle_ = LoadLibrary(abs_path.c_str());
    if (lib_handle_ == nullptr) {
      throw std::runtime_error("Failed to dynamically load library '" +
                               model_name + "': error code " +
                               std::to_string(GetLastError()));
    }
    forward_zero =
        CudaFunction<Scalar>(model_name + "_forward_zero", lib_handle_);
    sparse_jacobian =
        CudaFunction<Scalar>(model_name + "_jacobian", lib_handle_);
  }
#else
  // loads the shared library
  CudaModel(const std::string& model_name, int dlOpenMode = RTLD_NOW)
      : model_name_(model_name) {
    // load the dynamic library
    std::string path = model_name + ".so";
    std::string abs_path;
    bool found = tds::FileUtils::find_file(path, abs_path);
    assert(found);
    lib_handle_ = dlopen(abs_path.c_str(), dlOpenMode);
    // _dynLibHandle = dlmopen(LM_ID_NEWLM, path.c_str(), RTLD_NOW);
    if (lib_handle_ == nullptr) {
      throw std::runtime_error("Failed to dynamically load library '" +
                               model_name + "': " + std::string(dlerror()));
    }
    forward_zero =
        CudaFunction<Scalar>(model_name + "_forward_zero", lib_handle_);
  }
#endif
};

template <class Base>
class CudaLibraryProcessor {
 protected:
  CudaSourceGen<Base>* cgen_;

  std::string nvcc_path_{"/usr/bin/nvcc"};
  int optimization_level_{0};

  std::vector<std::string> gen_srcs_;
  std::filesystem::path src_dir_;

 public:
  CudaLibraryProcessor(CudaSourceGen<Base>* cgen) : cgen_(cgen) {}

  std::string& nvcc_path() { return nvcc_path_; }
  const std::string& nvcc_path() const { return nvcc_path_; }

  int& optimization_level() { return optimization_level_; }
  const int& optimization_level() const { return optimization_level_; }

  void generate_code() {
    src_dir_ = std::filesystem::path(cgen_->getName() + "_srcs");
    std::filesystem::create_directories(src_dir_);
    gen_srcs_.clear();
    std::string util_name = (src_dir_ / "util.h").string();
    std::ofstream util_file(util_name);
    util_file << util_header_src();
    util_file.close();
    if (cgen_->isCreateForwardZero()) {
      std::string src_name = cgen_->getName() + "_forward_zero.cu";
      // generate CUDA code
      std::string source = cgen_->zero_source();
      std::ofstream cuda_file(src_dir_ / src_name);
      cuda_file << source;
      cuda_file.close();
      std::cout << "Saved forward zero source code at " << src_name << ".\n";
      gen_srcs_.push_back(src_name);
    }
    if (cgen_->isCreateJacobian() || cgen_->isCreateSparseJacobian()) {
      std::string src_name = cgen_->getName() + "_jacobian.cu";
      // generate CUDA code
      std::string source = cgen_->jacobian_source();
      std::ofstream cuda_file(src_dir_ / src_name);
      cuda_file << source;
      cuda_file.close();
      std::cout << "Saved Jacobian source code at " << src_name << ".\n";
      gen_srcs_.push_back(src_name);
    }
    // generate "main" source file
    std::string main_name = (src_dir_ / (cgen_->getName() + ".cu")).string();
    std::ofstream main_file(main_name);
    for (const auto& src : gen_srcs_) {
      main_file << "#include \"" << src << "\"\n";
    }
    main_file.close();
  }

  void create_library() const {
    std::stringstream cmd;
    std::cout << "Compiling CUDA library via " << nvcc_path_ << std::endl;
    cmd << "\"" << nvcc_path_ << "\" ";
    cmd << "--ptxas-options=-O" << std::to_string(optimization_level_) << ",-v "
        << "-rdc=true "
#if CPPAD_CG_SYSTEM_WIN
        << "-o " << cgen_->getName() << ".dll "
#else
        << "--compiler-options "
        << "-fPIC "
        << "-o " << cgen_->getName() << ".so "
#endif
        << "--shared ";
    cmd << (src_dir_ / (cgen_->getName() + ".cu")).string();
    tds::Stopwatch timer;
    timer.start();
    int return_code = std::system(cmd.str().c_str());
    timer.stop();
    std::cout << "CUDA compilation process terminated after " << timer.elapsed()
              << " seconds.\n";
    if (return_code) {
      throw std::runtime_error("CUDA compilation failed with return code " +
                               std::to_string(return_code) + ".");
    }
  }

 protected:
  std::string util_header_src() const {
    std::ostringstream code;
    code << "#ifndef CUDA_UTILS_H\n#define CUDA_UTILS_H\n\n";
    code << "#include <math.h>\n#include <stdio.h>\n\n";

    code << "typedef " << this->cgen_->base_type_name() << " Float;\n\n";

    code << R"(#ifdef _WIN32
#define MODULE_API __declspec(dllexport)
#else
#define MODULE_API
#endif

struct CudaFunctionMetaData {
  int output_dim;
  int local_input_dim;
  int global_input_dim;
  bool accumulated_output;
};

void allocate(void **x, size_t size) {
  cudaError status = cudaMallocHost(x, size);
  if (status != cudaSuccess) {
    fprintf(stderr, "Error %i (%s) while allocating CUDA memory: %s.\n",
            status, cudaGetErrorName(status), cudaGetErrorString(status));
    exit((int)status);
  }
}

#endif  // CUDA_UTILS_H)";
    return code.str();
  }
};

static std::string exec(const char* cmd) {
  std::array<char, 1024> buffer;
  std::string result;
#if CPPAD_CG_SYSTEM_WIN
  std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd, "r"), _pclose);
#else
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
#endif
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());

  return result;
}

}  // namespace tds