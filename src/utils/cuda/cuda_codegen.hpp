#pragma once

#include <cppad/cg.hpp>
#include <cppad/cg/arithmetic.hpp>
#include <cppad/cg/support/cppadcg_eigen.hpp>
#include <numeric>

#include "cuda_language.hpp"

namespace tds {
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

  CudaFunctionSourceGen(const std::string &function_name, int local_input_dim,
                        int global_input_dim, int output_dim,
                        CudaAccumulationMethod acc_method)
      : function_name(function_name), local_input_dim(local_input_dim),
        global_input_dim(global_input_dim), output_dim(output_dim),
        acc_method(acc_method) {}

  void emit_header(std::ostringstream &code) const {
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

  template <typename Base>
  void emit_kernel(std::ostringstream &code, const std::ostringstream &body,
                   LanguageCuda<Base> &language,
                   bool is_function = false) const {
    std::string kernel_name = function_name;
    if (is_function) {
      code << "__device__\n";
    } else {
      code << "\n__global__\n";
      kernel_name += "_kernel";
    }
    std::string fun_head_start = "void " + kernel_name + "(";
    std::string fun_arg_pad = std::string(fun_head_start.size(), ' ');
    code << fun_head_start;
    if (!is_function) {
      code << "int num_total_threads,\n";
      code << fun_arg_pad;
    }
    code << "Float *output,\n";
    code << fun_arg_pad << "const Float *local_input";
    if (global_input_dim > 0) {
      code << ",\n" << fun_arg_pad << "const Float *global_input";
    }
    code << ") {\n";
    if (!is_function) {
      code << "  const int ti = blockIdx.x * blockDim.x + threadIdx.x;\n";
      code << "  if (ti >= num_total_threads) {\n";
      code << "    printf(\"ERROR: thread index %i exceeded provided "
              "number of total threads %i.\\n\", ti, num_total_threads);\n";
      code << "    return;\n  }\n";
    }
    code << "\n";
    if (global_input_dim > 0) {
      code << "  const Float *x = &(global_input[0]);  // global input\n";
    }
    if (!is_function) {
      code << "  const Float *xj = &(local_input[ti * " << local_input_dim
           << "]);  // thread-local input\n";
      code << "  Float *y = &(output[ti * " << output_dim << "]);\n";
    } else {
      code << "  const Float *xj = &(local_input[0]);  // thread-local "
              "input\n";
      code << "  Float *y = &(output[0]);\n";
    }

    auto &info = language.getInfo();
    code << language.generateTemporaryVariableDeclaration(
        false, false, info->atomicFunctionsMaxForward,
        info->atomicFunctionsMaxReverse);

    code << "\n";

    code << body.str();

    code << "}\n\n";
  }

  void emit_allocation_functions(std::ostringstream &code) const {
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

  void emit_send_functions(std::ostringstream &code) const {
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

  void emit_kernel_launch(std::ostringstream &code) const {
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
      code << "   for (int j = 0; j < " << output_dim << "; ++j) {\n";
      code << "     output[j] += output[i*" << output_dim << " + j];\n";
      code << "   }\n  }\n";
      if (acc_method == CUDA_ACCUMULATE_MEAN) {
        code << "  for (int j = 0; j < " << output_dim << "; ++j) {\n";
        code << "   output[j] /= num_total_threads;\n  }";
      }
    }

    code << "\n}\n}\n";
  }
};
} // namespace

template <class Base>
class CudaModelSourceGen : public CppAD::cg::ModelCSourceGen<Base> {
  using CGBase = CppAD::cg::CG<Base>;

protected:
  std::size_t global_input_dim_{0};

  CudaAccumulationMethod jac_acc_method_{CUDA_ACCUMULATE_MEAN};

  std::vector<std::size_t> jac_local_input_sparsity_;
  std::vector<std::size_t> jac_global_input_sparsity_;
  std::vector<std::size_t> jac_output_sparsity_;

  /**
   * Whether to only generate the CUDA kernel, not the kernel launch and memory
   * access functions.
   */
  bool kernel_only_{false};

public:
  CudaModelSourceGen(CppAD::ADFun<CppAD::cg::CG<Base>> &fun, std::string model,
                     bool kernel_only = false)
      : CppAD::cg::ModelCSourceGen<Base>(fun, model),
        kernel_only_(kernel_only) {}

  std::size_t &global_input_dim() { return global_input_dim_; }
  const std::size_t &global_input_dim() const { return global_input_dim_; }

  std::size_t local_input_dim() const {
    return this->_fun.Domain() - global_input_dim_;
  }
  std::size_t output_dim() const { return this->_fun.Range(); }

  std::string base_type_name() const { return this->_baseTypeName; }

  bool is_kernel_only() const { return kernel_only_; }
  void set_kernel_only(bool option) { kernel_only_ = option; }

  CudaAccumulationMethod &jacobian_acc_method() { return jac_acc_method_; }
  const CudaAccumulationMethod &jacobian_acc_method() const {
    return jac_acc_method_;
  }

  void set_jac_local_input_sparsity(const std::vector<std::size_t> &sparsity) {
    for (auto idx : sparsity) {
      assert(idx < local_input_dim());
    }
    jac_local_input_sparsity_ = sparsity;
  }
  void set_jac_global_input_sparsity(const std::vector<std::size_t> &sparsity) {
    for (auto idx : sparsity) {
      assert(idx < global_input_dim());
    }
    jac_global_input_sparsity_ = sparsity;
  }
  void set_jac_output_sparsity(const std::vector<std::size_t> &sparsity) {
    for (auto idx : sparsity) {
      assert(idx < output_dim());
    }
    jac_output_sparsity_ = sparsity;
  }

  const std::map<std::string, std::string> &sources() {
    auto mtt = CppAD::cg::MultiThreadingType::NONE;
    CppAD::cg::JobTimer *timer = nullptr;
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
      jac_output_sparsity_.resize(output_dim());
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
    std::size_t n = this->_fun.Domain();

    this->startingJob("'" + jobName + "'", CppAD::cg::JobTimer::GRAPH);

    CppAD::cg::CodeHandler<Base> handler;
    handler.setJobTimer(this->_jobTimer);

    std::vector<CGBase> indVars(n);
    handler.makeVariables(indVars);
    if (this->_x.size() > 0) {
      for (size_t i = 0; i < n; i++) {
        indVars[i].setValue(this->_x[i]);
      }
    }


#ifdef USE_JACOBIAN
    auto jac = this->_fun.Jacobian(indVars);
#else //USE_JACOBIAN
    std::vector<CGBase> jac(this->_jacSparsity.rows.size());
    bool forward = local_input_dim() + global_input_dim() <= output_dim();
    if (this->_loopTapes.empty()) {
      // printSparsityPattern(this->_jacSparsity.sparsity, "jac sparsity");
      CppAD::sparse_jacobian_work work;

      if (forward) {
        this->_fun.SparseJacobianForward(indVars, this->_jacSparsity.sparsity,
                                         this->_jacSparsity.rows,
                                         this->_jacSparsity.cols, jac, work);
      } else {
        this->_fun.SparseJacobianReverse(indVars, this->_jacSparsity.sparsity,
                                         this->_jacSparsity.rows,
                                         this->_jacSparsity.cols, jac, work);
      }
    } else {
      jac = this->prepareSparseJacobianWithLoops(handler, indVars, forward);
    }
#endif //USE_JACOBIAN
    this->finishedJob();

    LanguageCuda<Base> langC;
    langC.setMaxAssignmentsPerFunction(this->_maxAssignPerFunc,
                                       &this->_sources);
    langC.setMaxOperationsPerAssignment(this->_maxOperationsPerAssignment);
    langC.setParameterPrecision(this->_parameterPrecision);
    langC.setGenerateFunction(""); // _name + "_" + FUNCTION_SPARSE_JACOBIAN

    std::ostringstream code;

    CudaVariableNameGenerator<Base> nameGen(global_input_dim_);

    // size_t arraySize = nameGen.getMaxTemporaryArrayVariableID();
    // size_t sArraySize = nameGen.getMaxTemporarySparseArrayVariableID();
    // if (arraySize > 0 || sArraySize > 0) {
    //   code << "  Float* " << langC.auxArrayName_ << ";\n";
    // }

    // if (arraySize > 0 || sArraySize > 0 || zeroDependentArray) {
    //   _ss << _spaces << U_INDEX_TYPE << " i;\n";
    // }

    handler.generateCode(code, langC, jac, nameGen, this->_atomicFunctions,
                         jobName);

    std::size_t temporary_dim = nameGen.getMaxTemporaryVariableID() + 1 -
                                nameGen.getMinTemporaryVariableID();
    if (temporary_dim == 0) {
      std::cerr << "Warning: the generated code has no temporary variables.\n";
    } else {
      std::cout << "Info: the generated code has " << temporary_dim
                << " temporary variables.\n";
    }

    std::ostringstream complete;

    CudaFunctionSourceGen generator(std::string(this->_name) + "_jacobian",
                                    local_input_dim(), global_input_dim_,
                                    rows.size(), jac_acc_method_);

    if (!kernel_only_) {
      generator.emit_header(complete);
    }
    generator.emit_kernel(complete, code, langC, kernel_only_);
    if (!kernel_only_) {
      generator.emit_allocation_functions(complete);
      generator.emit_send_functions(complete);
      generator.emit_kernel_launch(complete);
    }

    return complete.str();
  }

  std::string
  jacobian_source(const std::vector<std::size_t> &local_indices,
                  const std::vector<std::size_t> &global_indices,
                  CudaAccumulationMethod acc_method = CUDA_ACCUMULATE_MEAN) {
    const std::size_t output_dim = this->_fun.Range();
    std::vector<size_t> output_indices(output_dim, 0);
    std::iota(output_indices.begin(), output_indices.end(), 0);
    return jacobian_source(local_indices, global_indices, output_indices,
                           acc_method);
  }

  std::string
  jacobian_source(const std::vector<std::size_t> &global_indices,
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

    LanguageCuda<Base> langC;
    langC.setMaxAssignmentsPerFunction(this->_maxAssignPerFunc,
                                       &this->_sources);
    langC.setMaxOperationsPerAssignment(this->_maxOperationsPerAssignment);
    langC.setParameterPrecision(this->_parameterPrecision);
    // set function name to empty string so that only the body gets generated
    langC.setGenerateFunction("");
    // langC.setGenerateFunction(this->_name + "_forward_zero");

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

    if (!kernel_only_) {
      generator.emit_header(complete);
    }
    generator.emit_kernel(complete, code, langC, kernel_only_);
    if (!kernel_only_) {
      generator.emit_allocation_functions(complete);
      generator.emit_send_functions(complete);
      generator.emit_kernel_launch(complete);
    }

    return complete.str();
  }
};

} // namespace tds