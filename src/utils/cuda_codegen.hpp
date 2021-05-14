#pragma once

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
  std::size_t global_dim_{0};
  // name of thread-local input
  std::string local_name_;

 public:
  inline explicit CudaVariableNameGenerator(
      std::size_t global_dim, std::string depName = "y",
      std::string indepName = "x", std::string localName = "xj",
      std::string tmpName = "v", std::string tmpArrayName = "array",
      std::string tmpSparseArrayName = "sarray")
      : CppAD::cg::LangCDefaultVariableNameGenerator<Base>(
            depName, indepName, tmpName, tmpArrayName, tmpSparseArrayName),
        global_dim_(global_dim),
        local_name_(std::move(localName)) {}

  inline std::string generateIndependent(
      const CppAD::cg::OperationNode<Base>& independent, size_t id) override {
    this->_ss.clear();
    this->_ss.str("");

    if (id - 1 >= global_dim_) {
      // global inputs access directly independent vars starting from index 0
      this->_ss << this->local_name_ << "[" << (id - 1 - global_dim_) << "]";
    } else {
      // thread-local inputs use 'xj' (offset of input 'x')
      this->_ss << this->_indepName << "[" << (id - 1) << "]";
    }

    return this->_ss.str();
  }
};

struct CudaFunctionMetaData {
  int output_dim;
  int input_dim;
  int global_dim;
};

template <class Base>
class CudaSourceGen : public CppAD::cg::ModelCSourceGen<Base> {
  using CGBase = CppAD::cg::CG<Base>;

  std::size_t global_dim_{0};

 public:
  CudaSourceGen(CppAD::ADFun<CppAD::cg::CG<Base>>& fun, std::string model)
      : CppAD::cg::ModelCSourceGen<Base>(fun, model) {}

  void set_global_dim(std::size_t global_dim) { global_dim_ = global_dim; }

  const std::map<std::string, std::string>& sources() {
    auto mtt = CppAD::cg::MultiThreadingType::NONE;
    CppAD::cg::JobTimer* timer = nullptr;
    return this->getSources(mtt, timer);
  }

  /**
   * Generate CUDA library code for the forward zero pass.
   */
  std::string zero_source() {
    const std::string jobName = "model (zero-order forward)";

    this->startingJob("'" + jobName + "'", CppAD::cg::JobTimer::GRAPH);

    CppAD::cg::CodeHandler<Base> handler;
    handler.setJobTimer(this->_jobTimer);

    if (global_dim_ > this->_fun.Domain()) {
      std::cerr << "CUDA codegen failed: global data input size must not be "
                   "larger than the provided input vector size.\n";
      std::exit(1);
    }

    const std::size_t input_dim = this->_fun.Domain() - global_dim_;
    const std::size_t output_dim = this->_fun.Range();

    std::cout << "Generating code for function with input dimension "
              << input_dim << " and output dimension " << output_dim << "...\n";

    std::vector<CGBase> indVars(input_dim + global_dim_);
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
    CudaVariableNameGenerator<Base> nameGen(global_dim_);

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

    std::ostringstream complete;

    complete << "#include <math.h>\n#include <stdio.h>\n\n";
    complete << R"(#ifdef _WIN32
#define MODULE_API __declspec(dllexport)
#else
#define MODULE_API
#endif)";

    complete << "\n\ntypedef " << this->_baseTypeName << " Float;\n\n";

    complete << R"(struct CudaFunctionMetaData {
  int output_dim;
  int input_dim;
  int global_dim;
};)";
    std::string function_name = std::string(this->_name) + "_forward_zero";

    // meta data retrieval function
    complete << "\n\nextern \"C\" {\nMODULE_API CudaFunctionMetaData "
             << function_name << "_meta() {\n";
    complete << "  CudaFunctionMetaData data;\n";
    complete << "  data.output_dim = " << output_dim << ";\n";
    complete << "  data.input_dim = " << input_dim << ";\n";
    complete << "  data.global_dim = " << global_dim_ << ";\n";
    complete << "  return data;\n}\n}\n";

    // CUDA kernel
    complete << "\n__global__\n";
    std::string kernel_name = function_name + "_kernel";
    std::string fun_head_start = "void " + kernel_name + "(";
    std::string fun_arg_pad = std::string(fun_head_start.size(), ' ');
    complete << fun_head_start;
    complete << "int num_total_threads,\n";
    complete << fun_arg_pad << "Float *output,\n";
    complete << fun_arg_pad << "const Float *input) {\n";
    complete << "   const int i = blockIdx.x * blockDim.x + threadIdx.x;\n";
    complete << "   if (i >= num_total_threads) return;\n";
    complete << "   const int j = " << global_dim_ << " + i * " << (input_dim)
             << ";  // thread-local input index offset\n\n";
    if (temporary_dim > 0) {
      complete << "   Float v[" << temporary_dim << "];\n";
    }
    if (global_dim_ > 0) {
      complete << "   const Float *x = &(input[0]);\n";
    }
    complete << "   const Float *xj = &(input[j]);\n";
    complete << "   Float *y = &(output[i * " << output_dim << "]);\n";

    complete << "\n";

    complete << code.str();

    complete << "}\n\n";

    // allocation helper function
    complete << R"(void allocate(void **x, size_t size) {
  cudaError status = cudaMallocHost(x, size);
  if (status != cudaSuccess) {
    fprintf(stderr, "Error %i (%s) while allocating CUDA memory: %s.\n",
    status, cudaGetErrorName(status), cudaGetErrorString(status));
    exit((int)status);
  }
})";
    complete << "\n\n";

    // global device memory pointers
    complete << "Float* dev_output = nullptr;\n";
    complete << "Float* dev_input = nullptr;\n\n";

    // allocation function
    complete << "extern \"C\" {\nMODULE_API void " << function_name
             << "_allocate(int num_total_threads) {\n";
    complete << "  const size_t output_dim = num_total_threads * " << output_dim
             << ";\n";
    complete << "  const size_t input_dim = num_total_threads * " << input_dim
             << " + " << global_dim_ << ";\n\n";
    complete
        << "  allocate((void**)&dev_output, output_dim * sizeof(Float));\n";
    complete << "  allocate((void**)&dev_input, input_dim * sizeof(Float));\n";
    complete << "}\n\n";

    // deallocation function
    complete << "MODULE_API void " << function_name << "_deallocate() {\n";
    complete << "  cudaFreeHost(dev_output);\n";
    complete << "  cudaFreeHost(dev_input);\n";
    complete << "  // cudaDeviceReset();\n";
    complete << "}\n\n";

    // kernel launch function
    fun_head_start = "MODULE_API void " + function_name + "(";
    fun_arg_pad = std::string(fun_head_start.size(), ' ');
    complete << fun_head_start;
    complete << "int num_total_threads,\n";
    complete << fun_arg_pad << "int num_blocks,\n";
    complete << fun_arg_pad << "int num_threads_per_block,\n";
    complete << fun_arg_pad << "Float *output,\n";
    complete << fun_arg_pad << "const Float *input) {\n";

    complete << "  const size_t output_dim = num_total_threads * " << output_dim
             << ";\n";
    complete << "  const size_t input_dim = num_total_threads * " << input_dim
             << " + " << global_dim_ << ";\n";

    complete << R"(
  // Copy input vector from host memory to GPU buffers.
  cudaMemcpy(dev_input, input, input_dim * sizeof(Float), cudaMemcpyHostToDevice);
  cudaMemcpy(dev_output, output, output_dim * sizeof(Float), cudaMemcpyHostToDevice);

  )";
    complete << kernel_name;
    complete
        << R"(<<<num_blocks, num_threads_per_block>>>(num_total_threads, dev_output, dev_input);
  
  // cudaDeviceSynchronize waits for the kernel to finish, and returns
  // any errors encountered during the launch.
  cudaDeviceSynchronize();

  // Copy output vector from GPU buffer to host memory.
  cudaMemcpy(output, dev_output, output_dim * sizeof(Float), cudaMemcpyDeviceToHost);)";

    complete << "\n}\n}\n";

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
using DefaultCudaFunctionPtrT = void (*)(int, int, int, Scalar*, const Scalar*);
using MetaDataFunctionPtrT = CudaFunctionMetaData (*)();
using AllocateFunctionPtrT = void (*)(int);
using DeallocateFunctionPtrT = void (*)();

template <typename Scalar,
          typename kFunctionPtrT = DefaultCudaFunctionPtrT<Scalar>>
struct CudaFunction {
  using FunctionPtrT = kFunctionPtrT;

  std::string function_name;

  CudaFunctionMetaData meta_data;

  CudaFunction() = default;

  CudaFunction(const std::string& function_name, void* lib_handle)
      : function_name(function_name) {
    fun_ = load_function<FunctionPtrT>(function_name, lib_handle);
    auto meta_data_fun = load_function<MetaDataFunctionPtrT>(
        function_name + "_meta", lib_handle);
    meta_data = meta_data_fun();
    allocate_ = load_function<AllocateFunctionPtrT>(function_name + "_allocate",
                                                    lib_handle);
    deallocate_ = load_function<DeallocateFunctionPtrT>(
        function_name + "_deallocate", lib_handle);
  }

  inline void allocate(int num_total_threads) const {
    allocate_(num_total_threads);
  }

  inline void deallocate() const { deallocate_(); }

  inline bool operator()(int num_total_threads, int num_blocks,
                         int num_threads_per_block, Scalar* output,
                         const Scalar* input) const {
    assert(fun_);
    fun_(num_total_threads, num_blocks, num_threads_per_block, output, input);
    return true;
  }

  inline bool operator()(std::vector<std::vector<Scalar>>* thread_outputs,
                         const std::vector<std::vector<Scalar>>& thread_inputs,
                         int num_threads_per_block = 32,
                         const std::vector<Scalar>& global_input = {}) const {
    assert(fun_);
    if (thread_outputs == nullptr || thread_outputs->empty() ||
        (*thread_outputs)[0].empty()) {
      assert(false);
      return false;
    }
    if (thread_outputs->size() != thread_inputs.size()) {
      assert(false);
      return false;
    }
    if (static_cast<int>(thread_inputs[0].size()) != meta_data.input_dim) {
      assert(false);
      return false;
    }
    if (static_cast<int>((*thread_outputs)[0].size()) != meta_data.output_dim) {
      assert(false);
      return false;
    }
    if (static_cast<int>(global_input.size()) != meta_data.global_dim) {
      assert(false);
      return false;
    }

    auto num_total_threads = static_cast<int>(thread_inputs.size());
    // concatenate thread-wise inputs and global memory into contiguous input
    // array
    Scalar* input = new Scalar[global_input.size() +
                               thread_inputs[0].size() * num_total_threads];
    std::size_t i = 0;
    for (; i < global_input.size(); ++i) {
      input[i] = global_input[i];
    }
    for (const auto& thread : thread_inputs) {
      for (const Scalar& t : thread) {
        input[i] = t;
        ++i;
      }
    }
    Scalar* output =
        new Scalar[(*thread_outputs)[0].size() * num_total_threads];

    int num_blocks = ceil(num_total_threads * 1. / num_threads_per_block);

    // call GPU kernel
    fun_(num_total_threads, num_blocks, num_threads_per_block, output, input);

    // assign thread-wise outputs
    i = 0;
    for (auto& thread : *thread_outputs) {
      for (Scalar& t : thread) {
        t = output[i];
        ++i;
      }
    }

    delete[] input;
    delete[] output;
    return true;
  }

 protected:
  FunctionPtrT fun_{nullptr};
  AllocateFunctionPtrT allocate_{nullptr};
  DeallocateFunctionPtrT deallocate_{nullptr};
};

template <typename Scalar>
struct CudaModel {
 protected:
  const std::string model_name_;
  void* lib_handle_;

  CudaFunctionMetaData meta_data_;

 public:
  CudaFunction<Scalar> forward_zero;

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

  std::string forward_zero_src_name_;

  std::string nvcc_path_{"/usr/bin/nvcc"};
  int optimization_level_{3};

 public:
  CudaLibraryProcessor(CudaSourceGen<Base>* cgen) : cgen_(cgen) {}

  std::string& nvcc_path() { return nvcc_path_; }
  const std::string& nvcc_path() const { return nvcc_path_; }

  int& optimization_level() { return optimization_level_; }
  const int& optimization_level() const { return optimization_level_; }

  void generate_code() {
    std::filesystem::path src_dir(cgen_->getName() + "_srcs");
    std::filesystem::create_directories(src_dir);

    forward_zero_src_name_ =
        (src_dir / (cgen_->getName() + "_forward_zero.cu")).string();

    // generate CUDA code
    std::string source_zero = cgen_->zero_source();
    // std::cout << "Zero source:\n" << source_zero << std::endl;
    std::ofstream cuda_file(forward_zero_src_name_);
    cuda_file << source_zero;
    cuda_file.close();
    std::cout << "Saved forward zero source code at " << forward_zero_src_name_
              << ".\n";
  }

  void create_library() const {
    std::stringstream cmd;
    std::cout << "Compiling CUDA library via " << nvcc_path_ << std::endl;
    cmd << "\"" << nvcc_path_ << "\" ";
    cmd << "--ptxas-options=-O" << std::to_string(optimization_level_) << ",-v "
#if CPPAD_CG_SYSTEM_WIN
        << "-o " << cgen_->getName() << ".dll "
#else
        << "--compiler-options "
        << "-fPIC "
        << "-o " << cgen_->getName() << ".so "
#endif
        << "--shared " << forward_zero_src_name_;

    std::cout << cmd.str() << std::endl;
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