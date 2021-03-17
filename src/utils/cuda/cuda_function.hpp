#pragma once

namespace tds {
struct CudaFunctionMetaData {
  int output_dim;
  int local_input_dim;
  int global_input_dim;
  bool accumulated_output;
};

template <typename Scalar>
using DefaultCudaFunctionPtrT = void (*)(int, int, int, Scalar *);
template <typename Scalar>
using SendLocalFunctionPtrT = bool (*)(int, const Scalar *);
template <typename Scalar>
using SendGlobalFunctionPtrT = bool (*)(const Scalar *);

using MetaDataFunctionPtrT = CudaFunctionMetaData (*)();
using AllocateFunctionPtrT = void (*)(int);
using DeallocateFunctionPtrT = void (*)();

template <typename Scalar,
          typename kFunctionPtrT = DefaultCudaFunctionPtrT<Scalar>>
struct CudaFunction {
  template<typename OtherScalar>
  friend class CudaLibrary;

  using FunctionPtrT = kFunctionPtrT;
  std::string function_name;

protected:
  CudaFunctionMetaData meta_data_{};
  bool is_available_{false};

  template <typename FunctionPtrT>
  static FunctionPtrT load_function(const std::string &function_name,
                                    void *lib_handle) {
#if CPPAD_CG_SYSTEM_WIN
    auto ptr = (FunctionPtrT)GetProcAddress((HMODULE)lib_handle,
                                            function_name.c_str());
    if (!ptr) {
      throw std::runtime_error("Cannot load symbol '" + function_name +
                               "': error code " +
                               std::to_string(GetLastError()));
    }
#else
    auto ptr = (FunctionPtrT)dlsym(lib_handle, function_name.c_str());
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
      throw std::runtime_error("Cannot load symbol '" + function_name +
                               "': " + std::string(dlsym_error));
    }
#endif
    return ptr;
  }

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
  CudaFunction(const std::string &function_name, void *lib_handle)
      : function_name(function_name) {
    try {
      fun_ = load_function<FunctionPtrT>(function_name, lib_handle);
      is_available_ = true;
    } catch (const std::runtime_error &ex) {
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
                         int num_threads_per_block, Scalar *output,
                         const Scalar *input) const {
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

  inline bool operator()(int num_total_threads, Scalar *output,
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

  inline bool operator()(std::vector<std::vector<Scalar>> *thread_outputs,
                         const std::vector<std::vector<Scalar>> &local_inputs,
                         int num_threads_per_block = 32,
                         const std::vector<Scalar> &global_input = {}) const {
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
    Scalar *output = new Scalar[num_total_threads * meta_data_.output_dim];

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
      for (auto &thread : *thread_outputs) {
        for (Scalar &t : thread) {
          t = output[i];
          ++i;
        }
      }
    }

    delete[] output;
    return true;
  }

  inline bool send_local_input(int num_total_threads,
                               const Scalar *input) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_local_fun_);
    return send_local_fun_(num_total_threads, input);
  }
  inline bool send_local_input(
      const std::vector<std::vector<Scalar>> &thread_inputs) const {
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
    Scalar *input = new Scalar[thread_inputs[0].size() * num_total_threads];
    std::size_t i = 0;
    for (const auto &thread : thread_inputs) {
      for (const Scalar &t : thread) {
        input[i] = t;
        ++i;
      }
    }
    bool status = send_local_fun_(num_total_threads, input);
    delete[] input;
    return status;
  }
  inline bool send_local_input(const std::vector<Scalar> &thread_inputs) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_local_fun_);
    auto num_total_threads =
        static_cast<int>(thread_inputs.size() / meta_data_.local_input_dim);
    return send_local_fun_(num_total_threads, thread_inputs.data());
  }

  inline bool send_global_input(const Scalar *input) const {
    if (!is_available_) {
      throw std::runtime_error("Function \"" + function_name +
                               "\" is not available.");
    }
    assert(send_global_fun_);
    return send_global_fun_(input);
  }

  inline bool send_global_input(const std::vector<Scalar> &input) const {
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
} // namespace tds