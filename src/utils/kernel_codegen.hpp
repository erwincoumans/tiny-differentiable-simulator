#pragma once

// Generates batched CPU and CUDA kernels from TDS functions with
// CppADCodeGen.
//
// Each function maps named input arrays to named output arrays. It is traced
// once with CppAD, so it must not branch on its input values (use the
// tds::where_* functions instead), and the generated code is a single
// straight-line function that is valid for any input. KernelCodegen::write
// creates these files for a model named NAME:
//   NAME_kernels.h          array sizes and constants of the model, and
//                           NAME_kernel_model() that returns the
//                           tds::KernelModel (kernel_model.hpp)
//   NAME_kernels_impl.h     the generated functions as templates on the
//                           floating point type, for the CPU and CUDA
//   NAME_kernels.cpp        the tds::KernelModel table. Define
//                           TDS_KERNELS_CUDA when compiling it with the
//                           CUDA sources.
//   NAME_kernels_cpu_float.cpp, NAME_kernels_cpu_double.cpp
//                           CPU launchers, serial or with OpenMP
//   NAME_kernels_cuda_float.cu, NAME_kernels_cuda_double.cu,
//   NAME_kernels_cuda.cu    CUDA kernels, launchers and runtime functions
// See examples/kernels/g1/g1_codegen.cpp for an example and
// cmake/TdsKernels.cmake to compile the generated files.

#include <cppad/cg.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "kernel_model.hpp"
#include "stopwatch.hpp"

namespace tds {

namespace kernel_codegen {

// Names the independent and dependent variables after the input and output
// arrays of the function: element k of an array is name[k * s].
template <class Base>
class VariableNameGenerator
    : public CppAD::cg::LangCDefaultVariableNameGenerator<Base> {
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  static std::vector<std::string> element_names(
      const std::vector<KernelArray>& arrays) {
    std::vector<std::string> names;
    for (const auto& array : arrays) {
      for (int k = 0; k < array.dim; ++k) {
        names.push_back(array.name + "[" + std::to_string(k) + " * s]");
      }
    }
    return names;
  }

 public:
  VariableNameGenerator(const std::vector<KernelArray>& inputs,
                        const std::vector<KernelArray>& outputs,
                        bool scalar_temporaries)
      : input_names_(element_names(inputs)),
        output_names_(element_names(outputs)) {
    // declare each temporary as a separate variable instead of an array
    this->_temporary[0].array = !scalar_temporaries;
  }

  std::string generateDependent(size_t index) override {
    return output_names_.at(index);
  }

  std::string generateIndependent(
      const CppAD::cg::OperationNode<Base>& independent, size_t id) override {
    return input_names_.at(id - 1);
  }
};

// C code with the floating point type Float, a template argument of the
// generated function, so that the same code compiles for float and double.
template <class Base>
class Language : public CppAD::cg::LanguageC<Base> {
 public:
  Language() : CppAD::cg::LanguageC<Base>("Float") {}

 protected:
  template <class Output>
  void write_float(const Base& value, Output& output) {
    // small integers convert exactly to float and double
    if (value == std::floor(value) && std::abs(value) <= Base(1 << 24)) {
      std::ostringstream os;
      os << static_cast<long long>(value);
      output << os.str();
    } else {
      output << "Float(";
      this->writeParameter(value, output);
      output << ")";
    }
  }

  void printParameter(const Base& value) override {
    write_float(value, this->_code);
  }

  void pushParameter(const Base& value) override {
    write_float(value, this->_streamStack);
  }
};

}  // namespace kernel_codegen

class KernelCodegen {
 public:
  using CGScalar = CppAD::cg::CG<double>;
  using ADScalar = CppAD::AD<CGScalar>;
  // evaluates the function: x are the input arrays, concatenated in the order
  // of the function inputs, y receives the concatenated output arrays
  using TraceFunction =
      std::function<void(const std::vector<ADScalar>& x,
                         std::vector<ADScalar>& y)>;

  struct Options {
    // run the CppAD tape optimizer, which removes unused and duplicate
    // operations
    bool optimize{true};
    // declare each temporary as a separate variable instead of an array
    // element. The compilers can keep them in registers, this is much faster
    // on the GPU.
    bool scalar_temporaries{true};
  };

  explicit KernelCodegen(std::string model_name)
      : model_name_(std::move(model_name)) {}

  Options& options() { return options_; }

  void add_constant(const std::string& name, double value) {
    constants_[name] = value;
  }
  void add_constant(const std::string& name, int value) {
    constants_[name] = value;
    int_constants_.insert(name);
  }
  void add_string(const std::string& name, const std::string& value) {
    strings_[name] = value;
  }

  // Traces the function f at the given input values (concatenated input
  // arrays) and generates its code.
  void add_function(const std::string& name,
                    const std::vector<KernelArray>& inputs,
                    const std::vector<KernelArray>& outputs,
                    const std::vector<double>& input_values,
                    const TraceFunction& f) {
    for (const auto& array : inputs) check_array(array);
    for (const auto& array : outputs) check_array(array);
    const int input_dim = total_dim(inputs);
    const int output_dim = total_dim(outputs);
    if (static_cast<int>(input_values.size()) != input_dim) {
      throw std::runtime_error("KernelCodegen: function \"" + name +
                               "\" expects " + std::to_string(input_dim) +
                               " input values.");
    }

    tds::Stopwatch timer;
    timer.start();
    std::cout << "Tracing " << model_name_ << " " << name << " ("
              << input_dim << " inputs, " << output_dim << " outputs)..."
              << std::endl;
    std::vector<ADScalar> x(input_dim);
    for (int i = 0; i < input_dim; ++i) {
      x[i] = input_values[i];
    }
    CppAD::Independent(x);
    std::vector<ADScalar> y(output_dim);
    f(x, y);
    CppAD::ADFun<CGScalar> tape;
    tape.Dependent(x, y);
    if (options_.optimize) {
      tape.optimize();
    }
    Function function;
    function.name = name;
    function.inputs = inputs;
    function.outputs = outputs;
    function.num_operations = static_cast<int>(tape.size_op());
    std::cout << "  traced " << tape.size_op() << " operations in "
              << timer.elapsed() << " s" << std::endl;

    timer.start();
    CppAD::cg::CodeHandler<double> handler;
    std::vector<CGScalar> independent(input_dim);
    handler.makeVariables(independent);
    std::vector<CGScalar> dependent = tape.Forward(0, independent);

    kernel_codegen::Language<double> language;
    language.setGenerateFunction("");
    kernel_codegen::VariableNameGenerator<double> name_generator(
        inputs, outputs, options_.scalar_temporaries);
    std::ostringstream code;
    handler.generateCode(code, language, dependent, name_generator,
                         model_name_ + "_" + name);
    function.body = code.str();
    function.min_temporary =
        static_cast<int>(name_generator.getMinTemporaryVariableID());
    const int max_temporary =
        static_cast<int>(name_generator.getMaxTemporaryVariableID());
    function.num_temporaries =
        max_temporary >= function.min_temporary
            ? max_temporary + 1 - function.min_temporary
            : 0;
    std::cout << "  generated " << function.num_temporaries
              << " temporaries, " << function.body.size() / 1024
              << " KB of code in " << timer.elapsed() << " s" << std::endl;
    functions_.push_back(function);
  }

  // Writes the generated files to the directory.
  void write(const std::string& directory) const {
    namespace fs = std::filesystem;
    fs::create_directories(directory);
    const fs::path base = fs::path(directory) / (model_name_ + "_kernels");
    write_file(base.string() + ".h", header());
    write_file(base.string() + "_impl.h", impl_header());
    write_file(base.string() + ".cpp", model_source());
    for (const std::string type : {"float", "double"}) {
      write_file(base.string() + "_cpu_" + type + ".cpp", cpu_source(type));
      write_file(base.string() + "_cuda_" + type + ".cu", cuda_source(type));
    }
    write_file(base.string() + "_cuda.cu", cuda_api_source());
  }

 private:
  struct Function {
    std::string name;
    std::vector<KernelArray> inputs;
    std::vector<KernelArray> outputs;
    std::string body;
    int num_operations{0};
    int num_temporaries{0};
    int min_temporary{0};
  };

  std::string model_name_;
  Options options_;
  std::vector<Function> functions_;
  // all arrays by name, arrays with the same name must have the same size
  std::map<std::string, int> arrays_;
  std::map<std::string, double> constants_;
  std::set<std::string> int_constants_;
  std::map<std::string, std::string> strings_;

  static int total_dim(const std::vector<KernelArray>& arrays) {
    int dim = 0;
    for (const auto& array : arrays) dim += array.dim;
    return dim;
  }

  void check_array(const KernelArray& array) {
    auto it = arrays_.find(array.name);
    if (it != arrays_.end() && it->second != array.dim) {
      throw std::runtime_error("KernelCodegen: array \"" + array.name +
                               "\" is used with sizes " +
                               std::to_string(it->second) + " and " +
                               std::to_string(array.dim) + ".");
    }
    arrays_[array.name] = array.dim;
  }

  static void write_file(const std::filesystem::path& path,
                         const std::string& content) {
    std::ofstream file(path);
    if (!file) {
      throw std::runtime_error("KernelCodegen: cannot write " + path.string());
    }
    file << content;
    std::cout << "Wrote " << path.string() << std::endl;
  }

  static std::string generated_comment() {
    return "// Generated by tds::KernelCodegen (src/utils/kernel_codegen.hpp), "
           "do not edit.\n";
  }

  static std::string double_literal(double value) {
    std::ostringstream os;
    os.precision(17);
    os << value;
    std::string s = os.str();
    if (s.find_first_of(".eEn") == std::string::npos) s += ".";
    return s;
  }

  static std::string string_literal(const std::string& value) {
    std::string s = "\"";
    for (char c : value) {
      if (c == '"' || c == '\\') s += '\\';
      s += c;
    }
    return s + "\"";
  }

  // (const Float* TDS_RESTRICT state, ..., Float* TDS_RESTRICT next_state)
  static std::string array_parameters(const Function& f,
                                      const std::string& type) {
    std::string params;
    for (const auto& array : f.inputs) {
      params += ", const " + type + "* TDS_RESTRICT " + array.name;
    }
    for (const auto& array : f.outputs) {
      params += ", " + type + "* TDS_RESTRICT " + array.name;
    }
    return params;
  }

  static std::string array_list(const std::vector<KernelArray>& arrays) {
    std::string list;
    for (const auto& array : arrays) {
      if (!list.empty()) list += ", ";
      list += array.name + "[" + std::to_string(array.dim) + "]";
    }
    return list;
  }

  std::string header() const {
    std::ostringstream code;
    code << generated_comment();
    code << "\n#pragma once\n\n#include \"utils/kernel_model.hpp\"\n\n";
    code << "namespace " << model_name_ << "_kernels {\n\n";
    code << "// array sizes\n";
    for (const auto& [name, dim] : arrays_) {
      code << "constexpr int " << name << "_dim = " << dim << ";\n";
    }
    if (!constants_.empty() || !strings_.empty()) {
      code << "\n// constants of the model\n";
    }
    for (const auto& [name, value] : constants_) {
      if (int_constants_.count(name)) {
        code << "constexpr int " << name << " = " << static_cast<int>(value)
             << ";\n";
      } else {
        code << "constexpr double " << name << " = " << double_literal(value)
             << ";\n";
      }
    }
    for (const auto& [name, value] : strings_) {
      code << "constexpr const char* " << name << " = "
           << string_literal(value) << ";\n";
    }
    code << "\n}  // namespace " << model_name_ << "_kernels\n\n";
    code << "// The generated functions with their CPU and CUDA launchers:\n";
    for (const auto& f : functions_) {
      code << "//   " << f.name << ": " << array_list(f.inputs) << " -> "
           << array_list(f.outputs) << "\n";
    }
    code << "const tds::KernelModel& " << model_name_ << "_kernel_model();\n";
    return code.str();
  }

  std::string impl_header() const {
    std::ostringstream code;
    code << generated_comment();
    code << R"(
// Each function evaluates one world. Element k of an array is at array[k * s],
// the array pointers point to the first element of the world.

#pragma once

#include <cmath>

#include "utils/kernel_model.hpp"

)";
    code << "namespace " << model_name_ << "_kernels {\n\n";
    code << R"(#ifndef __CUDACC__
// float overloads, CUDA provides them for device code
using std::abs;
using std::acos;
using std::asin;
using std::atan;
using std::cos;
using std::cosh;
using std::exp;
using std::fabs;
using std::log;
using std::pow;
using std::sin;
using std::sinh;
using std::sqrt;
using std::tan;
using std::tanh;
#endif

)";
    for (const auto& f : functions_) {
      code << "// " << f.name << ": " << array_list(f.inputs) << " -> "
           << array_list(f.outputs) << "\n";
      code << "// " << f.num_operations << " operations, "
           << f.num_temporaries << " temporaries\n";
      code << "template <typename Float>\n";
      code << "TDS_KERNEL_FUNCTION void " << f.name << "(const int s"
           << array_parameters(f, "Float") << ") {\n";
      if (f.num_temporaries > 0) {
        if (options_.scalar_temporaries) {
          const int per_line = 10;
          for (int i = 0; i < f.num_temporaries; ++i) {
            code << (i % per_line == 0 ? "  Float " : ", ") << "v"
                 << (f.min_temporary + i);
            if (i % per_line == per_line - 1 || i == f.num_temporaries - 1) {
              code << ";\n";
            }
          }
        } else {
          code << "  Float v[" << f.num_temporaries << "];\n";
        }
        code << "\n";
      }
      // indent the generated code
      std::istringstream body(f.body);
      std::string line;
      while (std::getline(body, line)) {
        // the generated code is indented with 3 spaces
        const size_t start = line.find_first_not_of(' ');
        if (start == std::string::npos) continue;
        code << "  " << line.substr(std::min<size_t>(start, 3)) << "\n";
      }
      code << "}\n\n";
    }
    code << "}  // namespace " << model_name_ << "_kernels\n";
    return code.str();
  }

  // call of the generated function for world i, with arrays from the
  // launcher's inputs and outputs pointer arrays
  std::string world_call(const Function& f, const std::string& type) const {
    // qualified, an array can have the same name as a function
    std::string call =
        model_name_ + "_kernels::" + f.name + "<" + type + ">(stride";
    for (size_t a = 0; a < f.inputs.size(); ++a) {
      call += ", inputs[" + std::to_string(a) + "] + i";
    }
    for (size_t a = 0; a < f.outputs.size(); ++a) {
      call += ", outputs[" + std::to_string(a) + "] + i";
    }
    return call + ")";
  }

  static std::string array_initializer(const std::vector<KernelArray>& arrays) {
    std::string list;
    for (const auto& array : arrays) {
      if (!list.empty()) list += ", ";
      list += "{\"" + array.name + "\", " + std::to_string(array.dim) + "}";
    }
    return "{" + list + "}";
  }

  std::string launcher_name(const Function& f, const std::string& backend,
                            const std::string& type) const {
    return model_name_ + "_" + f.name + "_" + backend + "_" + type;
  }

  std::string cpu_launcher(const Function& f, const std::string& type) const {
    return "void " + launcher_name(f, "cpu", type) +
           "(int num_worlds, int stride,\n    const " + type +
           "* const* inputs, " + type +
           "* const* outputs, int num_threads)";
  }

  std::string cuda_launcher(const Function& f, const std::string& type) const {
    return "void " + launcher_name(f, "cuda", type) +
           "(int num_worlds, int stride,\n    const " + type +
           "* const* inputs, " + type +
           "* const* outputs, int block_size, void* stream)";
  }

  std::string model_source() const {
    std::ostringstream code;
    code << generated_comment() << "\n";
    code << "#include \"" << model_name_ << "_kernels.h\"\n\n";
    code << "// defined in " << model_name_ << "_kernels_cpu_float.cpp and "
         << model_name_ << "_kernels_cpu_double.cpp\n";
    for (const auto& f : functions_) {
      for (const std::string type : {"float", "double"}) {
        code << cpu_launcher(f, type) << ";\n";
      }
    }
    code << "\n#ifdef TDS_KERNELS_CUDA\n";
    code << "// defined in " << model_name_ << "_kernels_cuda*.cu\n";
    for (const auto& f : functions_) {
      for (const std::string type : {"float", "double"}) {
        code << cuda_launcher(f, type) << ";\n";
      }
    }
    code << "const tds::KernelCudaApi* " << model_name_
         << "_kernels_cuda_api();\n";
    code << "#endif\n\n";

    code << "const tds::KernelModel& " << model_name_ << "_kernel_model() {\n";
    code << "  static const tds::KernelModel model = [] {\n";
    code << "    tds::KernelModel model;\n";
    code << "    model.name = \"" << model_name_ << "\";\n";
    for (const auto& f : functions_) {
      code << "    {\n";
      code << "      tds::KernelFunction f;\n";
      code << "      f.name = \"" << f.name << "\";\n";
      code << "      f.inputs = " << array_initializer(f.inputs) << ";\n";
      code << "      f.outputs = " << array_initializer(f.outputs) << ";\n";
      code << "      f.num_operations = " << f.num_operations << ";\n";
      code << "      f.num_temporaries = " << f.num_temporaries << ";\n";
      code << "      f.cpu_float = &" << launcher_name(f, "cpu", "float")
           << ";\n";
      code << "      f.cpu_double = &" << launcher_name(f, "cpu", "double")
           << ";\n";
      code << "#ifdef TDS_KERNELS_CUDA\n";
      code << "      f.cuda_float = &" << launcher_name(f, "cuda", "float")
           << ";\n";
      code << "      f.cuda_double = &" << launcher_name(f, "cuda", "double")
           << ";\n";
      code << "#endif\n";
      code << "      model.functions.push_back(f);\n";
      code << "    }\n";
    }
    for (const auto& [name, value] : constants_) {
      code << "    model.constants[\"" << name << "\"] = " << model_name_
           << "_kernels::" << name << ";\n";
    }
    for (const auto& [name, value] : strings_) {
      code << "    model.strings[\"" << name << "\"] = " << model_name_
           << "_kernels::" << name << ";\n";
    }
    code << "#ifdef TDS_KERNELS_CUDA\n";
    code << "    model.cuda = " << model_name_ << "_kernels_cuda_api();\n";
    code << "#endif\n";
    code << "    return model;\n";
    code << "  }();\n";
    code << "  return model;\n";
    code << "}\n";
    return code.str();
  }

  std::string cpu_source(const std::string& type) const {
    std::ostringstream code;
    code << generated_comment() << "\n";
    code << "#include \"" << model_name_ << "_kernels_impl.h\"\n\n";
    for (const auto& f : functions_) {
      code << cpu_launcher(f, type) << " {\n";
      code << "  tds::kernel_parallel_for(num_worlds, num_threads, [&](int i) "
              "{\n";
      code << "    " << world_call(f, type) << ";\n";
      code << "  });\n}\n\n";
    }
    return code.str();
  }

  std::string cuda_source(const std::string& type) const {
    std::ostringstream code;
    code << generated_comment() << "\n";
    code << "#include \"" << model_name_ << "_kernels_impl.h\"\n\n";
    code << "namespace " << model_name_ << "_kernels {\n\n";
    for (const auto& f : functions_) {
      code << "__global__ void " << f.name << "_kernel(int num_worlds, "
           << "int stride" << array_parameters(f, type) << ") {\n";
      code << "  const int i = blockIdx.x * blockDim.x + threadIdx.x;\n";
      code << "  if (i >= num_worlds) return;\n";
      code << "  " << model_name_ << "_kernels::" << f.name << "<" << type
           << ">(stride";
      for (const auto& array : f.inputs) code << ", " << array.name << " + i";
      for (const auto& array : f.outputs) code << ", " << array.name << " + i";
      code << ");\n}\n\n";
    }
    code << "}  // namespace " << model_name_ << "_kernels\n\n";
    for (const auto& f : functions_) {
      code << cuda_launcher(f, type) << " {\n";
      code << "  if (num_worlds <= 0) return;\n";
      code << "  const int num_blocks = (num_worlds + block_size - 1) / "
              "block_size;\n";
      code << "  " << model_name_ << "_kernels::" << f.name
           << "_kernel<<<num_blocks, block_size, 0,\n"
           << "      static_cast<cudaStream_t>(stream)>>>(num_worlds, stride";
      for (size_t a = 0; a < f.inputs.size(); ++a) {
        code << ", inputs[" << a << "]";
      }
      for (size_t a = 0; a < f.outputs.size(); ++a) {
        code << ", outputs[" << a << "]";
      }
      code << ");\n}\n\n";
    }
    return code.str();
  }

  std::string cuda_api_source() const {
    std::ostringstream code;
    code << generated_comment() << "\n";
    code << "#include \"utils/kernel_cuda_api.cuh\"\n\n";
    code << "const tds::KernelCudaApi* " << model_name_
         << "_kernels_cuda_api() { return tds::kernel_cuda_api(); }\n";
    return code.str();
  }
};

}  // namespace tds
