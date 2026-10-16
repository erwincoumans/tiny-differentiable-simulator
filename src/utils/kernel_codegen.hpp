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
//
// KernelCodegen::write_runtime_sources writes self-contained sources instead,
// for kernels that are compiled and loaded at run time (kernel_loader.hpp):
//   kernels_cpu.cpp         the functions with C launchers, compiled into a
//                           shared library per precision
//   kernels_cuda.cu         CUDA kernels for NVRTC, compiled per precision
//   kernels.json            the functions, arrays and constants
// Compile them with -DTDS_KERNELS_FLOAT or -DTDS_KERNELS_DOUBLE.

#include <cppad/cg.hpp>

#include <algorithm>
#include <cctype>
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
    // For the run time sources: split functions with more generated code
    // than this (in KB) into segments that are compiled separately, in
    // parallel. The compile time of the CUDA kernels grows quickly with the
    // size of a function. The temporaries that are live between segments are
    // stored in a scratch array. 0 does not split.
    int max_segment_kb{256};
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

  // Adds a loop function: its single output is evaluated from the inputs in
  // a loop of iterations (a run time argument of the kernels, see
  // KernelBatch::launch), where the input array loop_input, of the size of
  // the output, is the initial value in the first iteration and the previous
  // output afterwards. Such as the iterations of a projected Gauss-Seidel
  // solver. Only the code of one iteration is generated, and only for the
  // run time sources (write_runtime_sources).
  void add_loop_function(const std::string& name,
                         const std::vector<KernelArray>& inputs,
                         const KernelArray& output,
                         const std::string& loop_input,
                         int default_iterations,
                         const std::vector<double>& input_values,
                         const TraceFunction& f) {
    bool found = false;
    for (const auto& array : inputs) {
      if (array.name == loop_input) {
        found = true;
        if (array.dim != output.dim) {
          throw std::runtime_error("KernelCodegen: the loop input \"" +
                                   loop_input +
                                   "\" must have the size of the output.");
        }
      }
    }
    if (!found) {
      throw std::runtime_error("KernelCodegen: function \"" + name +
                               "\" has no input \"" + loop_input + "\".");
    }
    add_function(name, inputs, {output}, input_values, f);
    functions_.back().loop_input = loop_input;
    functions_.back().default_iterations = default_iterations;
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

  // Additional description of the model, a JSON value that is written to
  // kernels.json as "metadata", such as the layout of the state array.
  void set_metadata_json(const std::string& json) { metadata_json_ = json; }

  // Writes the sources for kernels that are compiled at run time, see
  // kernel_loader.hpp: kernels_cpu.cpp, kernels_cuda.cu and kernels.json.
  void write_runtime_sources(const std::string& directory) const {
    namespace fs = std::filesystem;
    fs::create_directories(directory);
    const fs::path dir(directory);
    write_file(dir / "kernels_cpu.cpp", runtime_cpu_source());
    write_file(dir / "kernels_cuda.cu", runtime_cuda_source());
    write_file(dir / "kernels.json", manifest_json());
  }

  // name of the C function of the CPU launcher or CUDA kernel of segment k
  // of a function
  static std::string runtime_symbol(const std::string& function_name, int k,
                                    const std::string& type) {
    return "tds_kernel_" + function_name + "_" + std::to_string(k) + "_" +
           type;
  }

  // C++ source of the CPU kernels. Each function is a separately compiled
  // part: compile with -DTDS_KERNEL_PART=p for part p (see "cpu_parts" in
  // kernels.json, all parts without it), and -DTDS_KERNELS_FLOAT or
  // -DTDS_KERNELS_DOUBLE. For function NAME it exports
  //   void tds_kernel_NAME_0_float(int begin, int end, int stride,
  //       const float* const* inputs, float* const* outputs,
  //       float* scratch, int iterations)
  // that evaluates worlds [begin, end). The scratch array has the stride of
  // the other arrays, see KernelFunction::scratch_dim.
  std::string runtime_cpu_source() const {
    std::ostringstream code;
    code << generated_comment();
    code << R"(
// CPU kernels of the model. Compile with -DTDS_KERNELS_FLOAT or
// -DTDS_KERNELS_DOUBLE, and -DTDS_KERNEL_PART=p to compile only part p.

#include <cmath>

#if defined(_MSC_VER)
#define TDS_RESTRICT __restrict
#define TDS_KERNEL_EXPORT extern "C" __declspec(dllexport)
#else
#define TDS_RESTRICT __restrict__
#define TDS_KERNEL_EXPORT extern "C" __attribute__((visibility("default")))
#endif
#define TDS_KERNEL_FUNCTION static inline

)";
    code << math_using_declarations();
    const auto all_parts = parts(false);
    for (size_t p = 0; p < all_parts.size(); ++p) {
      const Function& f = functions_[all_parts[p].function];
      const int k = all_parts[p].segment;
      code << "#if !defined(TDS_KERNEL_PART) || TDS_KERNEL_PART == " << p
           << "\n\n";
      code << segment_definition(f, k, false);
      for (const std::string type : {"float", "double"}) {
        code << "#ifdef TDS_KERNELS_" << (type == "float" ? "FLOAT" : "DOUBLE")
             << "\n";
        code << "TDS_KERNEL_EXPORT void " << runtime_symbol(f.name, k, type)
             << "(int begin, int end, int stride,\n    const " << type
             << "* const* inputs, " << type << "* const* outputs, " << type
             << "* scratch, int iterations) {\n";
        code << "  for (int i = begin; i < end; ++i) {\n";
        std::vector<std::string> in, out;
        for (size_t a = 0; a < f.inputs.size(); ++a) {
          in.push_back("inputs[" + std::to_string(a) + "] + i");
        }
        for (size_t a = 0; a < f.outputs.size(); ++a) {
          out.push_back("outputs[" + std::to_string(a) + "] + i");
        }
        code << world_body(f, k, in, out, "scratch + i", type, "    ");
        code << "  }\n}\n#endif\n";
      }
      code << "\n#endif  // part " << p << "\n\n";
    }
    return code.str();
  }

  // CUDA source of the kernels for NVRTC, without includes. Functions with
  // much code are split into segments (Options::max_segment_kb). Each segment
  // is a separately compiled part: compile with -DTDS_KERNEL_PART=p for part
  // p (see "parts" in kernels.json, all parts without it), and
  // -DTDS_KERNELS_FLOAT or -DTDS_KERNELS_DOUBLE. For segment k of function
  // NAME it defines the kernel
  //   extern "C" __global__ void tds_kernel_NAME_k_float(int num_worlds,
  //       int stride, const float* input_0, ..., float* output_0, ...,
  //       float* scratch, int iterations)
  // with one thread per world. All arrays have the same stride.
  std::string runtime_cuda_source() const {
    std::ostringstream code;
    code << generated_comment();
    code << R"(
// CUDA kernels of the model for NVRTC. Compile with -DTDS_KERNELS_FLOAT or
// -DTDS_KERNELS_DOUBLE, and -DTDS_KERNEL_PART=p to compile only part p.

#define TDS_RESTRICT __restrict__
#define TDS_KERNEL_FUNCTION static __device__ __forceinline__

)";
    const auto all_parts = parts();
    for (size_t p = 0; p < all_parts.size(); ++p) {
      const Function& f = functions_[all_parts[p].function];
      const int k = all_parts[p].segment;
      code << "#if !defined(TDS_KERNEL_PART) || TDS_KERNEL_PART == " << p
           << "\n\n";
      code << segment_definition(f, k, true);
      for (const std::string type : {"float", "double"}) {
        code << "#ifdef TDS_KERNELS_" << (type == "float" ? "FLOAT" : "DOUBLE")
             << "\n";
        code << "extern \"C\" __global__ void "
             << runtime_symbol(f.name, k, type) << "(int num_worlds, int stride"
             << array_parameters(f, type) << ", " << type
             << "* TDS_RESTRICT scratch, int iterations) {\n";
        code << "  const int i = blockIdx.x * blockDim.x + threadIdx.x;\n";
        code << "  if (i >= num_worlds) return;\n";
        std::vector<std::string> in, out;
        for (const auto& array : f.inputs) in.push_back(array.name + " + i");
        for (const auto& array : f.outputs) out.push_back(array.name + " + i");
        code << world_body(f, k, in, out, "scratch + i", type, "  ");
        code << "}\n#endif\n";
      }
      code << "\n#endif  // part " << p << "\n\n";
    }
    return code.str();
  }

  // description of the model for kernel_loader.hpp
  std::string manifest_json() const {
    std::ostringstream code;
    code << "{\n";
    code << "  \"format\": \"tds_kernels\",\n";
    code << "  \"version\": 1,\n";
    code << "  \"name\": " << string_literal(model_name_) << ",\n";
    code << "  \"functions\": [\n";
    for (size_t i = 0; i < functions_.size(); ++i) {
      const Function& f = functions_[i];
      code << "    {\"name\": " << string_literal(f.name)
           << ", \"inputs\": " << json_arrays(f.inputs)
           << ", \"outputs\": " << json_arrays(f.outputs)
           << ", \"num_operations\": " << f.num_operations
           << ", \"num_temporaries\": " << f.num_temporaries
           << ", \"segments\": " << function_segments()[i].size()
           << ", \"scratch_dim\": "
           << scratch_dim(f, function_segments()[i])
           << ", \"loop_input\": " << string_literal(f.loop_input)
           << ", \"default_iterations\": " << f.default_iterations
           << "}" << (i + 1 < functions_.size() ? "," : "") << "\n";
    }
    code << "  ],\n";
    code << "  \"parts\": [";
    const auto all_parts = parts();
    for (size_t p = 0; p < all_parts.size(); ++p) {
      const Function& f = functions_[all_parts[p].function];
      const Segment& segment =
          function_segments()[all_parts[p].function][all_parts[p].segment];
      code << (p ? ",\n" : "\n") << "    {\"function\": "
           << string_literal(f.name) << ", \"segment\": "
           << all_parts[p].segment << ", \"code_kb\": "
           << segment.code.size() / 1024
           << ", \"live_in\": " << segment.live_in.size()
           << ", \"live_out\": " << segment.live_out.size() << "}";
    }
    code << "\n  ],\n";
    code << "  \"cpu_parts\": [";
    const auto cpu_parts = parts(false);
    for (size_t p = 0; p < cpu_parts.size(); ++p) {
      code << (p ? ", " : "")
           << string_literal(functions_[cpu_parts[p].function].name);
    }
    code << "],\n";
    code << "  \"constants\": {";
    bool first = true;
    for (const auto& [name, value] : constants_) {
      code << (first ? "\n" : ",\n") << "    " << string_literal(name) << ": "
           << (int_constants_.count(name)
                   ? std::to_string(static_cast<long long>(value))
                   : json_number(value));
      first = false;
    }
    code << (first ? "},\n" : "\n  },\n");
    code << "  \"strings\": {";
    first = true;
    for (const auto& [name, value] : strings_) {
      code << (first ? "\n" : ",\n") << "    " << string_literal(name) << ": "
           << string_literal(value);
      first = false;
    }
    code << (first ? "}" : "\n  }");
    if (!metadata_json_.empty()) {
      code << ",\n  \"metadata\": " << metadata_json_;
    }
    code << "\n}\n";
    return code.str();
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
    // see add_loop_function
    std::string loop_input;
    int default_iterations{1};
  };

  // A part of the generated code of a function, see Options::max_segment_kb.
  // The segment loads the temporaries live_in from the scratch array (live_in
  // [k] at scratch[k * ss]) and stores live_out to it.
  struct Segment {
    std::string code;
    std::vector<int> temporaries;
    std::vector<int> live_in;
    std::vector<int> live_out;
  };

  // the separately compiled parts of the run time sources
  struct Part {
    int function;
    int segment;
  };

  // A statement of the generated code: an assignment, or an if-else block.
  struct Statement {
    std::string code;
    std::vector<int> defs;
    std::vector<int> uses;
    size_t cost{0};
  };

  // the temporaries vN in a line of generated code
  static std::vector<int> temporaries_in(const std::string& line) {
    std::vector<int> ids;
    auto is_identifier = [](char c) {
      return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
    };
    for (size_t i = 0; i + 1 < line.size(); ++i) {
      if (line[i] != 'v' || (i > 0 && is_identifier(line[i - 1])) ||
          !std::isdigit(static_cast<unsigned char>(line[i + 1]))) {
        continue;
      }
      size_t j = i + 1;
      int id = 0;
      while (j < line.size() && std::isdigit(static_cast<unsigned char>(line[j]))) {
        id = 10 * id + (line[j] - '0');
        ++j;
      }
      if (j < line.size() && is_identifier(line[j])) continue;
      ids.push_back(id);
      i = j - 1;
    }
    return ids;
  }

  // splits an assignment "lhs = rhs;" into the assigned temporary (or -1 for
  // an output array element) and the temporaries used
  static void parse_assignment(const std::string& line, int& def,
                               std::vector<int>& uses) {
    const size_t eq = line.find(" = ");
    def = -1;
    if (eq == std::string::npos) {
      auto ids = temporaries_in(line);
      uses.insert(uses.end(), ids.begin(), ids.end());
      return;
    }
    auto lhs = temporaries_in(line.substr(0, eq));
    if (lhs.size() == 1 && line.substr(0, eq).find('[') == std::string::npos) {
      def = lhs[0];
    } else {
      uses.insert(uses.end(), lhs.begin(), lhs.end());
    }
    auto rhs = temporaries_in(line.substr(eq + 3));
    uses.insert(uses.end(), rhs.begin(), rhs.end());
  }

  static std::vector<Statement> parse_statements(const std::string& body) {
    std::vector<Statement> statements;
    std::istringstream lines(body);
    std::string line;
    while (std::getline(lines, line)) {
      const size_t start = line.find_first_not_of(' ');
      if (start == std::string::npos) continue;
      Statement st;
      if (line.compare(start, 3, "if(") != 0) {
        st.code = line.substr(start) + "\n";
        int def;
        parse_assignment(st.code, def, st.uses);
        if (def >= 0) st.defs.push_back(def);
      } else {
        // if-else block: temporaries assigned in both branches are defined
        // by the block, the others keep their previous value in one branch
        int depth = 0;
        std::vector<std::set<int>> branch_defs(1);
        std::vector<int> uses;
        std::string block;
        bool first = true;
        do {
          if (!first && !std::getline(lines, line)) break;
          const size_t b = line.find_first_not_of(' ');
          const std::string text = b == std::string::npos ? "" : line.substr(b);
          // indent relative to the block
          block += std::string(b == std::string::npos ? 0 : b - std::min(b, start), ' ') +
                   text + "\n";
          if (first) {
            auto ids = temporaries_in(text);
            uses.insert(uses.end(), ids.begin(), ids.end());
          } else if (text.rfind("} else", 0) == 0) {
            branch_defs.emplace_back();
          } else if (text != "}") {
            int def;
            parse_assignment(text, def, uses);
            if (def >= 0) branch_defs.back().insert(def);
          }
          for (char c : text) {
            if (c == '{') ++depth;
            if (c == '}') --depth;
          }
          first = false;
        } while (depth > 0);
        st.code = block;
        std::set<int> all_defs;
        for (const auto& d : branch_defs) all_defs.insert(d.begin(), d.end());
        for (int d : all_defs) {
          bool in_all = branch_defs.size() > 1;
          for (const auto& bd : branch_defs) in_all &= bd.count(d) > 0;
          if (in_all) {
            st.defs.push_back(d);
          } else {
            uses.push_back(d);
          }
        }
        st.uses = uses;
      }
      for (char c : st.code) st.cost += c != ' ' && c != '\n';
      statements.push_back(std::move(st));
    }
    return statements;
  }

  // Splits the code of the function into segments of at most about
  // max_segment_kb KB, at the statements where the fewest temporaries are
  // live.
  std::vector<Segment> segments(const Function& f, bool split) const {
    const std::vector<Statement> statements = parse_statements(f.body);
    const size_t n = statements.size();
    size_t total_cost = 0;
    int max_id = 0;
    for (const auto& st : statements) {
      total_cost += st.cost;
      for (int id : st.defs) max_id = std::max(max_id, id);
      for (int id : st.uses) max_id = std::max(max_id, id);
    }
    int num_segments = 1;
    if (split && options_.max_segment_kb > 0 && f.loop_input.empty()) {
      const size_t max_cost = static_cast<size_t>(options_.max_segment_kb) * 1024;
      num_segments = static_cast<int>((total_cost + max_cost - 1) / max_cost);
      num_segments = std::max(1, std::min<int>(num_segments, static_cast<int>(n)));
    }

    // live[i]: number of live temporaries before statement i
    auto liveness = [&](const std::vector<size_t>& record,
                        std::map<size_t, std::vector<int>>* sets) {
      std::vector<char> is_live(max_id + 1, 0);
      std::vector<int> count(n + 1, 0);
      int live = 0;
      for (size_t i = n; i-- > 0;) {
        for (int d : statements[i].defs) {
          if (is_live[d]) is_live[d] = 0, --live;
        }
        for (int u : statements[i].uses) {
          if (!is_live[u]) is_live[u] = 1, ++live;
        }
        count[i] = live;
        if (sets && std::binary_search(record.begin(), record.end(), i)) {
          auto& set = (*sets)[i];
          for (int id = 0; id <= max_id; ++id) {
            if (is_live[id]) set.push_back(id);
          }
        }
      }
      return count;
    };

    std::vector<size_t> cuts = {0};
    if (num_segments > 1) {
      const std::vector<int> live = liveness({}, nullptr);
      const double segment_cost = double(total_cost) / num_segments;
      size_t cost = 0;
      size_t i = 0;
      for (int k = 1; k < num_segments; ++k) {
        const double target = k * segment_cost;
        // the boundary with the fewest live temporaries near the target
        while (i < n && cost + statements[i].cost < target - 0.15 * segment_cost) {
          cost += statements[i++].cost;
        }
        size_t best = 0;
        int best_live = -1;
        size_t c = cost;
        for (size_t j = i; j < n && c <= target + 0.15 * segment_cost; ++j) {
          if (j > cuts.back() && (best_live < 0 || live[j] < best_live)) {
            best = j;
            best_live = live[j];
          }
          c += statements[j].cost;
        }
        if (best_live >= 0) cuts.push_back(best);
      }
    }
    std::map<size_t, std::vector<int>> live_sets;
    if (cuts.size() > 1) {
      std::vector<size_t> record(cuts.begin() + 1, cuts.end());
      liveness(record, &live_sets);
    }
    cuts.push_back(n);

    std::vector<Segment> result;
    for (size_t k = 0; k + 1 < cuts.size(); ++k) {
      Segment segment;
      std::set<int> temporaries;
      for (size_t i = cuts[k]; i < cuts[k + 1]; ++i) {
        segment.code += statements[i].code;
        temporaries.insert(statements[i].defs.begin(), statements[i].defs.end());
        temporaries.insert(statements[i].uses.begin(), statements[i].uses.end());
      }
      if (k > 0) segment.live_in = live_sets[cuts[k]];
      if (k + 2 < cuts.size()) segment.live_out = live_sets[cuts[k + 1]];
      temporaries.insert(segment.live_in.begin(), segment.live_in.end());
      temporaries.insert(segment.live_out.begin(), segment.live_out.end());
      segment.temporaries.assign(temporaries.begin(), temporaries.end());
      result.push_back(std::move(segment));
    }
    return result;
  }

  // the segments of all functions, computed once
  // The segments of all functions, computed once. Only the CUDA kernels are
  // split, the C++ compilers are fast enough for large functions, and the
  // CPU kernels are faster without the scratch arrays.
  const std::vector<std::vector<Segment>>& function_segments(
      bool cuda = true) const {
    auto& cache = cuda ? cuda_segments_ : cpu_segments_;
    if (cache.size() != functions_.size()) {
      cache.clear();
      for (const auto& f : functions_) cache.push_back(segments(f, cuda));
    }
    return cache;
  }

  std::vector<Part> parts(bool cuda = true) const {
    std::vector<Part> result;
    const auto& all = function_segments(cuda);
    for (size_t f = 0; f < all.size(); ++f) {
      for (size_t k = 0; k < all[f].size(); ++k) {
        result.push_back({static_cast<int>(f), static_cast<int>(k)});
      }
    }
    return result;
  }

  // size of the scratch array of a function per world: the temporaries
  // between segments, or the previous output of a loop function
  static int scratch_dim(const Function& f,
                         const std::vector<Segment>& segments) {
    size_t dim = 0;
    for (const auto& segment : segments) {
      dim = std::max({dim, segment.live_in.size(), segment.live_out.size()});
    }
    if (!f.loop_input.empty()) {
      dim = std::max(dim, static_cast<size_t>(f.outputs[0].dim));
    }
    return static_cast<int>(dim);
  }

  // Body of the CPU launcher or CUDA kernel of segment k of function f for
  // world i, with the expressions of the input and output arrays of the
  // world (such as "inputs[0] + i"), and of its scratch array.
  std::string world_body(const Function& f, int k,
                         const std::vector<std::string>& inputs,
                         const std::vector<std::string>& outputs,
                         const std::string& scratch, const std::string& type,
                         const std::string& indent) const {
    std::ostringstream code;
    auto call = [&](const std::vector<std::string>& in,
                    const std::vector<std::string>& out) {
      std::string c = model_name_ + "_kernels::" + f.name + "_" +
                      std::to_string(k) + "<" + type + ">(stride";
      for (const auto& a : in) c += ", " + a;
      for (const auto& a : out) c += ", " + a;
      return c + ", " + scratch + ", stride);";
    };
    if (f.loop_input.empty()) {
      code << indent << call(inputs, outputs) << "\n";
      return code.str();
    }
    // a loop function: the iterations alternate between the output and the
    // scratch array, such that the last one writes the output
    std::string x0;
    std::vector<std::string> in = inputs;
    for (size_t a = 0; a < f.inputs.size(); ++a) {
      if (f.inputs[a].name == f.loop_input) {
        x0 = inputs[a];
        in[a] = "tds_x";
      }
    }
    const std::string& y = outputs[0];
    code << indent << "for (int tds_it = 0; tds_it < iterations; ++tds_it) {\n";
    code << indent << "  const " << type << "* tds_x = tds_it == 0 ? " << x0
         << " : ((iterations - tds_it) % 2 == 0 ? " << y << " : " << scratch
         << ");\n";
    code << indent << "  " << type
         << "* tds_y = (iterations - 1 - tds_it) % 2 == 0 ? " << y << " : "
         << scratch << ";\n";
    code << indent << "  " << call(in, {"tds_y"}) << "\n";
    code << indent << "}\n";
    code << indent << "if (iterations <= 0) {\n";
    code << indent << "  for (int tds_k = 0; tds_k < " << f.outputs[0].dim
         << "; ++tds_k) {\n";
    code << indent << "    (" << y << ")[tds_k * stride] = (" << x0
         << ")[tds_k * stride];\n";
    code << indent << "  }\n";
    code << indent << "}\n";
    return code.str();
  }

  // definition of segment k of function f, as a template on the floating
  // point type in the namespace NAME_kernels
  std::string segment_definition(const Function& f, int k, bool cuda) const {
    const auto& segments = function_segments(cuda)[&f - &functions_[0]];
    const Segment& segment = segments[k];
    const int num_segments = static_cast<int>(segments.size());
    std::ostringstream code;
    code << "namespace " << model_name_ << "_kernels {\n\n";
    code << "// " << f.name << ", segment " << k + 1 << " of " << num_segments
         << ": " << array_list(f.inputs) << " -> " << array_list(f.outputs)
         << "\n";
    code << "template <typename Float>\n";
    code << "TDS_KERNEL_FUNCTION void " << f.name << "_" << k << "(const int s"
         << array_parameters(f, "Float")
         << ", Float* TDS_RESTRICT scratch, const int ss) {\n";
    const int per_line = 10;
    for (size_t i = 0; i < segment.temporaries.size(); ++i) {
      code << (i % per_line == 0 ? "  Float " : ", ") << "v"
           << segment.temporaries[i];
      if (i % per_line == per_line - 1 || i + 1 == segment.temporaries.size()) {
        code << ";\n";
      }
    }
    for (size_t i = 0; i < segment.live_in.size(); ++i) {
      code << "  v" << segment.live_in[i] << " = scratch[" << i << " * ss];\n";
    }
    std::istringstream body(segment.code);
    std::string line;
    while (std::getline(body, line)) code << "  " << line << "\n";
    for (size_t i = 0; i < segment.live_out.size(); ++i) {
      code << "  scratch[" << i << " * ss] = v" << segment.live_out[i] << ";\n";
    }
    code << "}\n\n}  // namespace " << model_name_ << "_kernels\n\n";
    return code.str();
  }

  std::string model_name_;
  Options options_;
  std::vector<Function> functions_;
  // all arrays by name, arrays with the same name must have the same size
  std::map<std::string, int> arrays_;
  std::map<std::string, double> constants_;
  std::set<std::string> int_constants_;
  std::map<std::string, std::string> strings_;
  std::string metadata_json_;
  mutable std::vector<std::vector<Segment>> cuda_segments_;
  mutable std::vector<std::vector<Segment>> cpu_segments_;

  static std::string json_arrays(const std::vector<KernelArray>& arrays) {
    std::string list;
    for (const auto& array : arrays) {
      if (!list.empty()) list += ", ";
      list += "{\"name\": " + string_literal(array.name) +
              ", \"dim\": " + std::to_string(array.dim) + "}";
    }
    return "[" + list + "]";
  }

  static std::string json_number(double value) {
    if (!std::isfinite(value)) return "null";
    std::ostringstream os;
    os.precision(17);
    os << value;
    return os.str();
  }

  static int total_dim(const std::vector<KernelArray>& arrays) {
    int dim = 0;
    for (const auto& array : arrays) dim += array.dim;
    return dim;
  }

  void check_array(const KernelArray& array) {
    // the arrays are parameters of the generated code
    static const std::set<std::string> reserved = {
        "s",     "ss",      "i",          "stride",  "scratch", "inputs",
        "outputs", "begin", "end",        "iterations", "num_worlds", "Float"};
    bool valid = !array.name.empty() &&
                 !std::isdigit(static_cast<unsigned char>(array.name[0])) &&
                 !reserved.count(array.name) &&
                 array.name.rfind("tds_", 0) != 0 &&
                 temporaries_in(array.name).empty();
    for (char c : array.name) {
      valid &= std::isalnum(static_cast<unsigned char>(c)) || c == '_';
    }
    if (!valid) {
      throw std::runtime_error("KernelCodegen: invalid array name \"" +
                               array.name + "\".");
    }
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
    std::ofstream file(path, std::ios::binary);
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

  // a C++ and JSON string literal
  static std::string string_literal(const std::string& value) {
    std::string s = "\"";
    for (char c : value) {
      if (c == '"' || c == '\\') {
        s += '\\';
        s += c;
      } else if (c == '\n') {
        s += "\\n";
      } else if (c == '\t') {
        s += "\\t";
      } else {
        s += c;
      }
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
    code << function_definitions();
    return code.str();
  }

  // float overloads of the math functions in the namespace NAME_kernels
  std::string math_using_declarations() const {
    std::ostringstream code;
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
    code << "}  // namespace " << model_name_ << "_kernels\n\n";
    return code.str();
  }

  // the generated functions as templates on the floating point type, in the
  // namespace NAME_kernels. TDS_KERNEL_FUNCTION and TDS_RESTRICT must be
  // defined.
  std::string function_definitions() const {
    std::ostringstream code;
    code << math_using_declarations();
    code << "namespace " << model_name_ << "_kernels {\n\n";
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
      for (const std::string type : {"float", "double"}) {
        code << "      f.cpu_" << type << " = [](int n, int stride, const "
             << type << "* const* in, " << type
             << "* const* out, int threads, int) {\n"
             << "        " << launcher_name(f, "cpu", type)
             << "(n, stride, in, out, threads);\n      };\n";
      }
      code << "#ifdef TDS_KERNELS_CUDA\n";
      for (const std::string type : {"float", "double"}) {
        code << "      f.cuda_" << type << " = [](int n, int stride, const "
             << type << "* const* in, " << type
             << "* const* out, int block_size, void* stream, int) {\n"
             << "        " << launcher_name(f, "cuda", type)
             << "(n, stride, in, out, block_size, stream);\n      };\n";
      }
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
