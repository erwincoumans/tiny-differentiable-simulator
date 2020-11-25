#pragma once

#include <cmath>

// clang-format off
// Stan Math needs to be included first to get its Eigen plugins
#if USE_STAN
#include <stan/math.hpp>
#include <stan/math/fwd.hpp>
#endif
#include <ceres/autodiff_cost_function.h>
// clang-format on

#include <cppad/cg.hpp>
#include <cppad/cg/support/cppadcg_eigen.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "base.hpp"
#include "math/tiny/tiny_dual.h"
#include "math/tiny/tiny_dual_utils.h"
#include "math/eigen_algebra.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/ceres_utils.h"
#include "math/tiny/cppad_utils.h"
#include "stopwatch.hpp"

namespace tds {
enum DiffMethod {
  DIFF_NUMERICAL,
  DIFF_CERES,
  DIFF_DUAL,
  DIFF_STAN_REVERSE,
  DIFF_STAN_FORWARD,
  DIFF_CPPAD_AUTO,
  DIFF_CPPAD_CODEGEN_AUTO,
};

/**
 * Choose whether to use TinyAlgebra (true) or EigenAlgebra for differentiation
 * with CppADCodeGen.
 */
static const inline bool DiffCppAdCodeGenUsesTinyAlgebra = false;

TINY_INLINE std::string diff_method_name(DiffMethod m) {
  switch (m) {
    case DIFF_NUMERICAL:
      return "NUMERICAL";
    case DIFF_CERES:
      return "CERES";
    case DIFF_DUAL:
      return "DUAL";
    case DIFF_STAN_REVERSE:
      return "STAN_REVERSE";
    case DIFF_STAN_FORWARD:
      return "STAN_FORWARD";
    case DIFF_CPPAD_AUTO:
      return "CPPAD_AUTO";
    case DIFF_CPPAD_CODEGEN_AUTO:
      return "CPPAD_CODEGEN_AUTO";
    default:
      return "UNKNOWN";
  }
}

template <DiffMethod Method, int Dim, typename Scalar>
struct default_diff_algebra {};
template <int Dim, typename Scalar>
struct default_diff_algebra<DIFF_NUMERICAL, Dim, Scalar> {
  using type = EigenAlgebraT<Scalar>;
};
template <int Dim, typename Scalar>
struct default_diff_algebra<DIFF_CERES, Dim, Scalar> {
  using ADScalar = ceres::Jet<Scalar, Dim>;
  using type = TinyAlgebra<ADScalar, CeresUtils<Dim, Scalar>>;
};
template <int Dim, typename Scalar>
struct default_diff_algebra<DIFF_DUAL, Dim, Scalar> {
  // using type = TinyAlgebra<TinyDual<Scalar>, TinyDualUtils<Scalar>>;
  using type = TinyAlgebra<TinyDualDouble, TinyDualDoubleUtils>;
};
template <int Dim>
struct default_diff_algebra<DIFF_STAN_REVERSE, Dim, double> {
#if USE_STAN
  using type = EigenAlgebraT<stan::math::var>;
#else
  using type = EigenAlgebra;
  // static_assert(false,
  //               "Need to set USE_STAN option to ON to use Stan's autodiff "
  //               "functionality.");
#endif
};
template <int Dim, typename Scalar>
struct default_diff_algebra<DIFF_STAN_FORWARD, Dim, Scalar> {
#if USE_STAN
  using type = EigenAlgebraT<stan::math::fvar<Scalar>>;
#else
  using type = EigenAlgebra;
  // static_assert(false,
  //               "Need to set USE_STAN option to ON to use Stan's autodiff "
  //               "functionality.");
#endif
};
template <int Dim, typename Scalar>
struct default_diff_algebra<DIFF_CPPAD_AUTO, Dim, Scalar> {
  using ADScalar = typename CppAD::AD<Scalar>;
  using type = EigenAlgebraT<ADScalar>;
};
template <int Dim, typename Scalar>
struct default_diff_algebra<DIFF_CPPAD_CODEGEN_AUTO, Dim, Scalar> {
  using CGScalar = typename CppAD::cg::CG<Scalar>;
  using ADScalar = typename CppAD::AD<CGScalar>;
  using type = std::conditional_t<DiffCppAdCodeGenUsesTinyAlgebra,
                                  TinyAlgebra<ADScalar, CppADUtils<CGScalar>>,
                                  EigenAlgebraT<ADScalar>>;
};

/**
 * Central difference for scalar-valued function `f` given vector `x`.
 */
template <DiffMethod Method, typename F, typename Scalar = double>
static std::enable_if_t<Method == DIFF_NUMERICAL, void> compute_gradient(
    F f, const std::vector<Scalar>& x, std::vector<Scalar>& dfx,
    const Scalar eps = 1e-6) {
  dfx.resize(x.size());
  const Scalar fx = f(x);
  std::vector<Scalar> left_x = x, right_x = x;
  for (std::size_t i = 0; i < x.size(); ++i) {
    left_x[i] -= eps;
    right_x[i] += eps;
    Scalar dx = right_x[i] - left_x[i];
    Scalar fl = f(left_x);
    Scalar fr = f(right_x);
    dfx[i] = (fr - fl) / dx;
    left_x[i] = right_x[i] = x[i];
  }
}

namespace {
template <int Dim, template <typename> typename F, typename Scalar>
struct CeresFunctional {
  F<Scalar> f_double;
  F<ceres::Jet<Scalar, Dim>> f_jet;

  template <typename T>
  bool operator()(const T* const x, T* e) const {
    std::vector<T> arg(x, x + Dim);
    if constexpr (std::is_same_v<T, Scalar>) {
      *e = f_double(arg);
    } else {
      *e = f_jet(arg);
    }
    return true;
  }
};
}  // namespace

/**
 * Forward-mode autodiff using Ceres' Jet implementation.
 * Note that template argument `F` must be a functional that accepts
 * double and ceres::Jet<double, Dim> scalars, as declared by a template
 * argument on F.
 */
template <DiffMethod Method, int Dim, template <typename> typename F,
          typename Scalar = double>
static std::enable_if_t<Method == DIFF_CERES, void> compute_gradient(
    const std::vector<Scalar>& x, std::vector<Scalar>& dfx) {
  assert(static_cast<int>(x.size()) == Dim);
  dfx.resize(x.size());
  typedef CeresFunctional<Dim, F, Scalar> CF;
  ceres::AutoDiffCostFunction<CF, 1, Dim> cost_function(new CF);
  Scalar fx;
  Scalar* grad = dfx.data();
  const Scalar* params = x.data();
  cost_function.Evaluate(&params, &fx, &grad);
}

/**
 * Forward-mode AD using TinyDual for scalar-valued function `f` given vector
 * `x`.
 */
template <DiffMethod Method, typename F, typename Scalar = double>
static std::enable_if_t<Method == DIFF_DUAL, void> compute_gradient(
    F f, const std::vector<Scalar>& x, std::vector<Scalar>& dfx) {
  // typedef TinyDual<Scalar> Dual;
  typedef TinyDualDouble Dual;
  dfx.resize(x.size());
  std::vector<Dual> x_dual(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].real() = x[i];
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].dual() = 1.;
    Dual fx = f(x_dual);
    dfx[i] = fx.dual();
    x_dual[i].dual() = 0.;
  }
}

/**
 * Reverse-mode AD using Stan Math.
 */
template <DiffMethod Method, typename F>
static std::enable_if_t<Method == DIFF_STAN_REVERSE, void> compute_gradient(
    F f, const std::vector<double>& x, std::vector<double>& dfx) {
#if USE_STAN
  dfx.resize(x.size());

  std::vector<stan::math::var> x_var(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_var[i] = x[i];
  }

  stan::math::var fx = f(x_var);
  stan::math::grad(fx.vi_);

  for (std::size_t i = 0; i < x.size(); ++i) {
    dfx[i] = x_var[i].adj();
  }
  stan::math::recover_memory();
  stan::math::zero_adjoints();
#else
  throw std::runtime_error(
      "Variable 'USE_STAN' must be set to use automatic "
      "differentiation functions from Stan Math.");
#endif
}

/**
 * Forward-mode AD using Stan Math.
 */
template <DiffMethod Method, typename F, typename Scalar = double>
static std::enable_if_t<Method == DIFF_STAN_FORWARD, void> compute_gradient(
    F f, const std::vector<Scalar>& x, std::vector<Scalar>& dfx) {
#if USE_STAN
  typedef stan::math::fvar<Scalar> Dual;
  dfx.resize(x.size());

  std::vector<Dual> x_dual(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].val_ = x[i];
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    x_dual[i].d_ = 1.;
    Dual fx = f(x_dual);
    dfx[i] = fx.d_;
    x_dual[i].d_ = 0.;
  }
#else
  throw std::runtime_error(
      "Variable 'USE_STAN' must be set to use automatic "
      "differentiation functions from Stan Math.");
#endif
}

template <DiffMethod Method, template <typename> typename F,
          typename ScalarAlgebra = EigenAlgebra>
struct GradientFunctional {
  static const int kDim = F<ScalarAlgebra>::kDim;
  using Scalar = typename ScalarAlgebra::Scalar;
  virtual Scalar value(const std::vector<Scalar>&) const = 0;
  virtual const std::vector<Scalar>& gradient(
      const std::vector<Scalar>&) const = 0;
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_NUMERICAL, F, ScalarAlgebra> {
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
  mutable std::vector<Scalar> gradient_;

 public:
  static const int kDim = F<ScalarAlgebra>::kDim;
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_NUMERICAL>(f_scalar_, x, gradient_);
    return gradient_;
  }
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_CERES, F, ScalarAlgebra> {
 public:
  static const int kDim = F<ScalarAlgebra>::kDim;

 private:
  using Scalar = typename ScalarAlgebra::Scalar;
  using ADScalar = ceres::Jet<Scalar, kDim>;
  mutable std::vector<Scalar> gradient_;

  struct CostFunctional {
    GradientFunctional* parent;

    F<ScalarAlgebra> f_scalar;
    F<TinyAlgebra<ADScalar, CeresUtils<kDim, Scalar>>> f_jet;

    CostFunctional(GradientFunctional* parent) : parent(parent) {}

    template <typename T>
    bool operator()(const T* const x, T* e) const {
      std::vector<T> arg(x, x + kDim);
      if constexpr (std::is_same_v<T, Scalar>) {
        *e = f_scalar(arg);
      } else {
        *e = f_jet(arg);
      }
      return true;
    }
  };

  CostFunctional* cost_{nullptr};
  ceres::AutoDiffCostFunction<CostFunctional, 1, kDim> cost_function_;

 public:
  // CostFunctional pointer is managed by cost_function_.
  GradientFunctional()
      : cost_(new CostFunctional(this)), cost_function_(cost_) {}
  GradientFunctional(GradientFunctional& f)
      : cost_(new CostFunctional(this)), cost_function_(cost_) {}
  GradientFunctional(const GradientFunctional& f)
      : cost_(new CostFunctional(this)), cost_function_(cost_) {}
  GradientFunctional& operator=(const GradientFunctional& f) {
    if (cost_) {
      delete cost_;
    }
    cost_ = new CostFunctional(this);
    cost_function_ =
        ceres::AutoDiffCostFunction<CostFunctional, 1, kDim>(cost_);
    return *this;
  }

  Scalar value(const std::vector<Scalar>& x) const {
    return cost_->f_scalar(x);
  }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    assert(static_cast<int>(x.size()) == kDim);
    gradient_.resize(x.size());
    Scalar fx;
    Scalar* grad = gradient_.data();
    const Scalar* params = x.data();
    cost_function_.Evaluate(&params, &fx, &grad);
    return gradient_;
  }
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_DUAL, F, ScalarAlgebra> {
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
  // F<TinyAlgebra<TinyDual<Scalar>, TinyDualUtils<Scalar>>> f_ad_;
  F<TinyAlgebra<TinyDualDouble, TinyDualDoubleUtils>> f_ad_;
  mutable std::vector<Scalar> gradient_;

 public:
  static const int kDim = F<ScalarAlgebra>::kDim;
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_DUAL>(f_ad_, x, gradient_);
    return gradient_;
  }
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_STAN_REVERSE, F, ScalarAlgebra> {
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
#if USE_STAN
  F<EigenAlgebraT<stan::math::var>> f_ad_;
  mutable std::vector<Scalar> gradient_;

 public:
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_STAN_REVERSE>(f_ad_, x, gradient_);
    return gradient_;
  }
#else
 public:
  static const int kDim = F<ScalarAlgebra>::kDim;
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    throw std::runtime_error(
        "Variable 'USE_STAN' must be set to use automatic "
        "differentiation functions from Stan Math.");
  }
#endif
};

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_STAN_FORWARD, F, ScalarAlgebra> {
  using Scalar = typename ScalarAlgebra::Scalar;
  F<ScalarAlgebra> f_scalar_;
#if USE_STAN
  F<EigenAlgebraT<stan::math::fvar<Scalar>>> f_ad_;
  mutable std::vector<Scalar> gradient_;

 public:
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    tds::compute_gradient<tds::DIFF_STAN_FORWARD>(f_ad_, x, gradient_);
    return gradient_;
  }
#else
 public:
  static const int kDim = F<ScalarAlgebra>::kDim;
  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    throw std::runtime_error(
        "Variable 'USE_STAN' must be set to use automatic "
        "differentiation functions from Stan Math.");
  }
#endif
};

static bool gCppADParallelMode = false;
template <typename GradientFunctional>
inline void CppADParallelSetup(int num_threads) {
  auto in_parallel = []() { return gCppADParallelMode; };
  auto thread_num = []() {
    static std::map<std::thread::id, std::size_t> thread_ids;
    static std::mutex thread_ids_mutex;
    std::lock_guard<std::mutex> thread_ids_lock(thread_ids_mutex);
    const std::thread::id my_id = std::this_thread::get_id();
    if (thread_ids.find(my_id) == thread_ids.end()) {
      thread_ids[my_id] = thread_ids.size();
    }
    return thread_ids[my_id];
  };

  CppAD::thread_alloc::parallel_setup(num_threads, in_parallel, thread_num);
  // Try enabling this for faster execution.
  CppAD::thread_alloc::hold_memory(true);
  CppAD::parallel_ad<typename GradientFunctional::Scalar>();
  CppAD::parallel_ad<typename GradientFunctional::Dual>();
  gCppADParallelMode = true;
}

template <typename GradientFunctional>
inline void CppADParallelShutdown() {
  CppAD::thread_alloc::parallel_setup(1, CPPAD_NULL, CPPAD_NULL);
  gCppADParallelMode = false;
}

template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_CPPAD_AUTO, F, ScalarAlgebra> {
 public:
  using Scalar = typename ScalarAlgebra::Scalar;
  using Dual = typename CppAD::AD<Scalar>;
  static const int kDim = F<ScalarAlgebra>::kDim;

  GradientFunctional(const std::vector<Scalar>& x_init = {}) { Init(x_init); }
  GradientFunctional(const GradientFunctional& other) { Init(); }
  GradientFunctional(GradientFunctional& f) { Init(); }
  GradientFunctional& operator=(const GradientFunctional& other) = delete;

  Scalar value(const std::vector<Scalar>& x) const { return f_scalar_(x); }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    gradient_ = tape_.Jacobian(x);
    return gradient_;
  }

 private:
  void Init(const std::vector<Scalar>& x_init = {}) {
    std::vector<Dual> ax(kDim);
    for (int i = 0; i < kDim; ++i) {
      if (i < static_cast<int>(x_init.size())) {
        ax[i] = x_init[i];
      } else {
        ax[i] = ScalarAlgebra::zero();
      }
    }
    CppAD::Independent(ax);
    std::vector<Dual> ay(1);
    ay[0] = f_ad_(ax);
    // std::cout << "CppAD value: " << ay[0] << std::endl;
    tape_.Dependent(ax, ay);
    tape_.optimize();
  }

  F<ScalarAlgebra> f_scalar_;
  F<EigenAlgebraT<Dual>> f_ad_;
  mutable CppAD::ADFun<Scalar> tape_;
  mutable std::vector<Scalar> gradient_;
};

namespace {
// make sure every model has its own ID
static inline int cpp_ad_codegen_model_counter = 0;
}  // namespace

struct CodeGenSettings {
  bool verbose{true};
  bool use_clang{true};
  int optimization_level{0};
  std::size_t max_assignments_per_func{5000};
  std::size_t max_operations_per_assignment{50};
  std::string sources_folder{"cppadcg_src"};
  bool save_to_disk{true};

  // function arguments used while generating the code (will be zero if not set)
  std::vector<double> default_x;
  std::vector<double> default_nograd_x;
};

#if CPPAD_CG_SYSTEM_LINUX
template <template <typename> typename F, typename ScalarAlgebra>
class GradientFunctional<DIFF_CPPAD_CODEGEN_AUTO, F, ScalarAlgebra> {
 public:
  using Scalar = typename ScalarAlgebra::Scalar;
  using CGScalar = typename CppAD::cg::CG<Scalar>;
  using Dual = typename CppAD::AD<CGScalar>;
  static const int kDim = F<ScalarAlgebra>::kDim;
  using DualAlgebra = typename default_diff_algebra<DIFF_CPPAD_CODEGEN_AUTO,
                                                    kDim, Scalar>::type;

  static void Compile(const CodeGenSettings& settings = CodeGenSettings()) {
    std::vector<Dual> ax(kDim + settings.default_nograd_x.size());
    for (std::size_t i = 0; i < kDim; ++i) {
      if (i >= settings.default_x.size()) {
        ax[i] = ScalarAlgebra::zero();
      } else {
        ax[i] = settings.default_x[i];
      }
    }

    for (std::size_t i = 0; i < settings.default_nograd_x.size(); ++i) {
      ax[i + kDim] = settings.default_nograd_x[i];
    }

    CppAD::Independent(ax);
    std::vector<Dual> ay(1);
    std::cout << "Tracing cost functor for code generation...\n";
    F<DualAlgebra> f;
    ay[0] = f(ax);
    CppAD::ADFun<CGScalar> tape;
    tape.Dependent(ax, ay);
    tape.optimize();

    Stopwatch timer;
    timer.start();
    std::string model_name =
        "model_" + std::to_string(++cpp_ad_codegen_model_counter);
    CppAD::cg::ModelCSourceGen<Scalar> cgen(tape, model_name);
    cgen.setCreateSparseJacobian(true);
    // cgen.setCreateJacobian(true);
    if (settings.default_nograd_x.size() > 0) {
      if (settings.verbose) {
        printf(
            "Dynamic parameters provided, creating sparsity pattern. (%d "
            "active, %ld inactive)\n",
            kDim, ax.size() - kDim);
      }
      std::vector<size_t> rows(kDim, 0);
      std::vector<size_t> cols(kDim, 0);
      std::iota(cols.begin(), cols.end(), 0);
      cgen.setCustomSparseJacobianElements(rows, cols);
    }
    cgen.setMaxAssignmentsPerFunc(settings.max_assignments_per_func);
    cgen.setMaxOperationsPerAssignment(settings.max_operations_per_assignment);
    if (settings.verbose) {
      printf("Created CppAD::cg::ModelCSourceGen.\t(%.3fs)\n", timer.stop());
      fflush(stdout);
      timer.start();
    }
    CppAD::cg::ModelLibraryCSourceGen<Scalar> libcgen(cgen);
    libcgen.setVerbose(settings.verbose);
    if (settings.verbose) {
      printf("Created CppAD::cg::ModelLibraryCSourceGen.\t(%.3fs)\n",
             timer.stop());
      fflush(stdout);
      timer.start();
    }
    CppAD::cg::DynamicModelLibraryProcessor<Scalar> p(libcgen);
    if (settings.verbose) {
      printf("Created CppAD::cg::DynamicModelLibraryProcessor.\t(%.3fs)\n",
             timer.stop());
      fflush(stdout);
      timer.start();
    }
    std::unique_ptr<CppAD::cg::AbstractCCompiler<Scalar>> compiler;
    if (settings.use_clang) {
      compiler = std::make_unique<CppAD::cg::ClangCompiler<Scalar>>();
      printf("Created CppAD::cg::ClangCompiler.\t");
    } else {
      compiler = std::make_unique<CppAD::cg::GccCompiler<Scalar>>();
      printf("Created CppAD::cg::GccCompiler.\t");
    }
    compiler->setSourcesFolder(settings.sources_folder);
    compiler->setSaveToDiskFirst(settings.save_to_disk);
    compiler->addCompileFlag("-O" +
                             std::to_string(settings.optimization_level));
    std::cout << "{ ";
    for (const auto& flag : compiler->getCompileFlags()) {
      std::cout << flag << " ";
    }
    std::cout << "}\t";
    if (settings.verbose) {
      printf("(%.3fs)\n", timer.stop());
      fflush(stdout);
      timer.start();
    }

    p.setLibraryName(model_name);
    p.createDynamicLibrary(*compiler, false);
    std::cout << "Created new dynamic library at " << p.getLibraryName()
              << ".so.\n";
    if (settings.verbose) {
      printf("Finished compiling dynamic library.\t(%.3fs)\n", timer.stop());
      fflush(stdout);
    }
  }

  Scalar value(const std::vector<Scalar>& x) const {
    const auto fx = model_->ForwardZero(x);
    #ifndef NDEBUG
        const auto fx_slow = f_scalar_(x);
        const bool close = std::fabs(fx_slow - fx[0]) < 1e-6;
        if (!close) {
          std::cout << "Scalar/CodeGen 0th order mismatch: " << fx_slow << " vs "
                    << fx[0] << "\n";
        }
        assert(close);
    #endif
    return fx[0];
  }
  const std::vector<Scalar>& gradient(const std::vector<Scalar>& x) const {
    assert(lib_ != nullptr && model_ != nullptr);
    gradient_.resize(kDim);
    rows_.resize(kDim);
    cols_.resize(kDim);
    model_->SparseJacobian(x, gradient_, rows_, cols_);
    // gradient_ = model_->Jacobian(x);
    // gradient_.resize(kDim);
    // #ifndef NDEBUG
    //     // In debug mode, verify the gradient matches the (slower) Ceres
    //     gradient.
    //     // This can help catch if/else branches that CppAD isn't aware of.
    //     if (kDim != static_cast<int>(x.size())) {
    //       std::cout << "Cannot compare codegen gradient against Ceres at the"
    //                    "moment if the functor has non-grad variables as
    //                    input.\n";
    //     } else {
    //       const auto ceres_gradient = ceres_functional_.gradient(x);
    //       assert(ceres_gradient.size() == gradient_.size());
    //       bool allclose = true;
    //       for (size_t i = 0; i < ceres_gradient.size(); ++i) {
    //         const bool close = std::fabs(ceres_gradient[i] - gradient_[i]) <
    //         1e-6; if (!close) {
    //           std::cout << "Ceres/CodeGen gradient mismatch at " << i << ": "
    //                     << ceres_gradient[i] << " vs " << gradient_[i] <<
    //                     "\n";
    //           allclose = false;
    //         }
    //       }
    //       assert(allclose);
    //     }
    // #endif
    return gradient_;
  }

  void Init() {
    if (library_name_.empty()) {
      library_name_ = "./" + model_name_ + ".so";
    }
    lib_ = std::make_unique<CppAD::cg::LinuxDynamicLib<Scalar>>(library_name_);
    model_ = lib_->model(model_name_);
    std::cout << "Loaded compiled model \"" << model_name_ << "\" from \""
              << library_name_ << "\".\n";
  }

  GradientFunctional(const std::string& model_name =
                         "model_" +
                         std::to_string(cpp_ad_codegen_model_counter),
                     const std::string& library_name = "")
      : model_name_(model_name), library_name_(library_name) {
    Init();
  }
  GradientFunctional(const GradientFunctional& other)
      : model_name_(other.model_name_), library_name_(other.library_name_) {
    Init();
  }
  GradientFunctional& operator=(const GradientFunctional& other) {
    model_name_ = other.model_name_;
    library_name_ = other.library_name_;
    Init();
    return *this;
  }

 private:
  // name of the model and library to load, uses latest compiled model by
  // default
  std::string model_name_{""};

  // file name of the dynamic library to load
  std::string library_name_{""};

  #ifndef NDEBUG
    F<ScalarAlgebra> f_scalar_;
  //   GradientFunctional<tds::DIFF_CERES, F, ScalarAlgebra> ceres_functional_;
  #endif
  mutable std::vector<Scalar> gradient_;
  mutable std::vector<std::size_t> rows_;
  mutable std::vector<std::size_t> cols_;
  std::unique_ptr<CppAD::cg::LinuxDynamicLib<Scalar>> lib_{nullptr};
  std::unique_ptr<CppAD::cg::GenericModel<Scalar>> model_;
};
#endif
}  // namespace tds
