#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <cppad/cppad.hpp>

// copy of eigen_mat_inv from CppAD to make the reverse pass codegen compatible (not assume double scalar)

namespace tds {
template <class Base>
class atomic_eigen_mat_inv : public CppAD::atomic_base<Base> {
 public:
  // -----------------------------------------------------------
  // type of elements during calculation of derivatives
  typedef Base scalar;
  // type of elements during taping
  typedef CppAD::AD<scalar> ad_scalar;
  // type of matrix during calculation of derivatives
  typedef Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      matrix;
  // type of matrix during taping
  typedef Eigen::Matrix<ad_scalar, Eigen::Dynamic, Eigen::Dynamic,
                        Eigen::RowMajor>
      ad_matrix;
  /* %$$
  $subhead Constructor$$
  $srccode%cpp% */
  // constructor
  atomic_eigen_mat_inv(void)
      : CppAD::atomic_base<Base>("atom_eigen_mat_inv",
                                 CppAD::atomic_base<Base>::set_sparsity_enum) {}
  /* %$$
  $subhead op$$
  $srccode%cpp% */
  // use atomic operation to invert an AD matrix
  ad_matrix op(const ad_matrix& arg) {
    size_t nr = size_t(arg.rows());
    size_t ny = nr * nr;
    size_t nx = 1 + ny;
    assert(nr == size_t(arg.cols()));
    // -------------------------------------------------------------------
    // packed version of arg
    CPPAD_TESTVECTOR(ad_scalar) packed_arg(nx);
    packed_arg[0] = ad_scalar(nr);
    for (size_t i = 0; i < ny; i++) packed_arg[1 + i] = arg.data()[i];
    // -------------------------------------------------------------------
    // packed version of result = arg^{-1}.
    // This is an atomic_base function call that CppAD uses to
    // store the atomic operation on the tape.
    CPPAD_TESTVECTOR(ad_scalar) packed_result(ny);
    (*this)(packed_arg, packed_result);
    // -------------------------------------------------------------------
    // unpack result matrix
    ad_matrix result(nr, nr);
    for (size_t i = 0; i < ny; i++) result.data()[i] = packed_result[i];
    return result;
  }
  /* %$$
$head Private$$

$subhead Variables$$
$srccode%cpp% */
 private:
  // -------------------------------------------------------------
  // one forward mode vector of matrices for argument and result
  CppAD::vector<matrix> f_arg_, f_result_;
  // one reverse mode vector of matrices for argument and result
  CppAD::vector<matrix> r_arg_, r_result_;
  // -------------------------------------------------------------
  /* %$$
  $subhead forward$$
  $srccode%cpp% */
  // forward mode routine called by CppAD
  virtual bool forward(
      // lowest order Taylor coefficient we are evaluating
      size_t p,
      // highest order Taylor coefficient we are evaluating
      size_t q,
      // which components of x are variables
      const CppAD::vector<bool>& vx,
      // which components of y are variables
      CppAD::vector<bool>& vy,
      // tx [ j * (q+1) + k ] is x_j^k
      const CppAD::vector<scalar>& tx,
      // ty [ i * (q+1) + k ] is y_i^k
      CppAD::vector<scalar>& ty) {
    size_t n_order = q + 1;
    size_t nr = size_t(CppAD::Integer(tx[0 * n_order + 0]));
    size_t ny = nr * nr;
#ifndef NDEBUG
    size_t nx = 1 + ny;
#endif
    assert(vx.size() == 0 || nx == vx.size());
    assert(vx.size() == 0 || ny == vy.size());
    assert(nx * n_order == tx.size());
    assert(ny * n_order == ty.size());
    //
    // -------------------------------------------------------------------
    // make sure f_arg_ and f_result_ are large enough
    assert(f_arg_.size() == f_result_.size());
    if (f_arg_.size() < n_order) {
      f_arg_.resize(n_order);
      f_result_.resize(n_order);
      //
      for (size_t k = 0; k < n_order; k++) {
        f_arg_[k].resize(long(nr), long(nr));
        f_result_[k].resize(long(nr), long(nr));
      }
    }
    // -------------------------------------------------------------------
    // unpack tx into f_arg_
    for (size_t k = 0; k < n_order; k++) {  // unpack arg values for this order
      for (size_t i = 0; i < ny; i++)
        f_arg_[k].data()[i] = tx[(1 + i) * n_order + k];
    }
    // -------------------------------------------------------------------
    // result for each order
    // (we could avoid recalculting f_result_[k] for k=0,...,p-1)
    //
    f_result_[0] = f_arg_[0].inverse();
    for (size_t k = 1; k < n_order; k++) {  // initialize sum
      matrix f_sum = matrix::Zero(long(nr), long(nr));
      // compute sum
      for (size_t ell = 1; ell <= k; ell++)
        f_sum -= f_arg_[ell] * f_result_[k - ell];
      // result_[k] = arg_[0]^{-1} * sum_
      f_result_[k] = f_result_[0] * f_sum;
    }
    // -------------------------------------------------------------------
    // pack result_ into ty
    for (size_t k = 0; k < n_order; k++) {
      for (size_t i = 0; i < ny; i++)
        ty[i * n_order + k] = f_result_[k].data()[i];
    }
    // -------------------------------------------------------------------
    // check if we are computing vy
    if (vx.size() == 0) return true;
    // ------------------------------------------------------------------
    // This is a very dumb algorithm that over estimates which
    // elements of the inverse are variables (which is not efficient).
    bool var = false;
    for (size_t i = 0; i < ny; i++) var |= vx[1 + i];
    for (size_t i = 0; i < ny; i++) vy[i] = var;
    return true;
  }
  /* %$$
  $subhead reverse$$
  $srccode%cpp% */
  // reverse mode routine called by CppAD
  virtual bool reverse(
      // highest order Taylor coefficient that we are computing derivative of
      size_t q,
      // forward mode Taylor coefficients for x variables
      const CppAD::vector<Base>& tx,
      // forward mode Taylor coefficients for y variables
      const CppAD::vector<Base>& ty,
      // upon return, derivative of G[ F[ {x_j^k} ] ] w.r.t {x_j^k}
      CppAD::vector<Base>& px,
      // derivative of G[ {y_i^k} ] w.r.t. {y_i^k}
      const CppAD::vector<Base>& py) {
    size_t n_order = q + 1;
    size_t nr = size_t(CppAD::Integer(tx[0 * n_order + 0]));
    size_t ny = nr * nr;
#ifndef NDEBUG
    size_t nx = 1 + ny;
#endif
    //
    assert(nx * n_order == tx.size());
    assert(ny * n_order == ty.size());
    assert(px.size() == tx.size());
    assert(py.size() == ty.size());
    // -------------------------------------------------------------------
    // make sure f_arg_ is large enough
    assert(f_arg_.size() == f_result_.size());
    // must have previous run forward with order >= n_order
    assert(f_arg_.size() >= n_order);
    // -------------------------------------------------------------------
    // make sure r_arg_, r_result_ are large enough
    assert(r_arg_.size() == r_result_.size());
    if (r_arg_.size() < n_order) {
      r_arg_.resize(n_order);
      r_result_.resize(n_order);
      //
      for (size_t k = 0; k < n_order; k++) {
        r_arg_[k].resize(long(nr), long(nr));
        r_result_[k].resize(long(nr), long(nr));
      }
    }
    // -------------------------------------------------------------------
    // unpack tx into f_arg_
    for (size_t k = 0; k < n_order; k++) {  // unpack arg values for this order
      for (size_t i = 0; i < ny; i++)
        f_arg_[k].data()[i] = tx[(1 + i) * n_order + k];
    }
    // -------------------------------------------------------------------
    // unpack py into r_result_
    for (size_t k = 0; k < n_order; k++) {
      for (size_t i = 0; i < ny; i++)
        r_result_[k].data()[i] = py[i * n_order + k];
    }
    // -------------------------------------------------------------------
    // initialize r_arg_ as zero
    for (size_t k = 0; k < n_order; k++)
      r_arg_[k] = matrix::Zero(long(nr), long(nr));
    // -------------------------------------------------------------------
    // matrix reverse mode calculation
    //
    for (size_t k1 = n_order; k1 > 1; k1--) {
      size_t k = k1 - 1;
      // bar{R}_0 = bar{R}_0 + bar{R}_k (A_0 R_k)^T
      r_result_[0] +=
          r_result_[k] * f_result_[k].transpose() * f_arg_[0].transpose();
      //
      for (size_t ell = 1; ell <= k;
           ell++) {  // bar{A}_l = bar{A}_l - R_0^T bar{R}_k R_{k-l}^T
        r_arg_[ell] -= f_result_[0].transpose() * r_result_[k] *
                       f_result_[k - ell].transpose();
        // bar{R}_{k-l} = bar{R}_{k-1} - (R_0 A_l)^T bar{R}_k
        r_result_[k - ell] -=
            f_arg_[ell].transpose() * f_result_[0].transpose() * r_result_[k];
      }
    }
    r_arg_[0] -=
        f_result_[0].transpose() * r_result_[0] * f_result_[0].transpose();
    // -------------------------------------------------------------------
    // pack r_arg into px
    for (size_t k = 0; k < n_order; k++) {
      for (size_t i = 0; i < ny; i++)
        px[(1 + i) * n_order + k] = r_arg_[k].data()[i];
    }
    //
    return true;
  }
};
}  // namespace tds