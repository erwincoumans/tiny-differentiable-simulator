/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _TINY_HELPER_H
#define _TINY_HELPER_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>

#include "tiny_matrix_x.h"

namespace TINY {

template <typename TinyScalar, typename TinyConstants,
          template <typename, typename> typename ColumnType>
static Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic> to_eigen(
    const TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType>& m) {
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic> result(m.m_rows,
                                                                   m.m_cols);
  for (int i = 0; i < m.m_rows; ++i) {
    for (int j = 0; j < m.m_cols; ++j) {
      result(i, j) = m(i, j);
    }
  }
  return result;
}

template <typename TinyScalar, typename TinyConstants>
static Eigen::Matrix<TinyScalar, Eigen::Dynamic, 1> to_eigen(
    const TinyVectorX<TinyScalar, TinyConstants>& v) {
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, 1> result(v.m_size, 1);
  for (int i = 0; i < v.m_size; ++i) {
    result(i, 0) = v[i];
  }
  return result;
}

template <typename TinyScalar, typename TinyConstants>
static TinyMatrixXxX<TinyScalar, TinyConstants> from_eigen(
    const Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic>& m) {
  TinyMatrixXxX<TinyScalar, TinyConstants> result(m.rows(), m.cols());
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) {
      result(i, j) = m(i, j);
    }
  }
  return result;
}

template <typename Algebra, int Cols>
static typename Algebra::VectorX  from_eigen_v(
    const Eigen::Matrix<typename Algebra::Scalar, Eigen::Dynamic, Cols>& m) {
  typename Algebra::VectorX result(m.rows());
  for (int i = 0; i < m.rows(); ++i) {
    result[i] = m(i, 0);
  }
  return result;
}

/**
 * Computes the inverse of a square matrix using Eigen.
 */
template <typename TinyScalar, typename TinyConstants>
static TinyMatrixXxX<TinyScalar, TinyConstants> inverse(
    const TinyMatrixXxX<TinyScalar, TinyConstants>& m) {
  TinyConstants::FullAssert(m.m_rows == m.m_cols);
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic> em = to_eigen(m);
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic> im = em.inverse();
  return from_eigen<TinyScalar, TinyConstants>(im);
}

/**
 * Computes the Moore-Penrose pseudo inverse using Eigen.
 */
template <typename TinyScalar, typename TinyConstants,
          template <typename, typename> typename ColumnType>
static TinyMatrixXxX<TinyScalar, TinyConstants> pseudo_inverse(
    const TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType>& m) {
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic> em = to_eigen(m);
  Eigen::CompleteOrthogonalDecomposition<
      Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic>>
      cod(em);
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic> im =
      cod.pseudoInverse();
  return from_eigen<TinyScalar, TinyConstants>(im);
}


template <typename Scalar>
static Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> pseudo_inverse(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& m) {

    Eigen::CompleteOrthogonalDecomposition<
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>
    cod(m);
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J_pinv =
    cod.pseudoInverse();
    return J_pinv;
}





/**
 * Solves V2 = L^-1 * V for input vector V,
 * given that this matrix is triangular so that only the lower triangular
 * matrix needs to be considered.
 */
template <typename TinyScalar, typename TinyConstants,
          template <typename, typename> typename ColumnType>
ColumnType<TinyScalar, TinyConstants> solve_triangular(
    const TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType>& m,
    const ColumnType<TinyScalar, TinyConstants>& v) {
  TinyConstants::FullAssert(m.m_rows == m.m_cols);
  TinyConstants::FullAssert(m.m_rows == v.m_size);
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, 1> ev(m.m_rows);
  for (int i = 0; i < m.m_rows; ++i) ev(i) = v[i];
  Eigen::Matrix<TinyScalar, Eigen::Dynamic, Eigen::Dynamic> em = to_eigen(m);
  em.template triangularView<Eigen::Lower>().solveInPlace(ev);
  ColumnType<TinyScalar, TinyConstants> result(m.m_rows);
  for (int i = 0; i < m.m_rows; ++i) result[i] = ev(i);
  return result;
}
};  // namespace TINY

#endif  // _TINY_HELPER_H
