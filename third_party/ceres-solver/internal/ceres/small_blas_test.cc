// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)

#include "ceres/small_blas.h"

#include <limits>

#include "ceres/internal/eigen.h"
#include "gtest/gtest.h"

namespace ceres {
namespace internal {

const double kTolerance = 3.0 * std::numeric_limits<double>::epsilon();

TEST(BLAS, MatrixMatrixMultiply) {
  const int kRowA = 3;
  const int kColA = 5;
  Matrix A(kRowA, kColA);
  A.setOnes();

  const int kRowB = 5;
  const int kColB = 7;
  Matrix B(kRowB, kColB);
  B.setOnes();

  for (int row_stride_c = kRowA; row_stride_c < 3 * kRowA; ++row_stride_c) {
    for (int col_stride_c = kColB; col_stride_c < 3 * kColB; ++col_stride_c) {
      Matrix C(row_stride_c, col_stride_c);
      C.setOnes();

      Matrix C_plus = C;
      Matrix C_minus = C;
      Matrix C_assign = C;

      Matrix C_plus_ref = C;
      Matrix C_minus_ref = C;
      Matrix C_assign_ref = C;
      // clang-format off
      for (int start_row_c = 0; start_row_c + kRowA < row_stride_c; ++start_row_c) {
        for (int start_col_c = 0; start_col_c + kColB < col_stride_c; ++start_col_c) {
          C_plus_ref.block(start_row_c, start_col_c, kRowA, kColB) +=
              A * B;

          MatrixMatrixMultiply<kRowA, kColA, kRowB, kColB, 1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_plus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_plus_ref - C_plus).norm(), 0.0, kTolerance)
              << "C += A * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_plus_ref << "\n"
              << "C: \n" << C_plus;

          C_minus_ref.block(start_row_c, start_col_c, kRowA, kColB) -=
              A * B;

          MatrixMatrixMultiply<kRowA, kColA, kRowB, kColB, -1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_minus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

           EXPECT_NEAR((C_minus_ref - C_minus).norm(), 0.0, kTolerance)
              << "C -= A * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_minus_ref << "\n"
              << "C: \n" << C_minus;

          C_assign_ref.block(start_row_c, start_col_c, kRowA, kColB) =
              A * B;

          MatrixMatrixMultiply<kRowA, kColA, kRowB, kColB, 0>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_assign.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_assign_ref - C_assign).norm(), 0.0, kTolerance)
              << "C = A * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_assign_ref << "\n"
              << "C: \n" << C_assign;
        }
      }
      // clang-format on
    }
  }
}

TEST(BLAS, MatrixTransposeMatrixMultiply) {
  const int kRowA = 5;
  const int kColA = 3;
  Matrix A(kRowA, kColA);
  A.setOnes();

  const int kRowB = 5;
  const int kColB = 7;
  Matrix B(kRowB, kColB);
  B.setOnes();

  for (int row_stride_c = kColA; row_stride_c < 3 * kColA; ++row_stride_c) {
    for (int col_stride_c = kColB; col_stride_c < 3 * kColB; ++col_stride_c) {
      Matrix C(row_stride_c, col_stride_c);
      C.setOnes();

      Matrix C_plus = C;
      Matrix C_minus = C;
      Matrix C_assign = C;

      Matrix C_plus_ref = C;
      Matrix C_minus_ref = C;
      Matrix C_assign_ref = C;
      // clang-format off
      for (int start_row_c = 0; start_row_c + kColA < row_stride_c; ++start_row_c) {
        for (int start_col_c = 0; start_col_c + kColB < col_stride_c; ++start_col_c) {
          C_plus_ref.block(start_row_c, start_col_c, kColA, kColB) +=
              A.transpose() * B;

          MatrixTransposeMatrixMultiply<kRowA, kColA, kRowB, kColB, 1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_plus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_plus_ref - C_plus).norm(), 0.0, kTolerance)
              << "C += A' * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_plus_ref << "\n"
              << "C: \n" << C_plus;

          C_minus_ref.block(start_row_c, start_col_c, kColA, kColB) -=
              A.transpose() * B;

          MatrixTransposeMatrixMultiply<kRowA, kColA, kRowB, kColB, -1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_minus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_minus_ref - C_minus).norm(), 0.0, kTolerance)
              << "C -= A' * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_minus_ref << "\n"
              << "C: \n" << C_minus;

          C_assign_ref.block(start_row_c, start_col_c, kColA, kColB) =
              A.transpose() * B;

          MatrixTransposeMatrixMultiply<kRowA, kColA, kRowB, kColB, 0>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_assign.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_assign_ref - C_assign).norm(), 0.0, kTolerance)
              << "C = A' * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_assign_ref << "\n"
              << "C: \n" << C_assign;
        }
      }
      // clang-format on
    }
  }
}

// TODO(sameeragarwal): Dedup and reduce the amount of duplication of
// test code in this file.

TEST(BLAS, MatrixMatrixMultiplyNaive) {
  const int kRowA = 3;
  const int kColA = 5;
  Matrix A(kRowA, kColA);
  A.setOnes();

  const int kRowB = 5;
  const int kColB = 7;
  Matrix B(kRowB, kColB);
  B.setOnes();

  for (int row_stride_c = kRowA; row_stride_c < 3 * kRowA; ++row_stride_c) {
    for (int col_stride_c = kColB; col_stride_c < 3 * kColB; ++col_stride_c) {
      Matrix C(row_stride_c, col_stride_c);
      C.setOnes();

      Matrix C_plus = C;
      Matrix C_minus = C;
      Matrix C_assign = C;

      Matrix C_plus_ref = C;
      Matrix C_minus_ref = C;
      Matrix C_assign_ref = C;
      // clang-format off
      for (int start_row_c = 0; start_row_c + kRowA < row_stride_c; ++start_row_c) {
        for (int start_col_c = 0; start_col_c + kColB < col_stride_c; ++start_col_c) {
          C_plus_ref.block(start_row_c, start_col_c, kRowA, kColB) +=
              A * B;

          MatrixMatrixMultiplyNaive<kRowA, kColA, kRowB, kColB, 1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_plus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_plus_ref - C_plus).norm(), 0.0, kTolerance)
              << "C += A * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_plus_ref << "\n"
              << "C: \n" << C_plus;

          C_minus_ref.block(start_row_c, start_col_c, kRowA, kColB) -=
              A * B;

          MatrixMatrixMultiplyNaive<kRowA, kColA, kRowB, kColB, -1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_minus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

           EXPECT_NEAR((C_minus_ref - C_minus).norm(), 0.0, kTolerance)
              << "C -= A * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_minus_ref << "\n"
              << "C: \n" << C_minus;

          C_assign_ref.block(start_row_c, start_col_c, kRowA, kColB) =
              A * B;

          MatrixMatrixMultiplyNaive<kRowA, kColA, kRowB, kColB, 0>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_assign.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_assign_ref - C_assign).norm(), 0.0, kTolerance)
              << "C = A * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_assign_ref << "\n"
              << "C: \n" << C_assign;
        }
      }
      // clang-format on
    }
  }
}

TEST(BLAS, MatrixTransposeMatrixMultiplyNaive) {
  const int kRowA = 5;
  const int kColA = 3;
  Matrix A(kRowA, kColA);
  A.setOnes();

  const int kRowB = 5;
  const int kColB = 7;
  Matrix B(kRowB, kColB);
  B.setOnes();

  for (int row_stride_c = kColA; row_stride_c < 3 * kColA; ++row_stride_c) {
    for (int col_stride_c = kColB; col_stride_c < 3 * kColB; ++col_stride_c) {
      Matrix C(row_stride_c, col_stride_c);
      C.setOnes();

      Matrix C_plus = C;
      Matrix C_minus = C;
      Matrix C_assign = C;

      Matrix C_plus_ref = C;
      Matrix C_minus_ref = C;
      Matrix C_assign_ref = C;
      // clang-format off
      for (int start_row_c = 0; start_row_c + kColA < row_stride_c; ++start_row_c) {
        for (int start_col_c = 0; start_col_c + kColB < col_stride_c; ++start_col_c) {
          C_plus_ref.block(start_row_c, start_col_c, kColA, kColB) +=
              A.transpose() * B;

          MatrixTransposeMatrixMultiplyNaive<kRowA, kColA, kRowB, kColB, 1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_plus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_plus_ref - C_plus).norm(), 0.0, kTolerance)
              << "C += A' * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_plus_ref << "\n"
              << "C: \n" << C_plus;

          C_minus_ref.block(start_row_c, start_col_c, kColA, kColB) -=
              A.transpose() * B;

          MatrixTransposeMatrixMultiplyNaive<kRowA, kColA, kRowB, kColB, -1>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_minus.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_minus_ref - C_minus).norm(), 0.0, kTolerance)
              << "C -= A' * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_minus_ref << "\n"
              << "C: \n" << C_minus;

          C_assign_ref.block(start_row_c, start_col_c, kColA, kColB) =
              A.transpose() * B;

          MatrixTransposeMatrixMultiplyNaive<kRowA, kColA, kRowB, kColB, 0>(
              A.data(), kRowA, kColA,
              B.data(), kRowB, kColB,
              C_assign.data(), start_row_c, start_col_c, row_stride_c, col_stride_c);

          EXPECT_NEAR((C_assign_ref - C_assign).norm(), 0.0, kTolerance)
              << "C = A' * B \n"
              << "row_stride_c : " << row_stride_c << "\n"
              << "col_stride_c : " << col_stride_c << "\n"
              << "start_row_c  : " << start_row_c << "\n"
              << "start_col_c  : " << start_col_c << "\n"
              << "Cref : \n" << C_assign_ref << "\n"
              << "C: \n" << C_assign;
        }
      }
      // clang-format on
    }
  }
}

TEST(BLAS, MatrixVectorMultiply) {
  for (int num_rows_a = 1; num_rows_a < 10; ++num_rows_a) {
    for (int num_cols_a = 1; num_cols_a < 10; ++num_cols_a) {
      Matrix A(num_rows_a, num_cols_a);
      A.setOnes();

      Vector b(num_cols_a);
      b.setOnes();

      Vector c(num_rows_a);
      c.setOnes();

      Vector c_plus = c;
      Vector c_minus = c;
      Vector c_assign = c;

      Vector c_plus_ref = c;
      Vector c_minus_ref = c;
      Vector c_assign_ref = c;

      // clang-format off
      c_plus_ref += A * b;
      MatrixVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 1>(
          A.data(), num_rows_a, num_cols_a,
          b.data(),
          c_plus.data());
      EXPECT_NEAR((c_plus_ref - c_plus).norm(), 0.0, kTolerance)
          << "c += A * b \n"
          << "c_ref : \n" << c_plus_ref << "\n"
          << "c: \n" << c_plus;

      c_minus_ref -= A * b;
      MatrixVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, -1>(
          A.data(), num_rows_a, num_cols_a,
          b.data(),
          c_minus.data());
      EXPECT_NEAR((c_minus_ref - c_minus).norm(), 0.0, kTolerance)
          << "c += A * b \n"
          << "c_ref : \n" << c_minus_ref << "\n"
          << "c: \n" << c_minus;

      c_assign_ref = A * b;
      MatrixVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 0>(
          A.data(), num_rows_a, num_cols_a,
          b.data(),
          c_assign.data());
      EXPECT_NEAR((c_assign_ref - c_assign).norm(), 0.0, kTolerance)
          << "c += A * b \n"
          << "c_ref : \n" << c_assign_ref << "\n"
          << "c: \n" << c_assign;
      // clang-format on
    }
  }
}

TEST(BLAS, MatrixTransposeVectorMultiply) {
  for (int num_rows_a = 1; num_rows_a < 10; ++num_rows_a) {
    for (int num_cols_a = 1; num_cols_a < 10; ++num_cols_a) {
      Matrix A(num_rows_a, num_cols_a);
      A.setRandom();

      Vector b(num_rows_a);
      b.setRandom();

      Vector c(num_cols_a);
      c.setOnes();

      Vector c_plus = c;
      Vector c_minus = c;
      Vector c_assign = c;

      Vector c_plus_ref = c;
      Vector c_minus_ref = c;
      Vector c_assign_ref = c;

      // clang-format off
      c_plus_ref += A.transpose() * b;
      MatrixTransposeVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 1>(
          A.data(), num_rows_a, num_cols_a,
          b.data(),
          c_plus.data());
      EXPECT_NEAR((c_plus_ref - c_plus).norm(), 0.0, kTolerance)
          << "c += A' * b \n"
          << "c_ref : \n" << c_plus_ref << "\n"
          << "c: \n" << c_plus;

      c_minus_ref -= A.transpose() * b;
      MatrixTransposeVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, -1>(
          A.data(), num_rows_a, num_cols_a,
          b.data(),
          c_minus.data());
      EXPECT_NEAR((c_minus_ref - c_minus).norm(), 0.0, kTolerance)
          << "c += A' * b \n"
          << "c_ref : \n" << c_minus_ref << "\n"
          << "c: \n" << c_minus;

      c_assign_ref = A.transpose() * b;
      MatrixTransposeVectorMultiply<Eigen::Dynamic, Eigen::Dynamic, 0>(
          A.data(), num_rows_a, num_cols_a,
          b.data(),
          c_assign.data());
      EXPECT_NEAR((c_assign_ref - c_assign).norm(), 0.0, kTolerance)
          << "c += A' * b \n"
          << "c_ref : \n" << c_assign_ref << "\n"
          << "c: \n" << c_assign;
      // clang-format on
    }
  }
}

}  // namespace internal
}  // namespace ceres
