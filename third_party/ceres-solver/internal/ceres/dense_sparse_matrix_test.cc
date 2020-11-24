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
//
// TODO(keir): Implement a generic "compare sparse matrix implementations" test
// suite that can compare all the implementations. Then this file would shrink
// in size.

#include "ceres/dense_sparse_matrix.h"

#include <memory>

#include "ceres/casts.h"
#include "ceres/internal/eigen.h"
#include "ceres/linear_least_squares_problems.h"
#include "ceres/triplet_sparse_matrix.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace ceres {
namespace internal {

static void CompareMatrices(const SparseMatrix* a, const SparseMatrix* b) {
  EXPECT_EQ(a->num_rows(), b->num_rows());
  EXPECT_EQ(a->num_cols(), b->num_cols());

  int num_rows = a->num_rows();
  int num_cols = a->num_cols();

  for (int i = 0; i < num_cols; ++i) {
    Vector x = Vector::Zero(num_cols);
    x(i) = 1.0;

    Vector y_a = Vector::Zero(num_rows);
    Vector y_b = Vector::Zero(num_rows);

    a->RightMultiply(x.data(), y_a.data());
    b->RightMultiply(x.data(), y_b.data());

    EXPECT_EQ((y_a - y_b).norm(), 0);
  }
}

class DenseSparseMatrixTest : public ::testing::Test {
 protected:
  void SetUp() final {
    std::unique_ptr<LinearLeastSquaresProblem> problem(
        CreateLinearLeastSquaresProblemFromId(1));

    CHECK(problem != nullptr);

    tsm.reset(down_cast<TripletSparseMatrix*>(problem->A.release()));
    dsm.reset(new DenseSparseMatrix(*tsm));

    num_rows = tsm->num_rows();
    num_cols = tsm->num_cols();
  }

  int num_rows;
  int num_cols;

  std::unique_ptr<TripletSparseMatrix> tsm;
  std::unique_ptr<DenseSparseMatrix> dsm;
};

TEST_F(DenseSparseMatrixTest, RightMultiply) {
  CompareMatrices(tsm.get(), dsm.get());

  // Try with a not entirely zero vector to verify column interactions, which
  // could be masked by a subtle bug when using the elementary vectors.
  Vector a(num_cols);
  for (int i = 0; i < num_cols; i++) {
    a(i) = i;
  }
  Vector b1 = Vector::Zero(num_rows);
  Vector b2 = Vector::Zero(num_rows);

  tsm->RightMultiply(a.data(), b1.data());
  dsm->RightMultiply(a.data(), b2.data());

  EXPECT_EQ((b1 - b2).norm(), 0);
}

TEST_F(DenseSparseMatrixTest, LeftMultiply) {
  for (int i = 0; i < num_rows; ++i) {
    Vector a = Vector::Zero(num_rows);
    a(i) = 1.0;

    Vector b1 = Vector::Zero(num_cols);
    Vector b2 = Vector::Zero(num_cols);

    tsm->LeftMultiply(a.data(), b1.data());
    dsm->LeftMultiply(a.data(), b2.data());

    EXPECT_EQ((b1 - b2).norm(), 0);
  }

  // Try with a not entirely zero vector to verify column interactions, which
  // could be masked by a subtle bug when using the elementary vectors.
  Vector a(num_rows);
  for (int i = 0; i < num_rows; i++) {
    a(i) = i;
  }
  Vector b1 = Vector::Zero(num_cols);
  Vector b2 = Vector::Zero(num_cols);

  tsm->LeftMultiply(a.data(), b1.data());
  dsm->LeftMultiply(a.data(), b2.data());

  EXPECT_EQ((b1 - b2).norm(), 0);
}

TEST_F(DenseSparseMatrixTest, ColumnNorm) {
  Vector b1 = Vector::Zero(num_cols);
  Vector b2 = Vector::Zero(num_cols);

  tsm->SquaredColumnNorm(b1.data());
  dsm->SquaredColumnNorm(b2.data());

  EXPECT_EQ((b1 - b2).norm(), 0);
}

TEST_F(DenseSparseMatrixTest, Scale) {
  Vector scale(num_cols);
  for (int i = 0; i < num_cols; ++i) {
    scale(i) = i + 1;
  }
  tsm->ScaleColumns(scale.data());
  dsm->ScaleColumns(scale.data());
  CompareMatrices(tsm.get(), dsm.get());
}

TEST_F(DenseSparseMatrixTest, ToDenseMatrix) {
  Matrix tsm_dense;
  Matrix dsm_dense;

  tsm->ToDenseMatrix(&tsm_dense);
  dsm->ToDenseMatrix(&dsm_dense);

  EXPECT_EQ((tsm_dense - dsm_dense).norm(), 0.0);
}

}  // namespace internal
}  // namespace ceres
