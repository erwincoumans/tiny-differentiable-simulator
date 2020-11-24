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
// Author: sameeragarwal@google.com (Sameer Agarwal)

#include "ceres/compressed_row_sparse_matrix.h"

#include <memory>
#include <numeric>

#include "Eigen/SparseCore"
#include "ceres/casts.h"
#include "ceres/crs_matrix.h"
#include "ceres/internal/eigen.h"
#include "ceres/linear_least_squares_problems.h"
#include "ceres/random.h"
#include "ceres/triplet_sparse_matrix.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace ceres {
namespace internal {

using std::vector;

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

class CompressedRowSparseMatrixTest : public ::testing::Test {
 protected:
  void SetUp() final {
    std::unique_ptr<LinearLeastSquaresProblem> problem(
        CreateLinearLeastSquaresProblemFromId(1));

    CHECK(problem != nullptr);

    tsm.reset(down_cast<TripletSparseMatrix*>(problem->A.release()));
    crsm.reset(CompressedRowSparseMatrix::FromTripletSparseMatrix(*tsm));

    num_rows = tsm->num_rows();
    num_cols = tsm->num_cols();

    vector<int>* row_blocks = crsm->mutable_row_blocks();
    row_blocks->resize(num_rows);
    std::fill(row_blocks->begin(), row_blocks->end(), 1);

    vector<int>* col_blocks = crsm->mutable_col_blocks();
    col_blocks->resize(num_cols);
    std::fill(col_blocks->begin(), col_blocks->end(), 1);
  }

  int num_rows;
  int num_cols;

  std::unique_ptr<TripletSparseMatrix> tsm;
  std::unique_ptr<CompressedRowSparseMatrix> crsm;
};

TEST_F(CompressedRowSparseMatrixTest, Scale) {
  Vector scale(num_cols);
  for (int i = 0; i < num_cols; ++i) {
    scale(i) = i + 1;
  }

  tsm->ScaleColumns(scale.data());
  crsm->ScaleColumns(scale.data());
  CompareMatrices(tsm.get(), crsm.get());
}

TEST_F(CompressedRowSparseMatrixTest, DeleteRows) {
  // Clear the row and column blocks as these are purely scalar tests.
  crsm->mutable_row_blocks()->clear();
  crsm->mutable_col_blocks()->clear();

  for (int i = 0; i < num_rows; ++i) {
    tsm->Resize(num_rows - i, num_cols);
    crsm->DeleteRows(crsm->num_rows() - tsm->num_rows());
    CompareMatrices(tsm.get(), crsm.get());
  }
}

TEST_F(CompressedRowSparseMatrixTest, AppendRows) {
  // Clear the row and column blocks as these are purely scalar tests.
  crsm->mutable_row_blocks()->clear();
  crsm->mutable_col_blocks()->clear();

  for (int i = 0; i < num_rows; ++i) {
    TripletSparseMatrix tsm_appendage(*tsm);
    tsm_appendage.Resize(i, num_cols);

    tsm->AppendRows(tsm_appendage);
    std::unique_ptr<CompressedRowSparseMatrix> crsm_appendage(
        CompressedRowSparseMatrix::FromTripletSparseMatrix(tsm_appendage));

    crsm->AppendRows(*crsm_appendage);
    CompareMatrices(tsm.get(), crsm.get());
  }
}

TEST_F(CompressedRowSparseMatrixTest, AppendAndDeleteBlockDiagonalMatrix) {
  int num_diagonal_rows = crsm->num_cols();

  std::unique_ptr<double[]> diagonal(new double[num_diagonal_rows]);
  for (int i = 0; i < num_diagonal_rows; ++i) {
    diagonal[i] = i;
  }

  vector<int> row_and_column_blocks;
  row_and_column_blocks.push_back(1);
  row_and_column_blocks.push_back(2);
  row_and_column_blocks.push_back(2);

  const vector<int> pre_row_blocks = crsm->row_blocks();
  const vector<int> pre_col_blocks = crsm->col_blocks();

  std::unique_ptr<CompressedRowSparseMatrix> appendage(
      CompressedRowSparseMatrix::CreateBlockDiagonalMatrix(
          diagonal.get(), row_and_column_blocks));

  crsm->AppendRows(*appendage);

  const vector<int> post_row_blocks = crsm->row_blocks();
  const vector<int> post_col_blocks = crsm->col_blocks();

  vector<int> expected_row_blocks = pre_row_blocks;
  expected_row_blocks.insert(expected_row_blocks.end(),
                             row_and_column_blocks.begin(),
                             row_and_column_blocks.end());

  vector<int> expected_col_blocks = pre_col_blocks;

  EXPECT_EQ(expected_row_blocks, crsm->row_blocks());
  EXPECT_EQ(expected_col_blocks, crsm->col_blocks());

  crsm->DeleteRows(num_diagonal_rows);
  EXPECT_EQ(crsm->row_blocks(), pre_row_blocks);
  EXPECT_EQ(crsm->col_blocks(), pre_col_blocks);
}

TEST_F(CompressedRowSparseMatrixTest, ToDenseMatrix) {
  Matrix tsm_dense;
  Matrix crsm_dense;

  tsm->ToDenseMatrix(&tsm_dense);
  crsm->ToDenseMatrix(&crsm_dense);

  EXPECT_EQ((tsm_dense - crsm_dense).norm(), 0.0);
}

TEST_F(CompressedRowSparseMatrixTest, ToCRSMatrix) {
  CRSMatrix crs_matrix;
  crsm->ToCRSMatrix(&crs_matrix);
  EXPECT_EQ(crsm->num_rows(), crs_matrix.num_rows);
  EXPECT_EQ(crsm->num_cols(), crs_matrix.num_cols);
  EXPECT_EQ(crsm->num_rows() + 1, crs_matrix.rows.size());
  EXPECT_EQ(crsm->num_nonzeros(), crs_matrix.cols.size());
  EXPECT_EQ(crsm->num_nonzeros(), crs_matrix.values.size());

  for (int i = 0; i < crsm->num_rows() + 1; ++i) {
    EXPECT_EQ(crsm->rows()[i], crs_matrix.rows[i]);
  }

  for (int i = 0; i < crsm->num_nonzeros(); ++i) {
    EXPECT_EQ(crsm->cols()[i], crs_matrix.cols[i]);
    EXPECT_EQ(crsm->values()[i], crs_matrix.values[i]);
  }
}

TEST(CompressedRowSparseMatrix, CreateBlockDiagonalMatrix) {
  vector<int> blocks;
  blocks.push_back(1);
  blocks.push_back(2);
  blocks.push_back(2);

  Vector diagonal(5);
  for (int i = 0; i < 5; ++i) {
    diagonal(i) = i + 1;
  }

  std::unique_ptr<CompressedRowSparseMatrix> matrix(
      CompressedRowSparseMatrix::CreateBlockDiagonalMatrix(diagonal.data(),
                                                           blocks));

  EXPECT_EQ(matrix->num_rows(), 5);
  EXPECT_EQ(matrix->num_cols(), 5);
  EXPECT_EQ(matrix->num_nonzeros(), 9);
  EXPECT_EQ(blocks, matrix->row_blocks());
  EXPECT_EQ(blocks, matrix->col_blocks());

  Vector x(5);
  Vector y(5);

  x.setOnes();
  y.setZero();
  matrix->RightMultiply(x.data(), y.data());
  for (int i = 0; i < diagonal.size(); ++i) {
    EXPECT_EQ(y[i], diagonal[i]);
  }

  y.setZero();
  matrix->LeftMultiply(x.data(), y.data());
  for (int i = 0; i < diagonal.size(); ++i) {
    EXPECT_EQ(y[i], diagonal[i]);
  }

  Matrix dense;
  matrix->ToDenseMatrix(&dense);
  EXPECT_EQ((dense.diagonal() - diagonal).norm(), 0.0);
}

TEST(CompressedRowSparseMatrix, Transpose) {
  //  0  1  0  2  3  0
  //  4  6  7  0  0  8
  //  9 10  0 11 12  0
  // 13  0 14 15  9  0
  //  0 16 17  0  0  0

  // Block structure:
  //  A  A  A  A  B  B
  //  A  A  A  A  B  B
  //  A  A  A  A  B  B
  //  C  C  C  C  D  D
  //  C  C  C  C  D  D
  //  C  C  C  C  D  D

  CompressedRowSparseMatrix matrix(5, 6, 30);
  int* rows = matrix.mutable_rows();
  int* cols = matrix.mutable_cols();
  double* values = matrix.mutable_values();
  matrix.mutable_row_blocks()->push_back(3);
  matrix.mutable_row_blocks()->push_back(3);
  matrix.mutable_col_blocks()->push_back(4);
  matrix.mutable_col_blocks()->push_back(2);

  rows[0] = 0;
  cols[0] = 1;
  cols[1] = 3;
  cols[2] = 4;

  rows[1] = 3;
  cols[3] = 0;
  cols[4] = 1;
  cols[5] = 2;
  cols[6] = 5;

  rows[2] = 7;
  cols[7] = 0;
  cols[8] = 1;
  cols[9] = 3;
  cols[10] = 4;

  rows[3] = 11;
  cols[11] = 0;
  cols[12] = 2;
  cols[13] = 3;
  cols[14] = 4;

  rows[4] = 15;
  cols[15] = 1;
  cols[16] = 2;
  rows[5] = 17;

  std::copy(values, values + 17, cols);

  std::unique_ptr<CompressedRowSparseMatrix> transpose(matrix.Transpose());

  ASSERT_EQ(transpose->row_blocks().size(), matrix.col_blocks().size());
  for (int i = 0; i < transpose->row_blocks().size(); ++i) {
    EXPECT_EQ(transpose->row_blocks()[i], matrix.col_blocks()[i]);
  }

  ASSERT_EQ(transpose->col_blocks().size(), matrix.row_blocks().size());
  for (int i = 0; i < transpose->col_blocks().size(); ++i) {
    EXPECT_EQ(transpose->col_blocks()[i], matrix.row_blocks()[i]);
  }

  Matrix dense_matrix;
  matrix.ToDenseMatrix(&dense_matrix);

  Matrix dense_transpose;
  transpose->ToDenseMatrix(&dense_transpose);
  EXPECT_NEAR((dense_matrix - dense_transpose.transpose()).norm(), 0.0, 1e-14);
}

TEST(CompressedRowSparseMatrix, FromTripletSparseMatrix) {
  TripletSparseMatrix::RandomMatrixOptions options;
  options.num_rows = 5;
  options.num_cols = 7;
  options.density = 0.5;

  const int kNumTrials = 10;
  for (int i = 0; i < kNumTrials; ++i) {
    std::unique_ptr<TripletSparseMatrix> tsm(
        TripletSparseMatrix::CreateRandomMatrix(options));
    std::unique_ptr<CompressedRowSparseMatrix> crsm(
        CompressedRowSparseMatrix::FromTripletSparseMatrix(*tsm));

    Matrix expected;
    tsm->ToDenseMatrix(&expected);
    Matrix actual;
    crsm->ToDenseMatrix(&actual);
    EXPECT_NEAR((expected - actual).norm() / actual.norm(),
                0.0,
                std::numeric_limits<double>::epsilon())
        << "\nexpected: \n"
        << expected << "\nactual: \n"
        << actual;
  }
}

TEST(CompressedRowSparseMatrix, FromTripletSparseMatrixTransposed) {
  TripletSparseMatrix::RandomMatrixOptions options;
  options.num_rows = 5;
  options.num_cols = 7;
  options.density = 0.5;

  const int kNumTrials = 10;
  for (int i = 0; i < kNumTrials; ++i) {
    std::unique_ptr<TripletSparseMatrix> tsm(
        TripletSparseMatrix::CreateRandomMatrix(options));
    std::unique_ptr<CompressedRowSparseMatrix> crsm(
        CompressedRowSparseMatrix::FromTripletSparseMatrixTransposed(*tsm));

    Matrix tmp;
    tsm->ToDenseMatrix(&tmp);
    Matrix expected = tmp.transpose();
    Matrix actual;
    crsm->ToDenseMatrix(&actual);
    EXPECT_NEAR((expected - actual).norm() / actual.norm(),
                0.0,
                std::numeric_limits<double>::epsilon())
        << "\nexpected: \n"
        << expected << "\nactual: \n"
        << actual;
  }
}

typedef ::testing::tuple<CompressedRowSparseMatrix::StorageType> Param;

static std::string ParamInfoToString(testing::TestParamInfo<Param> info) {
  if (::testing::get<0>(info.param) ==
      CompressedRowSparseMatrix::UPPER_TRIANGULAR) {
    return "UPPER";
  }

  if (::testing::get<0>(info.param) ==
      CompressedRowSparseMatrix::LOWER_TRIANGULAR) {
    return "LOWER";
  }

  return "UNSYMMETRIC";
}

class RightMultiplyTest : public ::testing::TestWithParam<Param> {};

TEST_P(RightMultiplyTest, _) {
  const int kMinNumBlocks = 1;
  const int kMaxNumBlocks = 10;
  const int kMinBlockSize = 1;
  const int kMaxBlockSize = 5;
  const int kNumTrials = 10;

  for (int num_blocks = kMinNumBlocks; num_blocks < kMaxNumBlocks;
       ++num_blocks) {
    for (int trial = 0; trial < kNumTrials; ++trial) {
      Param param = GetParam();
      CompressedRowSparseMatrix::RandomMatrixOptions options;
      options.num_col_blocks = num_blocks;
      options.min_col_block_size = kMinBlockSize;
      options.max_col_block_size = kMaxBlockSize;
      options.num_row_blocks = 2 * num_blocks;
      options.min_row_block_size = kMinBlockSize;
      options.max_row_block_size = kMaxBlockSize;
      options.block_density = std::max(0.5, RandDouble());
      options.storage_type = ::testing::get<0>(param);
      std::unique_ptr<CompressedRowSparseMatrix> matrix(
          CompressedRowSparseMatrix::CreateRandomMatrix(options));
      const int num_rows = matrix->num_rows();
      const int num_cols = matrix->num_cols();

      Vector x(num_cols);
      x.setRandom();

      Vector actual_y(num_rows);
      actual_y.setZero();
      matrix->RightMultiply(x.data(), actual_y.data());

      Matrix dense;
      matrix->ToDenseMatrix(&dense);
      Vector expected_y;
      if (::testing::get<0>(param) ==
          CompressedRowSparseMatrix::UPPER_TRIANGULAR) {
        expected_y = dense.selfadjointView<Eigen::Upper>() * x;
      } else if (::testing::get<0>(param) ==
                 CompressedRowSparseMatrix::LOWER_TRIANGULAR) {
        expected_y = dense.selfadjointView<Eigen::Lower>() * x;
      } else {
        expected_y = dense * x;
      }

      ASSERT_NEAR((expected_y - actual_y).norm() / actual_y.norm(),
                  0.0,
                  std::numeric_limits<double>::epsilon() * 10)
          << "\n"
          << dense << "x:\n"
          << x.transpose() << "\n"
          << "expected: \n"
          << expected_y.transpose() << "\n"
          << "actual: \n"
          << actual_y.transpose();
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    CompressedRowSparseMatrix,
    RightMultiplyTest,
    ::testing::Values(CompressedRowSparseMatrix::LOWER_TRIANGULAR,
                      CompressedRowSparseMatrix::UPPER_TRIANGULAR,
                      CompressedRowSparseMatrix::UNSYMMETRIC),
    ParamInfoToString);

class LeftMultiplyTest : public ::testing::TestWithParam<Param> {};

TEST_P(LeftMultiplyTest, _) {
  const int kMinNumBlocks = 1;
  const int kMaxNumBlocks = 10;
  const int kMinBlockSize = 1;
  const int kMaxBlockSize = 5;
  const int kNumTrials = 10;

  for (int num_blocks = kMinNumBlocks; num_blocks < kMaxNumBlocks;
       ++num_blocks) {
    for (int trial = 0; trial < kNumTrials; ++trial) {
      Param param = GetParam();
      CompressedRowSparseMatrix::RandomMatrixOptions options;
      options.num_col_blocks = num_blocks;
      options.min_col_block_size = kMinBlockSize;
      options.max_col_block_size = kMaxBlockSize;
      options.num_row_blocks = 2 * num_blocks;
      options.min_row_block_size = kMinBlockSize;
      options.max_row_block_size = kMaxBlockSize;
      options.block_density = std::max(0.5, RandDouble());
      options.storage_type = ::testing::get<0>(param);
      std::unique_ptr<CompressedRowSparseMatrix> matrix(
          CompressedRowSparseMatrix::CreateRandomMatrix(options));
      const int num_rows = matrix->num_rows();
      const int num_cols = matrix->num_cols();

      Vector x(num_rows);
      x.setRandom();

      Vector actual_y(num_cols);
      actual_y.setZero();
      matrix->LeftMultiply(x.data(), actual_y.data());

      Matrix dense;
      matrix->ToDenseMatrix(&dense);
      Vector expected_y;
      if (::testing::get<0>(param) ==
          CompressedRowSparseMatrix::UPPER_TRIANGULAR) {
        expected_y = dense.selfadjointView<Eigen::Upper>() * x;
      } else if (::testing::get<0>(param) ==
                 CompressedRowSparseMatrix::LOWER_TRIANGULAR) {
        expected_y = dense.selfadjointView<Eigen::Lower>() * x;
      } else {
        expected_y = dense.transpose() * x;
      }

      ASSERT_NEAR((expected_y - actual_y).norm() / actual_y.norm(),
                  0.0,
                  std::numeric_limits<double>::epsilon() * 10)
          << "\n"
          << dense << "x\n"
          << x.transpose() << "\n"
          << "expected: \n"
          << expected_y.transpose() << "\n"
          << "actual: \n"
          << actual_y.transpose();
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    CompressedRowSparseMatrix,
    LeftMultiplyTest,
    ::testing::Values(CompressedRowSparseMatrix::LOWER_TRIANGULAR,
                      CompressedRowSparseMatrix::UPPER_TRIANGULAR,
                      CompressedRowSparseMatrix::UNSYMMETRIC),
    ParamInfoToString);

class SquaredColumnNormTest : public ::testing::TestWithParam<Param> {};

TEST_P(SquaredColumnNormTest, _) {
  const int kMinNumBlocks = 1;
  const int kMaxNumBlocks = 10;
  const int kMinBlockSize = 1;
  const int kMaxBlockSize = 5;
  const int kNumTrials = 10;

  for (int num_blocks = kMinNumBlocks; num_blocks < kMaxNumBlocks;
       ++num_blocks) {
    for (int trial = 0; trial < kNumTrials; ++trial) {
      Param param = GetParam();
      CompressedRowSparseMatrix::RandomMatrixOptions options;
      options.num_col_blocks = num_blocks;
      options.min_col_block_size = kMinBlockSize;
      options.max_col_block_size = kMaxBlockSize;
      options.num_row_blocks = 2 * num_blocks;
      options.min_row_block_size = kMinBlockSize;
      options.max_row_block_size = kMaxBlockSize;
      options.block_density = std::max(0.5, RandDouble());
      options.storage_type = ::testing::get<0>(param);
      std::unique_ptr<CompressedRowSparseMatrix> matrix(
          CompressedRowSparseMatrix::CreateRandomMatrix(options));
      const int num_cols = matrix->num_cols();

      Vector actual(num_cols);
      actual.setZero();
      matrix->SquaredColumnNorm(actual.data());

      Matrix dense;
      matrix->ToDenseMatrix(&dense);
      Vector expected;
      if (::testing::get<0>(param) ==
          CompressedRowSparseMatrix::UPPER_TRIANGULAR) {
        const Matrix full = dense.selfadjointView<Eigen::Upper>();
        expected = full.colwise().squaredNorm();
      } else if (::testing::get<0>(param) ==
                 CompressedRowSparseMatrix::LOWER_TRIANGULAR) {
        const Matrix full = dense.selfadjointView<Eigen::Lower>();
        expected = full.colwise().squaredNorm();
      } else {
        expected = dense.colwise().squaredNorm();
      }

      ASSERT_NEAR((expected - actual).norm() / actual.norm(),
                  0.0,
                  std::numeric_limits<double>::epsilon() * 10)
          << "\n"
          << dense << "expected: \n"
          << expected.transpose() << "\n"
          << "actual: \n"
          << actual.transpose();
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    CompressedRowSparseMatrix,
    SquaredColumnNormTest,
    ::testing::Values(CompressedRowSparseMatrix::LOWER_TRIANGULAR,
                      CompressedRowSparseMatrix::UPPER_TRIANGULAR,
                      CompressedRowSparseMatrix::UNSYMMETRIC),
    ParamInfoToString);

// TODO(sameeragarwal) Add tests for the random matrix creation methods.

}  // namespace internal
}  // namespace ceres
