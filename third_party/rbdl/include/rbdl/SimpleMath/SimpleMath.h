/*
 * SimpleMath - A simple highly inefficient single header C++ math library
 * Copyright (c) 2019 Martin Felis <martin@fysx.org>
 *
 * This is a highly inefficient math library. It was conceived while he was
 * waiting for code to compile which used a highly efficient math library.
 * 
 * It is intended to be used as a fast compiling substitute for the
 * blazingly fast Eigen3
 * http://eigen.tuxfamily.org/index.php?title=Main_Page library and tries
 * to mimic its API to a certain extent.
 * 
 * Feel free to use it wherever you like (even claim it as yours!). However,
 * no guarantees are given that this code does what it says it would.
 *
 * Should you need a more formal license go with the following (zlib license):
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
*/

#pragma once

#include <sstream>
#include <cassert>
#include <iostream>
#include <cstring> // for memcpy
#include <cmath>
#include <limits>
#include <type_traits>

namespace SimpleMath {

//
// Forward Declarations
//
enum {
        Dynamic = -1
};

template <typename ScalarType, int NumRows = Dynamic, int NumCols = Dynamic>
struct Matrix;

template <typename Derived>
struct CommaInitializer;

template <typename Derived, typename ScalarType, int NumRows = -1, int NumCols = -1>
struct Block;

template <typename Derived, typename ScalarType, int NumRows, int NumCols>
struct Transpose;

typedef Matrix<float, 3, 3> Matrix33f;
typedef Matrix<float, 3, 1> Vector3f;

template <typename Derived>
class LLT;

template <typename Derived>
class PartialPivLU;

template <typename Derived>
class HouseholderQR;

template <typename Derived>
class ColPivHouseholderQR;


//
// Main MatrixBase class which defines all functions available on the
// derived matrix types.
//
template <typename Derived, typename ScalarType, int Rows, int Cols>
struct MatrixBase {
  typedef MatrixBase<Derived, ScalarType, Rows, Cols> MatrixType;
  typedef ScalarType value_type;

  enum {
    RowsAtCompileTime = Rows,
    ColsAtCompileTime = Cols
  };


  Derived& operator=(const Derived& other) {
    if (static_cast<const void*>(this) != static_cast<const void*>(&other)) {
      int i, j, in = other.rows(), jn = other.cols();

      for (size_t i = 0; i < in; i++) {
        for (size_t j = 0; j < jn; j++) {
          this->operator()(i,j) = other(i,j);
        }
      }
    }

    return *this;
  }


  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Derived& operator=(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) {
    if (static_cast<const void*>(this) != static_cast<const void*>(&other)) {
      int i, j, in = other.rows(), jn = other.cols();

      for (size_t i = 0; i < in; i++) {
        for (size_t j = 0; j < jn; j++) {
          this->operator()(i,j) = other(i,j);
        }
      }
    }

    return *this;
  }

  //
  // operators with scalars
  //
  Matrix<ScalarType, Rows, Cols> operator*(const double& scalar) const {
    Matrix<ScalarType, Rows, Cols> result (rows(), cols());
    int i, j, in = rows(), jn = cols();

    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        result (i,j) = operator()(i,j) * static_cast<ScalarType>(scalar);
        }
    }
    return result;
  }

  Matrix<ScalarType, Rows, Cols> operator*(const float& scalar) const {
    Matrix<ScalarType, Rows, Cols> result (rows(), cols());
    int i, j, in = rows(), jn = cols();

    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        result (i,j) = operator()(i,j) * static_cast<ScalarType>(scalar);
      }
    }
    return result;
  }

  //
  // operators with other matrices
  //
  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  bool operator==(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) const {
    int i, j, in = rows(), jn = cols();

    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        if (this->operator()(i,j) != other(i,j))
          return false;
      }
    }
    
    return true;
  }

  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  bool operator!=(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) const {
    return !(operator==(other));
  }

  CommaInitializer<Derived> operator<< (const value_type& value) {
    return CommaInitializer<Derived> (*(static_cast<Derived*>(this)), value);
  }
  
  template <typename OtherDerived>
  Derived operator+(const OtherDerived& other) const {
    Derived result (*(static_cast<const Derived*>(this)));
    result += other;
    return result;
  }

  template <typename OtherDerived>
  Derived operator-(const OtherDerived& other) {
    Derived result (*(static_cast<Derived*>(this)));
    result -= other;
    return result;
  }

  template <typename OtherDerived>
  Derived operator-(const OtherDerived& other) const {
    Derived result (*(static_cast<const Derived*>(this)));
    result -= other;
    return result;
  }

  template<typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Matrix<ScalarType, Rows, OtherCols>
  operator*(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols> &other) const {
    Matrix<ScalarType, Rows, OtherCols> result(Matrix<ScalarType, Rows, OtherCols>::Zero(rows(), other.cols()));
    int i, j, k, in = rows(), jn = other.cols(), kn = other.rows();

    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        for (k = 0; k < kn; k++) {
          result(i, j) += operator()(i, k) * other(k, j);
        }
      }
    }

    return result;
  }

  template <typename OtherDerived>
  Derived operator*=(const MatrixBase<OtherDerived, typename OtherDerived:: value_type, OtherDerived::RowsAtCompileTime, OtherDerived::ColsAtCompileTime> &other) {
    Derived copy (*static_cast<const Derived*>(this));
    this->setZero();

    int i, j, k, in = rows(), jn = other.cols(), kn = other.rows();

    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        for (k = 0; k < kn; k++) {
          this->operator()(i, j) += copy.operator()(i, k) * other(k, j);
        }
      }
    }
    return *this;
  }

  Matrix<ScalarType, Rows, Cols> operator-() const {
    Matrix<ScalarType, Rows, Cols> copy (*static_cast<const Derived*>(this));
    int i, j, in = rows(), jn = cols();

    for (int i = 0; i < in; i++) {
      for (int j = 0; j < jn; j++) {
        copy(i,j) *= static_cast<ScalarType>(-1.);
      }
    }
    return copy;
  }

  Derived operator*=(const ScalarType& s) {
    int i, j, in = rows(), jn = cols();

    for (int i = 0; i < in; i++) {
      for (int j = 0; j < jn; j++) {
        operator()(i,j) *= s;
      }
    }

    return *this;
  }

  void resize(unsigned int nrows, unsigned int ncols = 1) {
    static_assert(Rows == Dynamic, "Resize of fixed size matrices not allowed.");

    // Resize the this matrix (so far only possible for subclasses of the
    // Matrix class)
    Matrix<ScalarType, Rows, Cols>* this_matrix = static_cast<Matrix<ScalarType, Rows, Cols>*>(this);
    this_matrix->mStorage.resize(nrows, ncols);
  }

  void conservativeResize(unsigned int nrows, unsigned int ncols = 1) {
    static_assert(Rows == Dynamic, "Resize of fixed size matrices not allowed.");

    Derived copy(*this);

    unsigned int arows = std::min(nrows, (unsigned int) rows());
    unsigned int acols = std::min(ncols, (unsigned int) cols());

    resize(nrows, ncols);
    setZero();

    // TODO: set entries to zero within the loop
    for (unsigned int i = 0; i < arows; i++) {
      for (unsigned int j = 0; j < acols; j++) {
        this->operator()(i, j) = copy(i,j);
      }
    }
  }

  void setZero() {
    int nrows = rows();
    int ncols = cols();
    for (int i = 0; i < nrows; i++) {
      for (int j = 0; j < ncols; j++) {
        operator()(i,j) = static_cast<ScalarType>(0.0);
      }
    }
  }

  void set(const ScalarType& v0) {
    static_assert(cols() * rows() == 1, "Invalid matrix size");
    data()[0] = v0;
  }

  void set(
      const ScalarType& v0,
      const ScalarType& v1,
      const ScalarType& v2
      ) {
    assert(cols() * rows() == 3);

    data()[0] = v0;
    data()[1] = v1;
    data()[2] = v2;
  }

  void set(
      const ScalarType& v0,
      const ScalarType& v1,
      const ScalarType& v2,
      const ScalarType& v3
      ) {
    assert(cols() * rows() == 4);

    data()[0] = v0;
    data()[1] = v1;
    data()[2] = v2;
    data()[3] = v3;
  }

  void set(
      const ScalarType& v0,
      const ScalarType& v1,
      const ScalarType& v2,
      const ScalarType& v3,
      const ScalarType& v4,
      const ScalarType& v5
      ) {
    assert(cols() * rows() == 6);

    data()[0] = v0;
    data()[1] = v1;
    data()[2] = v2;
    data()[3] = v3;
    data()[4] = v4;
    data()[5] = v5;
  }

  void set(
      const ScalarType& v00,
      const ScalarType& v01,
      const ScalarType& v02,
      const ScalarType& v03,
      const ScalarType& v04,
      const ScalarType& v05,

      const ScalarType& v10,
      const ScalarType& v11,
      const ScalarType& v12,
      const ScalarType& v13,
      const ScalarType& v14,
      const ScalarType& v15,

      const ScalarType& v20,
      const ScalarType& v21,
      const ScalarType& v22,
      const ScalarType& v23,
      const ScalarType& v24,
      const ScalarType& v25,

      const ScalarType& v30,
      const ScalarType& v31,
      const ScalarType& v32,
      const ScalarType& v33,
      const ScalarType& v34,
      const ScalarType& v35,

      const ScalarType& v40,
      const ScalarType& v41,
      const ScalarType& v42,
      const ScalarType& v43,
      const ScalarType& v44,
      const ScalarType& v45,

      const ScalarType& v50,
      const ScalarType& v51,
      const ScalarType& v52,
      const ScalarType& v53,
      const ScalarType& v54,
      const ScalarType& v55
      ) {
    assert(cols() == 6 && rows() == 6);

    operator()(0,0) = v00;
    operator()(0,1) = v01;
    operator()(0,2) = v02;
    operator()(0,3) = v03;
    operator()(0,4) = v04;
    operator()(0,5) = v05;

    operator()(1,0) = v10;
    operator()(1,1) = v11;
    operator()(1,2) = v12;
    operator()(1,3) = v13;
    operator()(1,4) = v14;
    operator()(1,5) = v15;

    operator()(2,0) = v20;
    operator()(2,1) = v21;
    operator()(2,2) = v22;
    operator()(2,3) = v23;
    operator()(2,4) = v24;
    operator()(2,5) = v25;

    operator()(3,0) = v30;
    operator()(3,1) = v31;
    operator()(3,2) = v32;
    operator()(3,3) = v33;
    operator()(3,4) = v34;
    operator()(3,5) = v35;

    operator()(4,0) = v40;
    operator()(4,1) = v41;
    operator()(4,2) = v42;
    operator()(4,3) = v43;
    operator()(4,4) = v44;
    operator()(4,5) = v45;

    operator()(5,0) = v50;
    operator()(5,1) = v51;
    operator()(5,2) = v52;
    operator()(5,3) = v53;
    operator()(5,4) = v54;
    operator()(5,5) = v55;
   }

  size_t rows() const {
    return static_cast<const Derived*>(this)->rows();
  }

  size_t cols() const {
    return static_cast<const Derived*>(this)->cols();
  }

  size_t size() const {
    return static_cast<const Derived*>(this)->rows() * static_cast<const Derived*>(this)->cols();
  }

  const ScalarType& operator()(const size_t& i, const size_t& j) const {
    return static_cast<const Derived*>(this)->operator()(i,j);
  }
  ScalarType& operator()(const size_t& i, const size_t& j) {
    return static_cast<Derived*>(this)->operator()(i,j);
  }

  const ScalarType& operator[](const size_t& i) const {
    assert(cols() == 1);
    return static_cast<const Derived*>(this)->operator()(i,0);
  }
  ScalarType& operator[](const size_t& i) {
    assert(cols() == 1);
    return static_cast<Derived*>(this)->operator()(i,0);
  }

  operator ScalarType() const {
#ifndef NDEBUG
    if ( static_cast<const Derived*>(this)->cols() != 1
        || static_cast<const Derived*>(this)->rows() != 1) {
      std::cout << "Error trying to cast to scalar type. Dimensions are: "
        << static_cast<const Derived*>(this)->rows() << ", "  
        << static_cast<const Derived*>(this)->cols() << "."  
        << std::endl;
    }
#endif
    assert ( static_cast<const Derived*>(this)->cols() == 1
        && static_cast<const Derived*>(this)->rows() == 1);
    return static_cast<const Derived*>(this)->operator()(0,0);
  }

  //
  // Numerical functions
  //

  // TODO: separate functions for float or ScalarType matrices  
  ScalarType dot(const Derived& other) const {
    assert ((rows() == 1 || cols() == 1) && (other.rows() == 1 || other.cols() == 1));

    ScalarType result = 0.0;

    size_t n = rows() * cols();
    for (size_t i = 0; i < n; ++i) {
      result += operator[](i) * other[i];
    }

    return result;
  }

  // TODO: separate functions for float or ScalarType matrices  
  ScalarType squaredNorm() const {
    ScalarType result = static_cast<ScalarType>(0.0);

    size_t nr = rows();
    size_t nc = cols();
    for (size_t i = 0; i < nr; ++i) {
      for (size_t j = 0; j < nc; ++j) {
        result += operator()(i, j) * operator()(i, j);
      }
    }

    return result;
  }

  // TODO: separate functions for float or ScalarType matrices  
  ScalarType norm() const {
    return static_cast<ScalarType>(std::sqrt(squaredNorm()));
  }
  
  // TODO: separate functions for float or ScalarType matrices  
  Derived normalized() const {
    Derived result (*this);

    ScalarType length = this->norm();

    return result / length;
  }

  // TODO: separate functions for float or ScalarType matrices  
  Derived normalize() {
    ScalarType length = norm();

    *this *= static_cast<ScalarType>(1.0) / length;

    return *this;
  }

  Derived cross(const Derived& other) const {
    assert(cols() * rows() == 3);
    
    Derived result(rows(), cols());
    result[0] = operator[](1) * other[2] - operator[](2) * other[1];
    result[1] = operator[](2) * other[0] - operator[](0) * other[2];
    result[2] = operator[](0) * other[1] - operator[](1) * other[0];

    return result;
  }

  Derived inverse() const {
    if (rows() == cols()) {
       if (rows() == 1) {
        Derived result(rows(), cols());
        result(0,0) = static_cast<ScalarType>(1.) / operator()(0,0);

        return result;
       } else if (rows() == 2) {
        const ScalarType& a = operator()(0,0);
        const ScalarType& b = operator()(0,1);
        const ScalarType& c = operator()(1,0);
        const ScalarType& d = operator()(1,1);

        Derived result(rows(), cols());

        ScalarType detinv = static_cast<ScalarType>(1.) / (a * d - b * c);

        result(0,0) = d * detinv;
        result(0,1) = -b * detinv;
        result(1,0) = -c * detinv;
        result(1,1) = d * detinv;
       
        return result;
      } else if (rows() == 3) {
        // source:
        // https://stackoverflow.com/questions/983999/simple-3x3-matrix-inverse-code-c
      
        // computes the inverse of a matrix m
        ScalarType det = operator()(0, 0) * (operator()(1, 1) * operator()(2, 2)
            - operator()(2, 1) * operator()(1, 2))
            - operator()(0, 1) * (operator()(1, 0) * operator()(2, 2) 
            - operator()(1, 2) * operator()(2, 0)) 
            + operator()(0, 2) * (operator()(1, 0) * operator()(2, 1)
            - operator()(1, 1) * operator()(2, 0));

        ScalarType invdet = 1. / det;

        Derived result(rows(), cols());

        result(0,0) = (operator()(1, 1) * operator()(2, 2) - operator()(2, 1) * operator()(1, 2)) * invdet;
        result(0,1) = (operator()(0, 2) * operator()(2, 1) - operator()(0, 1) * operator()(2, 2)) * invdet;
        result(0,2) = (operator()(0, 1) * operator()(1, 2) - operator()(0, 2) * operator()(1, 1)) * invdet;
        result(1,0) = (operator()(1, 2) * operator()(2, 0) - operator()(1, 0) * operator()(2, 2)) * invdet;
        result(1,1) = (operator()(0, 0) * operator()(2, 2) - operator()(0, 2) * operator()(2, 0)) * invdet;
        result(1,2) = (operator()(1, 0) * operator()(0, 2) - operator()(0, 0) * operator()(1, 2)) * invdet;
        result(2,0) = (operator()(1, 0) * operator()(2, 1) - operator()(2, 0) * operator()(1, 1)) * invdet;
        result(2,1) = (operator()(2, 0) * operator()(0, 1) - operator()(0, 0) * operator()(2, 1)) * invdet;
        result(2,2) = (operator()(0, 0) * operator()(1, 1) - operator()(1, 0) * operator()(0, 1)) * invdet;

        return result;
      }
    }
    return colPivHouseholderQr().inverse();
  }

  ScalarType trace() const {
    assert(rows() == cols());

    ScalarType result = static_cast<ScalarType>(0.0);

    for (unsigned int i = 0; i < rows(); i++) {
      result += operator()(i,i);
    }

    return result;
  }

  ScalarType mean() const {
    assert(rows() == 1 || cols() == 1);
    ScalarType result = static_cast<ScalarType>(0.0);
    for (unsigned int i = 0; i < rows(); i++) {
      result += operator[](i);
    }

    return result / static_cast<ScalarType>(rows() * cols());
  }

  const LLT<Derived> llt() const {
    return LLT<Derived>(*this);
  }

  const PartialPivLU<Derived> partialPivLu() const {
    return PartialPivLU<Derived>(*this);
  }

  const HouseholderQR<Derived> householderQr() const {
    return HouseholderQR<Derived>(*this);
  }

  const ColPivHouseholderQR<Derived> colPivHouseholderQr() const {
    return ColPivHouseholderQR<Derived>(*this);
  }

  ScalarType* data() {
    return static_cast<Derived*>(this)->data();
  }

  const ScalarType* data() const {
    return static_cast<const Derived*>(this)->data();
  }

  //
  // Special Constructors
  //
  static Derived Zero(int NumRows = (Rows == Dynamic) ? 1 : Rows, int NumCols = (Cols == Dynamic) ? 1 : Cols) {
    Derived result (NumRows, NumCols);

    for (size_t i = 0; i < NumRows; i++) {
      for (size_t j = 0; j < NumCols; j++) {
        result(i,j) = static_cast<ScalarType>(0.0);
      }
    }

    return result;
  }

  static Derived Identity(size_t NumRows = Rows, size_t NumCols = Cols) {
    Derived result (Derived::Zero(NumRows, NumCols));

    for (size_t i = 0; i < NumRows; i++) {
        result(i,i) = static_cast<ScalarType>(1.0);
    }

    return result;
  }

  static Derived Constant(int NumRows, const ScalarType &value) {
    Derived result (NumRows, 1);

    for (size_t i = 0; i < NumRows; i++) {
      result(i,0) = value;
    }

    return result;
  }

  static Derived Constant(int NumRows, int NumCols, const ScalarType &value) {
    Derived result (NumRows, NumCols);

    for (size_t i = 0; i < NumRows; i++) {
      for (size_t j = 0; j < NumCols; j++) {
        result(i,j) = value;
      }
    }

    return result;
  }

  static Derived Random(int NumRows = (Rows == Dynamic) ? 1 : Rows, int NumCols = (Cols == Dynamic) ? 1 : Cols) {
    Derived result (NumRows, NumCols);

    for (size_t i = 0; i < NumRows; i++) {
      for (size_t j = 0; j < NumCols; j++) {
        result(i,j) = (static_cast<value_type>(rand()) / static_cast<value_type>(RAND_MAX)) * 2.0 - 1.0;
      }
    }

    return result;
  }


  //
  // Block accessors
  //
  template <
    int block_rows, 
    int block_cols
    >
  Block<
    Derived,
    ScalarType,
    block_rows,
    block_cols
    > block(int block_row_index, int block_col_index) {
      assert(block_row_index + block_rows <= rows());
      assert(block_col_index + block_cols <= cols());
    return Block<Derived, ScalarType, block_rows, block_cols>(static_cast<Derived*>(this), block_row_index, block_col_index);
  }

  template <
    int block_rows, 
    int block_cols
    >
  const Block<
    Derived,
    ScalarType,
    block_rows,
    block_cols
    > block(int block_row_index, int block_col_index) const {
      assert(block_row_index + block_rows <= rows());
      assert(block_col_index + block_cols <= cols());
    return Block<Derived, ScalarType, block_rows, block_cols>(const_cast<Derived*>(static_cast<const Derived*>(this)), block_row_index, block_col_index);
  }

  Block<
    Derived,
    ScalarType
    > block(int block_row_index, int block_col_index,
        int block_num_rows, int block_num_cols) {
      assert(block_row_index + block_num_rows <= rows());
      assert(block_col_index + block_num_cols <= cols());
    return Block<Derived, ScalarType>(static_cast<Derived*>(this), block_row_index, block_col_index, block_num_rows, block_num_cols);
  }

  const Block<
    const Derived,
    ScalarType
    > block(int block_row_index, int block_col_index,
        int block_num_rows, int block_num_cols) const {
      assert(block_row_index + block_num_rows <= rows());
      assert(block_col_index + block_num_cols <= cols());
    return Block<const Derived, ScalarType>(static_cast<const Derived*>(this), block_row_index, block_col_index, block_num_rows, block_num_cols);
  }

  // TODO: head, tail

  //
  // Transpose
  //
  Transpose<Derived, ScalarType, Cols, Rows> transpose() {
    return Transpose<Derived, ScalarType, Cols, Rows>(static_cast<Derived*>(this));
  }

  const Transpose<const Derived, ScalarType, Cols, Rows> transpose() const {
    return Transpose<const Derived, ScalarType, Cols, Rows>(static_cast<const Derived*>(this));
  }
};


template <typename ScalarType, int SizeAtCompileTime, int NumRows, int NumCols>
struct Storage;

template <typename ScalarType, int SizeAtCompileTime, int NumCols>
struct Storage<ScalarType, SizeAtCompileTime, -1, NumCols> : public Storage<ScalarType, 0, -1, -1> {};

// fixed storage
template <typename ScalarType, int SizeAtCompileTime, int NumRows, int NumCols>
struct Storage {
  ScalarType mData[SizeAtCompileTime];

  Storage() {}

  Storage(int rows, int cols) {
    resize(rows, cols);
  }

  inline size_t rows() const { return NumRows; }

  inline size_t cols() const { return NumCols; }

  void resize(int num_rows, int num_cols) {
    // Resizing of fixed size matrices not allowed
#ifndef NDEBUG
    if (num_rows != NumRows || num_cols != NumCols) {
      std::cout << "Error: trying to resize fixed matrix from " 
        << NumRows << ", " << NumCols << " to "
        << num_rows << ", " << num_cols << "." << std::endl;
    }
#endif
    assert (num_rows == NumRows && num_cols == NumCols);
  }

  inline ScalarType& coeff(int row_index, int col_index) {
//    assert (row_index >= 0 && row_index <= NumRows);
//    assert (col_index >= 0 && col_index <= NumCols);
    return mData[row_index * NumCols + col_index];
  }

  inline const ScalarType& coeff(int row_index, int col_index) const {
//    assert (row_index >= 0 && row_index <= NumRows);
//    assert (col_index >= 0 && col_index <= NumCols);
    return mData[row_index * NumCols + col_index];
  }
};

template <typename ScalarType, int NumCols>
struct Storage<ScalarType, 0, Dynamic, NumCols> {
    ScalarType* mData = nullptr;
    int mRows = 0;
    int mCols = 0;

    Storage() {}

    ~Storage() {
      delete[] mData;
    }

    Storage(int rows, int cols) {
        resize(rows, cols);
    }

    inline size_t rows() const { return mRows; }
    inline size_t cols() const { return mCols; }

    void resize(int num_rows, int num_cols) {
        if (mRows != num_rows || mCols != num_cols) {
            if (mData != nullptr) {
                delete[] mData;
            }

            mData = new ScalarType[num_rows * num_cols];
            mRows = num_rows;
            mCols = num_cols;
        }
    }

    inline ScalarType& coeff(int row_index, int col_index) {
//        assert (row_index >= 0 && row_index <= mRows);
//        assert (col_index >= 0 && col_index <= mCols);
        return mData[row_index * mCols + col_index];
    }
    inline const ScalarType& coeff(int row_index, int col_index) const {
//        assert (row_index >= 0 && row_index <= mRows);
//        assert (col_index >= 0 && col_index <= mCols);
        return mData[row_index * mCols + col_index];
    }
};


template <typename ScalarType>
struct Storage<ScalarType, 0, Dynamic, Dynamic> {
    ScalarType* mData = nullptr;
    int mRows = 0;
    int mCols = 0;

    Storage() {}

    ~Storage() {
      delete[] mData;
    }

    Storage(int rows, int cols) {
        resize(rows, cols);
    }

    inline size_t rows() const { return mRows; }
    inline size_t cols() const { return mCols; }

    void resize(int num_rows, int num_cols) {
        if (mRows != num_rows || mCols != num_cols) {
            if (mData != nullptr) {
                delete[] mData;
            }

            mData = new ScalarType[num_rows * num_cols];
            mRows = num_rows;
            mCols = num_cols;
        }
    }

    inline ScalarType& coeff(int row_index, int col_index) {
//        assert (row_index >= 0 && row_index <= mRows);
//        assert (col_index >= 0 && col_index <= mCols);
        return mData[row_index * mCols + col_index];
    }
    inline const ScalarType& coeff(int row_index, int col_index) const {
//        assert (row_index >= 0 && row_index <= mRows);
//        assert (col_index >= 0 && col_index <= mCols);
        return mData[row_index * mCols + col_index];
    }
};


template <typename ScalarType, int NumRows, int NumCols>
struct Matrix : public MatrixBase<Matrix<ScalarType, NumRows, NumCols>, ScalarType, NumRows, NumCols> {
  typedef Matrix DerivedBase;

  enum {
    RowsAtCompileTime = (NumCols == Dynamic || NumRows == Dynamic) ? -1 : NumRows,
    ColsAtCompileTime = (NumCols == Dynamic || NumRows == Dynamic) ? -1 : NumCols,
    SizeAtCompileTime = (NumRows != Dynamic && NumCols != Dynamic) ? NumRows * NumCols : 0
  };

  Storage<ScalarType, SizeAtCompileTime, RowsAtCompileTime, ColsAtCompileTime> mStorage;

  Matrix() :
    mStorage (
        SizeAtCompileTime / ColsAtCompileTime,
        SizeAtCompileTime / RowsAtCompileTime
        ) {}

  explicit Matrix(int rows) : mStorage (rows, 1) {}
  explicit Matrix(unsigned int rows) : mStorage (rows, 1) {}
  explicit Matrix(size_t rows) : mStorage (rows, 1) {}

  explicit Matrix(int rows, int cols) :
    mStorage(rows, cols) {}

  explicit Matrix(int rows, unsigned int cols) :
    mStorage(rows, cols) {}

  explicit Matrix(int rows, size_t cols) :
    mStorage(rows, cols) {}

  explicit Matrix(unsigned int rows, int cols) :
    mStorage(rows, cols) {}

  explicit Matrix(unsigned int rows, unsigned int cols) :
    mStorage(rows, cols) {}

  explicit Matrix(unsigned int rows, size_t cols) :
    mStorage(rows, cols) {}

  explicit Matrix(size_t rows, int cols) :
    mStorage(rows, cols) {}

  explicit Matrix(size_t rows, unsigned int cols) :
    mStorage(rows, cols) {}

  explicit Matrix(size_t rows, size_t cols) :
    mStorage(rows, cols) {}

  template<typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Matrix(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols> &other) {
    mStorage.resize(other.rows(), other.cols());

    int i, j, in = rows(), jn = cols();

    for (size_t i = 0; i < in; i++) {
      for (size_t j = 0; j < jn; j++) {
        this->operator()(i, j) = other(i, j);
      }
    }
  }

    Matrix (const Matrix& other) :
            mStorage(other.rows(), other.cols()){
                    memcpy (data(), other.data(), sizeof (ScalarType) * rows() * cols());
            }


    Matrix& operator=(const Matrix& other) {
      if (&other != this) {
          mStorage.resize(other.rows(), other.cols());
          memcpy (data(), other.data(), sizeof (ScalarType) * rows() * cols());
      }
      return *this;
  }

  //
  // Constructor for vectors
  //
  explicit Matrix (
      const ScalarType& v0,
      const ScalarType& v1
      ) {
    static_assert (NumRows * NumCols == 2, "Invalid matrix size");

    operator()(0,0) = v0;
    operator()(1,0) = v1;
  }

  Matrix (
      const ScalarType& v0,
      const ScalarType& v1,
      const ScalarType& v2
      ) {
    static_assert (NumRows * NumCols == 3, "Invalid matrix size");

    operator()(0,0) = v0;
    operator()(1,0) = v1;
    operator()(2,0) = v2;
  }

  Matrix (
      const ScalarType& v0,
      const ScalarType& v1,
      const ScalarType& v2,
      const ScalarType& v3
      ) {
    static_assert (NumRows * NumCols == 4, "Invalid matrix size");

    operator()(0,0) = v0;
    operator()(1,0) = v1;
    operator()(2,0) = v2;
    operator()(3,0) = v3;
  }

  Matrix (
      const ScalarType& v0,
      const ScalarType& v1,
      const ScalarType& v2,
      const ScalarType& v3,
      const ScalarType& v4,
      const ScalarType& v5
      ) {
    static_assert (NumRows * NumCols == 6, "Invalid matrix size");

    operator()(0,0) = v0;
    operator()(1,0) = v1;
    operator()(2,0) = v2;
    operator()(3,0) = v3;
    operator()(4,0) = v4;
    operator()(5,0) = v5;
  }

  //
  // Constructor for matrices
  //
  Matrix (
      const ScalarType& v00,
      const ScalarType& v01,
      const ScalarType& v02,
      const ScalarType& v10,
      const ScalarType& v11,
      const ScalarType& v12,
      const ScalarType& v20,
      const ScalarType& v21,
      const ScalarType& v22
      ) {
    static_assert (NumRows == 3 && NumCols == 3, "Invalid matrix size");

    operator()(0,0) = v00;
    operator()(0,1) = v01;
    operator()(0,2) = v02;

    operator()(1,0) = v10;
    operator()(1,1) = v11;
    operator()(1,2) = v12;

    operator()(2,0) = v20;
    operator()(2,1) = v21;
    operator()(2,2) = v22;
  }

  Matrix (
      const ScalarType& v00,
      const ScalarType& v01,
      const ScalarType& v02,
      const ScalarType& v03,
      const ScalarType& v10,
      const ScalarType& v11,
      const ScalarType& v12,
      const ScalarType& v13,
      const ScalarType& v20,
      const ScalarType& v21,
      const ScalarType& v22,
      const ScalarType& v23,
      const ScalarType& v30,
      const ScalarType& v31,
      const ScalarType& v32,
      const ScalarType& v33
      ) {
    static_assert (NumRows == 4 && NumCols == 4, "Invalid matrix size");

    operator()(0,0) = v00;
    operator()(0,1) = v01;
    operator()(0,2) = v02;
    operator()(0,3) = v03;

    operator()(1,0) = v10;
    operator()(1,1) = v11;
    operator()(1,2) = v12;
    operator()(1,3) = v13;

    operator()(2,0) = v20;
    operator()(2,1) = v21;
    operator()(2,2) = v22;
    operator()(2,3) = v23;

    operator()(3,0) = v30;
    operator()(3,1) = v31;
    operator()(3,2) = v32;
    operator()(3,3) = v33;
  }

  Matrix (
      const ScalarType& v00,
      const ScalarType& v01,
      const ScalarType& v02,
      const ScalarType& v03,
      const ScalarType& v04,
      const ScalarType& v05,

      const ScalarType& v10,
      const ScalarType& v11,
      const ScalarType& v12,
      const ScalarType& v13,
      const ScalarType& v14,
      const ScalarType& v15,

      const ScalarType& v20,
      const ScalarType& v21,
      const ScalarType& v22,
      const ScalarType& v23,
      const ScalarType& v24,
      const ScalarType& v25,

      const ScalarType& v30,
      const ScalarType& v31,
      const ScalarType& v32,
      const ScalarType& v33,
      const ScalarType& v34,
      const ScalarType& v35,

      const ScalarType& v40,
      const ScalarType& v41,
      const ScalarType& v42,
      const ScalarType& v43,
      const ScalarType& v44,
      const ScalarType& v45,

      const ScalarType& v50,
      const ScalarType& v51,
      const ScalarType& v52,
      const ScalarType& v53,
      const ScalarType& v54,
      const ScalarType& v55
        ) {
          static_assert (NumRows == 6 && NumCols == 6, "Invalid matrix size");

          operator()(0,0) = v00;
          operator()(0,1) = v01;
          operator()(0,2) = v02;
          operator()(0,3) = v03;
          operator()(0,4) = v04;
          operator()(0,5) = v05;

          operator()(1,0) = v10;
          operator()(1,1) = v11;
          operator()(1,2) = v12;
          operator()(1,3) = v13;
          operator()(1,4) = v14;
          operator()(1,5) = v15;

          operator()(2,0) = v20;
          operator()(2,1) = v21;
          operator()(2,2) = v22;
          operator()(2,3) = v23;
          operator()(2,4) = v24;
          operator()(2,5) = v25;

          operator()(3,0) = v30;
          operator()(3,1) = v31;
          operator()(3,2) = v32;
          operator()(3,3) = v33;
          operator()(3,4) = v34;
          operator()(3,5) = v35;

          operator()(4,0) = v40;
          operator()(4,1) = v41;
          operator()(4,2) = v42;
          operator()(4,3) = v43;
          operator()(4,4) = v44;
          operator()(4,5) = v45;

          operator()(5,0) = v50;
          operator()(5,1) = v51;
          operator()(5,2) = v52;
          operator()(5,3) = v53;
          operator()(5,4) = v54;
          operator()(5,5) = v55;
        }

  template <typename OtherDerived>
    Matrix& operator+=(const OtherDerived& other) {
      assert (rows() == other.rows() && cols() == other.cols() && "Error: matrix dimensions do not match!");

      int i, j, in = rows(), jn = cols();

      for (size_t i = 0; i < in; i++) {
        for (size_t j = 0; j < jn; j++) {
          this->operator()(i,j) += other(i,j);
        }
      }
      return *this;
    }

  template <typename OtherDerived>
    Matrix& operator-=(const OtherDerived& other) {
      assert (rows() == other.rows() && cols() == other.cols() && "Error: matrix dimensions do not match!");

      int i, j, in = rows(), jn = cols();

      for (size_t i = 0; i < in; i++) {
        for (size_t j = 0; j < jn; j++) {
            this->operator()(i,j) -= other(i,j);
        }
      }
      return *this;
    }

  inline ScalarType& operator()(const size_t& i, const size_t& j) {
    return mStorage.coeff(i, j);
  }

  inline const ScalarType& operator()(const size_t& i, const size_t& j) const {
    return mStorage.coeff(i, j);
  }

  ScalarType* data() {
    return mStorage.mData;
  }

  const ScalarType* data() const {
    return mStorage.mData;
  }

  size_t cols() const {
    return mStorage.cols();
  }

  size_t rows() const {
    return mStorage.rows();
  }
};
  

//
// CommaInitializer
//
template <typename Derived>
struct CommaInitializer {
  typedef typename Derived::value_type value_type;

  private:
    CommaInitializer() {}

    Derived *mParentMatrix;
    unsigned int mRowIndex;
    unsigned int mColIndex;
    bool mElementWasAdded;

  public:

  CommaInitializer(Derived &matrix, const value_type &value) : 
    mParentMatrix(&matrix),
    mRowIndex(0),
    mColIndex(0),
    mElementWasAdded(false)
  {
    assert (matrix.rows() > 0 && matrix.cols() > 0);
    mParentMatrix->operator()(0,0) = value;
  }

  CommaInitializer(Derived &matrix, unsigned int row_index, unsigned int col_index) :
    mParentMatrix(&matrix),
    mRowIndex(row_index),
    mColIndex(col_index),
    mElementWasAdded(false)
  {
    assert (matrix.rows() > 0 && matrix.cols() > 0);
  }

  ~CommaInitializer() {
    if (!mElementWasAdded 
        && (mColIndex + 1 < mParentMatrix->cols()
          || mRowIndex + 1 < mParentMatrix->rows())) {
      std::cerr 
        << "Error: too few elements passed to CommaInitializer Expected " 
        << mParentMatrix->rows() * mParentMatrix->cols()
        << " but was given " 
        << mRowIndex * mParentMatrix->cols() + mColIndex + 1 << std::endl;
      abort();
    }
  }

  CommaInitializer<Derived> operator, (const value_type &value) {
    mColIndex++;
    if (mColIndex >= mParentMatrix->cols()) {
      mRowIndex++;
      mColIndex = 0;
    }

    if (mRowIndex == mParentMatrix->rows() && mColIndex == 0) {
      std::cerr 
        << "Error: too many elements passed to CommaInitializer!Expected " 
        << mParentMatrix->rows() * mParentMatrix->cols()
        << " but was given " 
        << mRowIndex *mParentMatrix->cols() + mColIndex + 1 << std::endl;
      abort();
    }
    (*mParentMatrix)(mRowIndex, mColIndex) = value;
    mElementWasAdded = true;

    return CommaInitializer(*mParentMatrix, mRowIndex, mColIndex);
  }
};

//
// Transpose
//
template <typename Derived, typename ScalarType, int NumRows, int NumCols>
struct Transpose : public MatrixBase<Transpose<Derived, ScalarType, NumRows, NumCols>, ScalarType, NumRows, NumCols> {
  Derived* mTransposeSource;

  Transpose(Derived* transpose_source) :
    mTransposeSource(transpose_source)
  { }

  Transpose(const Transpose &other) :
    mTransposeSource(other.mTransposeSource)
  { }

  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Matrix<ScalarType, NumRows, OtherCols> operator*(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) const {
    Matrix<ScalarType, NumRows, OtherCols> result (Matrix<ScalarType, NumRows, OtherCols>::Zero(rows(), other.cols()));

    unsigned int i,j,k;
    unsigned int nrows = rows();
    unsigned int other_ncols = other.cols();
    unsigned int other_nrows = other.rows();

    for (i = 0; i < nrows; i++) {
      for (j = 0; j < other_ncols; j++) {
        for (k = 0; k < other_nrows; k++) {
          result (i,j) += operator()(i,k) * other(k,j);
        }
      }
    }
    return result;
  }

  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Transpose& operator=(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) {
    if (static_cast<const void*>(this) != static_cast<const void*>(&other)) {
      int i, j, in = other.rows(), jn = other.cols();

      for (size_t i = 0; i < in; i++) {
        for (size_t j = 0; j < jn; j++) {
          this->operator()(i,j) = other(i,j);
        }
      }
    }

    return *this;
  }

  size_t rows() const {
    return static_cast<const Derived*>(mTransposeSource)->cols();
  }
  size_t cols() const {
    return static_cast<const Derived*>(mTransposeSource)->rows();
  }

  const ScalarType& operator()(const size_t& i, const size_t& j) const {
    return static_cast<const Derived*>(mTransposeSource)->operator()(j, i);
  }
  ScalarType& operator()(const size_t& i, const size_t& j) {
    return static_cast<Derived*>(mTransposeSource)->operator()(j, i);
  }
};

//
// Block
//
template <typename Derived, typename ScalarType, int NumRows, int NumCols>
struct Block : public MatrixBase<Block<Derived, ScalarType, NumRows, NumCols>, ScalarType, NumRows, NumCols> {
  typedef Block<Derived, ScalarType, NumRows, NumCols> matrix_type;

  Derived* mBlockSource;
  int row_index;
  int col_index;
  int nrows;
  int ncols;

  Block(Derived* block_source, int row_index, int col_index) :
    mBlockSource(block_source),
    row_index(row_index),
    col_index(col_index),
    nrows(NumRows), ncols(NumCols)
  { 
    static_assert(NumRows != -1 && NumCols != -1, "Invalid block specifications: unknown number of rows and columns!");
  }

  Block(Derived* block_source, int row_index, int col_index, int num_rows, int num_cols) :
    mBlockSource(block_source),
    row_index(row_index),
    col_index(col_index),
    nrows (num_rows),
    ncols (num_cols)
  { 
  }

  Block(const Block &other) :
    mBlockSource(other.mBlockSource),
    row_index(other.row_index),
    col_index(other.col_index)
  { }

  Block& operator=(const Block &other) {
    int i, j, in = rows(), jn = cols();

    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        this->operator()(i,j) = other(i,j);
      }
    }

    return *this;
  }

  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Block& operator=(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) {
    int i, j, in = rows(), jn = cols();

    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        this->operator()(i,j) = other(i,j);
      }
    }

    return *this;
  }


  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Matrix<ScalarType, NumRows, OtherCols> operator*(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) const {
    Matrix<ScalarType, NumRows, OtherCols> result (rows(), other.cols());

    int i, j, k, in = rows(), jn = other.cols(), kn = other.rows();
    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        for (k = 0; k < kn; k++) {
          result (i,j) += operator()(i,k) * other(k,j);
        }
      }
    }
    return result;
  }

  size_t rows() const {
    return nrows;
  }
  size_t cols() const {
    return ncols;
  }

  const ScalarType& operator()(const size_t& i, const size_t& j) const {
    return static_cast<const Derived*>(mBlockSource)->operator()(row_index + i, col_index + j);
  }
  ScalarType& operator()(const size_t& i, const size_t& j) {
    return static_cast<Derived*>(mBlockSource)->operator()(row_index + i,col_index + j);
  }

  template <typename OtherDerived, typename OtherScalarType, int OtherRows, int OtherCols>
  Matrix<ScalarType, NumRows, OtherCols> operator+(const MatrixBase<OtherDerived, OtherScalarType, OtherRows, OtherCols>& other) const {
    Matrix<ScalarType, NumRows, OtherCols> result (rows(), other.cols());

    int i, j, in = rows(), jn = other.cols();
    for (i = 0; i < in; i++) {
      for (j = 0; j < jn; j++) {
        result (i,j) = operator()(i,j) + other(i,j);
      }
    }
    return result;
  }

  private:
  Block() { assert(0 && "Invalid call!"); };

  ScalarType* data() {
    assert("invalid call");
    return NULL;
  }

  const ScalarType* data() const {
    assert("invalid call");
    return NULL;
  }
};

//
// LLT Decomposition
//
template <typename Derived>
class LLT {
public:
    typedef typename Derived::value_type value_type;
    typedef MatrixBase<Derived, value_type, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> MatrixType;

    LLT() :
            mIsFactorized(false)
    {}

private:
    typedef Matrix<value_type> VectorXd;
    typedef Matrix<value_type> MatrixXXd;
    typedef Matrix<value_type, Derived::RowsAtCompileTime, 1> ColumnVector;

    bool mIsFactorized;
    Matrix<value_type, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> mQ;
    Derived mL;

public:
    LLT(const Derived &matrix) :
            mIsFactorized(false),
            mL(matrix)
    {
        compute();
    }
    LLT compute() {
      for (int i = 0; i < mL.rows(); i++) {
        for (int j = 0; j < mL.rows(); j++) {
          if (j > i) {
            mL(i,j) = 0.;
            continue;
          }
          double s = mL(i,j);
          for (int k = 0; k < j; k++) {
            s = s - mL(i,k) * mL(j,k);
          }
          if (i > j) {
            mL(i,j) = s / mL(j,j);
          } else if (s > 0.) {
            mL (i,i) = sqrt (s);
          } else {
            std::cerr << "Error computing Cholesky decomposition: matrix not symmetric positive definite!" << std::endl;
            assert (false);
          }
        }
      }


      mIsFactorized = true;

      return *this;
    }
    ColumnVector solve (
        const ColumnVector &rhs
        ) const {
      assert (mIsFactorized);

      ColumnVector y (mL.rows());
      for (unsigned int i = 0; i < mL.rows(); i++) {
        double temp = rhs[i];

        for (unsigned int j = 0; j < i; j++) {
          temp = temp - mL(i,j) * y[j];
        }

        y[i] = temp / mL(i,i);
      }

      ColumnVector x (mL.rows());
      for (int i = mL.rows() - 1; i >= 0; i--) {
        double temp = y[i];

        for (unsigned int j = i + 1; j < mL.rows(); j++) {
          temp = temp - mL(j, i) * x[j];
        }

        x[i] = temp / mL(i,i);
      }

      return x;
    }
    Derived inverse() const {
        assert (mIsFactorized);

        VectorXd rhs_temp = VectorXd::Zero(mQ.cols());
        MatrixXXd result (mQ.cols(), mQ.cols());

        for (unsigned int i = 0; i < mQ.cols(); i++) {
            rhs_temp[i] = 1.;

            result.block(0, i, mQ.cols(), 1) = solve(rhs_temp);

            rhs_temp[i] = 0.;
        }

        return result;
    }
    Derived matrixL () const {
        return mL;
    }
};


//
// Partial Pivoting LU Decomposition
//
template <typename Derived>
class PartialPivLU {
public:
    typedef typename Derived::value_type value_type;
    typedef MatrixBase<Derived, value_type, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> MatrixType;
    PartialPivLU() :
            mIsFactorized(false)
    {}
private:
    typedef Matrix<value_type> VectorXd;
    typedef Matrix<value_type> MatrixXXd;
    typedef Matrix<value_type, Derived::RowsAtCompileTime, 1> ColumnVector;
    typedef Matrix<value_type, 1, Derived::ColsAtCompileTime> RowVector;
    bool mIsFactorized;
    unsigned int *mPermutations = nullptr;
    Derived mLU;

public:
    ~PartialPivLU() {
      delete[] mPermutations;
    }

    PartialPivLU(const Derived &matrix) :
            mIsFactorized(false),
            mLU (matrix)
    {
        mPermutations = new unsigned int [matrix.cols() + 1];
        for (unsigned int i = 0; i <= matrix.cols(); i++) {
            mPermutations[i] = i;
        }
        compute(matrix);
    }

    PartialPivLU& compute(const Derived &matrix) {
      unsigned int n = matrix.rows();

      double v_abs;
      RowVector temp_vec;

      unsigned int i,j,k;

      // over all columns
      for (i = 0; i < n; i++) {
        double max_v = 0.0;
        unsigned int max_i = i;

        // Find the row pivoting index
        for (k = i; k < n; k++) {
          if ((v_abs = fabs(mLU(k, i))) > max_v) {
            max_v = v_abs;
            max_i = k;
          }
        }

        if (max_v < std::numeric_limits<double>::epsilon()) {
          std::cerr << "Error: pivoting failed for matrix A = " << std::endl;
          std::cerr << "A = " << matrix << std::endl;
          abort();
        }

        // Perform the permutation
        if (max_i != i) {
          // update permutation vector
          j = mPermutations[i];
          mPermutations[i] = mPermutations[max_i];
          mPermutations[max_i] = j;

          // swap columns
          temp_vec = mLU.block(i,0,1,n);
          mLU.block(i, 0, 1, n) = mLU.block(max_i, 0, 1, n);
          mLU.block(max_i, 0, 1, n) = temp_vec;

          // Increase number of permutations
          mPermutations[n]++;
        }

        // eliminate i'th column of k'th row
        for (int k = i+1; k < n; k++) {
          mLU(k,i) = mLU(k,i) / mLU(i,i);

          // iterate over all columns
          for (int j = i+1; j < n; j++) {
            mLU(k,j) = mLU(k,j) - mLU(i,j) * mLU(k,i);
          }
        }
      }

      mIsFactorized = true;

      return *this;
    }

    Derived matrixL() const {
      Derived result (Derived::Zero(mLU.rows(), mLU.cols()));

      unsigned int n = mLU.rows();

      for (int i = 0; i < n; i++) {
        for (int j = 0; j < i; j++) {
          result(i,j) = mLU(i,j);
        }

        result(i,i) = 1.0;
      }

      return result;
    }

    Derived matrixU() const {
      Derived result (Derived::Zero(mLU.rows(), mLU.cols()));

      unsigned int n = mLU.rows();

      for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
          result(i,j) = mLU(i,j);
        }
      }

      return result;
    }

    Derived matrixP() const {
      Derived result(Derived::Zero(mLU.rows(), mLU.cols()));

      unsigned int n = mLU.rows();
      for (int i = 0; i < n; i++) {
        result(i, mPermutations[i]) = 1.0;
      }

      return result;
    }

    ColumnVector solve (
            const ColumnVector &rhs
    ) const {
      assert (mIsFactorized);

      unsigned int n = mLU.rows();

      // Backsolve L^-1 * rhs
      ColumnVector result(n, 1);

      for (int i = 0; i < n; i++) {
        result[i] = rhs[mPermutations[i]];
        for (int j = 0; j < i; j++) {
          result[i] = result[i] - result[j] * mLU(i,j);
        }
      }

      // Solve U^-1 * result
      for (int i = n - 1; i >= 0; i--) {
        for (int j = i + 1; j < n; j++) {
          result[i] = result[i] - result[j] * mLU(i,j);
        }

        result[i] = result[i] / mLU(i,i);
      }

      return result;
    }

    Derived inverse() const {
        assert (mIsFactorized);
        VectorXd rhs_temp = VectorXd::Zero(mLU.cols());
        MatrixXXd result (mLU.cols(), mLU.cols());
        for (unsigned int i = 0; i < mLU.cols(); i++) {
            rhs_temp[i] = 1.;
            result.block(0, i, mLU.cols(), 1) = solve(rhs_temp);
            rhs_temp[i] = 0.;
        }
        return result;
    }
};

//
// QR Decomposition
//
template <typename Derived>
class HouseholderQR {
public:
    typedef typename Derived::value_type value_type;
    typedef MatrixBase<Derived, value_type, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> MatrixType;

    HouseholderQR() :
            mIsFactorized(false)
    {}

private:
    typedef Matrix<value_type> VectorXd;
    typedef Matrix<value_type> MatrixXXd;
    typedef Matrix<value_type, Derived::RowsAtCompileTime, 1> ColumnVector;

    bool mIsFactorized;
    Matrix<value_type, Derived::RowsAtCompileTime, Derived::RowsAtCompileTime> mQ;
    Derived mR;

public:
    HouseholderQR(const Derived &matrix) :
            mIsFactorized(false),
            mQ(matrix.rows(), matrix.rows())
    {
        compute(matrix);
    }
    HouseholderQR compute(const Derived& matrix) {
        mR = matrix;
        mQ = MatrixType::Identity (mR.rows(), mR.rows());

        for (unsigned int i = 0; i < mR.cols(); i++) {
            unsigned int block_rows = mR.rows() - i;
            unsigned int block_cols = mR.cols() - i;

            MatrixXXd current_block = mR.block(i,i, block_rows, block_cols);
            VectorXd column = current_block.block(0, 0, block_rows, 1);

            value_type alpha = - column.norm();
            if (current_block(0,0) < 0) {
                alpha = - alpha;
            }

            VectorXd v = current_block.block(0, 0, block_rows, 1);
            v[0] = v[0] - alpha;

            MatrixXXd Q (MatrixXXd::Identity(mR.rows(), mR.rows()));

            Q.block(i, i, block_rows, block_rows) = MatrixXXd (Q.block(i, i, block_rows, block_rows))
                                                    - MatrixXXd(v * v.transpose() / (v.squaredNorm() * 0.5));

            mR = Q * mR;

            // Normalize so that we have positive diagonal elements
            if (mR(i,i) < 0) {
                mR.block(i,i,block_rows, block_cols) = MatrixXXd(mR.block(i,i,block_rows, block_cols)) * -1.;
                Q.block(i,i,block_rows, block_rows) = MatrixXXd(Q.block(i,i,block_rows, block_rows)) * -1.;
            }

            mQ = mQ * Q;
        }

        mIsFactorized = true;

        return *this;
    }
    ColumnVector solve (
        const ColumnVector &rhs
        ) const {
      assert (mIsFactorized);

      ColumnVector y = mQ.transpose() * rhs;
      ColumnVector x = ColumnVector::Zero(mR.cols());

      int ncols = mR.cols();
      for (int i = ncols - 1; i >= 0; i--) {
        value_type z = y[i];

        for (unsigned int j = i + 1; j < ncols; j++) {
          z = z - x[j] * mR(i,j);
        }

        if (fabs(mR(i,i)) < std::numeric_limits<value_type>::epsilon() * 10) {
          std::cerr << "HouseholderQR: Cannot back-substitute as diagonal element is near zero:" << fabs(mR(i,i))<< std::endl;
          abort();
        }
        x[i] = z / mR(i,i);
      }

      assert (!std::isnan(x.squaredNorm()));

      return x;
    }
    Derived inverse() const {
        assert (mIsFactorized);

        VectorXd rhs_temp = VectorXd::Zero(mQ.cols());
        MatrixXXd result (mQ.cols(), mQ.cols());

        for (unsigned int i = 0; i < mQ.cols(); i++) {
            rhs_temp[i] = 1.;

            result.block(0, i, mQ.cols(), 1) = solve(rhs_temp);

            rhs_temp[i] = 0.;
        }

        return result;
    }
    Matrix<value_type> householderQ () const {
        return mQ;
    }
    Derived matrixR () const {
        return mR;
    }
};

template <typename Derived>
class ColPivHouseholderQR {
public:
    typedef typename Derived::value_type value_type;
    typedef MatrixBase<Derived, value_type, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> MatrixType;


private:
    typedef Matrix<value_type> VectorXd;
    typedef Matrix<value_type> MatrixXXd;
    typedef Matrix<value_type, Derived::RowsAtCompileTime, 1> ColumnVector;

    bool mIsFactorized;
    Matrix<value_type, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> mQ;
    Derived mR;

    unsigned int *mPermutations;
    value_type mThreshold;
    unsigned int mRank;

public:
    ColPivHouseholderQR():
        mIsFactorized(false) {
        mPermutations = new unsigned int[1];
    }

    ColPivHouseholderQR (const ColPivHouseholderQR& other) {
        mIsFactorized = other.mIsFactorized;
        mQ = other.mQ;
        mR = other.mR;
        mPermutations = new unsigned int[mQ.cols()];
        mThreshold = other.mThreshold;
        mRank = other.mRank;
    }

    ColPivHouseholderQR& operator= (const ColPivHouseholderQR& other) {
        if (this != &other) {
            mIsFactorized = other.mIsFactorized;
            mQ = other.mQ;
            mR = other.mR;
            delete[] mPermutations;
            mPermutations = new unsigned int[mQ.cols()];
            mThreshold = other.mThreshold;
            mRank = other.mRank;
        }

        return *this;
    }

    ColPivHouseholderQR(const MatrixType &matrix) :
            mIsFactorized(false),
            mQ(matrix.rows(), matrix.rows()),
            mThreshold (std::numeric_limits<value_type>::epsilon() * matrix.cols()) {
        mPermutations = new unsigned int [matrix.cols()];
        for (unsigned int i = 0; i < matrix.cols(); i++) {
            mPermutations[i] = i;
        }
        compute(matrix);
    }
    ~ColPivHouseholderQR() {
        delete[] mPermutations;
    }

    ColPivHouseholderQR& setThreshold (const value_type& threshold) {
        mThreshold = threshold;

        return *this;
    }
    ColPivHouseholderQR& compute(const MatrixType &matrix) {
      mR = matrix;
      mQ = MatrixType::Identity (mR.rows(), mR.rows());

      for (unsigned int i = 0; i < mR.cols(); i++) {
        unsigned int block_rows = mR.rows() - i;
        unsigned int block_cols = mR.cols() - i;

        // find and swap the column with the highest norm
        unsigned int col_index_norm_max = i;
        value_type col_norm_max = VectorXd(mR.block(i,i, block_rows, 1)).squaredNorm();

        for (unsigned int j = i + 1; j < mR.cols(); j++) {
          VectorXd column = mR.block(i, j, block_rows, 1);

          if (column.squaredNorm() > col_norm_max) {
            col_index_norm_max = j;
            col_norm_max = column.squaredNorm();
          }
        }

        if (col_norm_max < mThreshold) {
          // if all entries of the column is close to zero, we bail out
          break;
        }


        if (col_index_norm_max != i) {
          VectorXd temp_col = mR.block(0, i, mR.rows(), 1);
          mR.block(0, i, mR.rows(), 1) = mR.block(0, col_index_norm_max, mR.rows(), 1);;
          mR.block(0, col_index_norm_max, mR.rows(), 1) = temp_col;

          unsigned int temp_index = mPermutations[i];
          mPermutations[i] = mPermutations[col_index_norm_max];
          mPermutations[col_index_norm_max] = temp_index;
        }

        MatrixXXd current_block = mR.block(i,i, block_rows, block_cols);
        VectorXd column = current_block.block(0, 0, block_rows, 1);

        value_type alpha = - column.norm();
        if (current_block(0,0) < 0) {
          alpha = - alpha;
        }

        VectorXd v = current_block.block(0, 0, block_rows, 1);
        v[0] = v[0] - alpha;

        MatrixXXd Q (MatrixXXd::Identity(mR.rows(), mR.rows()));

        Q.block(i, i, block_rows, block_rows) = MatrixXXd (Q.block(i, i, block_rows, block_rows))
          - (v * v.transpose()) / (v.squaredNorm() * static_cast<value_type>(0.5));

        mR = Q * mR;

        // Normalize so that we have positive diagonal elements
        if (mR(i,i) < 0) {
          mR.block(i,i,block_rows, block_cols) = MatrixXXd(mR.block(i,i,block_rows, block_cols)) * -1.;
          Q.block(i,i,block_rows, block_rows) = MatrixXXd(Q.block(i,i,block_rows, block_rows)) * -1.;
        }

        mQ = mQ * Q;
      }

      mIsFactorized = true;

      return *this;
    }
    ColumnVector solve (
        const ColumnVector &rhs
        ) const {
    assert (mIsFactorized);

    ColumnVector y = mQ.transpose() * rhs;
    ColumnVector x = ColumnVector::Zero(mR.cols());

    for (int i = mR.cols() - 1; i >= 0; --i) {
      value_type z = y[i];

      for (unsigned int j = i + 1; j < mR.cols(); j++) {
        z = z - x[mPermutations[j]] * mR(i,j);
      }

      if (fabs(mR(i,i)) < std::numeric_limits<value_type>::epsilon() * 10) {
        std::cerr << "HouseholderQR: Cannot back-substitute as diagonal element is near zero:" << fabs(mR(i,i))<< std::endl;
        abort();
      }
      x[mPermutations[i]] = z / mR(i,i);
    }

    assert (!std::isnan(x.squaredNorm()));

    return x;
  }
    Derived inverse() const {
        assert (mIsFactorized);

        VectorXd rhs_temp = VectorXd::Zero(mQ.cols());
        Derived result (mQ.cols(), mQ.cols());

        for (unsigned int i = 0; i < mQ.cols(); i++) {
            rhs_temp[i] = 1.;

            result.block(0, i, mQ.cols(), 1) = solve(rhs_temp);

            rhs_temp[i] = 0.;
        }

        return result;
    }

    Matrix<value_type> householderQ () const {
        return mQ;
    }
    Derived matrixR () const {
        return mR;
    }
    Matrix<value_type> matrixP () const {
        MatrixXXd P = MatrixXXd::Identity(mR.cols(), mR.cols());
        MatrixXXd identity = MatrixXXd::Identity(mR.cols(), mR.cols());
        for (unsigned int i = 0; i < mR.cols(); i++) {
            P.block(0,i,mR.cols(),1) = identity.block(0,mPermutations[i], mR.cols(), 1);
        }
        return P;
    }

    unsigned int rank() const {
        value_type abs_threshold = fabs(mR(0,0)) * mThreshold;

        for (unsigned int i = 1; i < mR.cols(); i++) {
            if (fabs(mR(i,i)) < abs_threshold)
                return i;
        }

        return mR.cols();
    }
};

template <typename Derived, typename ScalarType, int Rows, int Cols>
inline Matrix<ScalarType, Rows, Cols> operator*(const ScalarType& scalar, const MatrixBase<Derived, ScalarType, Rows, Cols> &matrix) {
    return matrix * scalar;
}

template <typename Derived, typename ScalarType, int Rows, int Cols>
inline Matrix<ScalarType, Rows, Cols> operator*(const MatrixBase<Derived, ScalarType, Rows, Cols> &matrix, const ScalarType& scalar) {
    return matrix * scalar;
}

template <typename Derived, typename ScalarType, int Rows, int Cols>
inline Matrix<ScalarType, Rows, Cols> operator/(const MatrixBase<Derived, ScalarType, Rows, Cols> &matrix, const ScalarType& scalar) {
    return matrix * (1.0 / scalar);
}

template <typename Derived, typename ScalarType, int Rows, int Cols>
inline Matrix<ScalarType, Rows, Cols> operator/=(MatrixBase<Derived, ScalarType, Rows, Cols> &matrix, const ScalarType& scalar) {
    return matrix *= (1.0 / scalar);
}

//
// OpenGL Matrices and Quaternions
//

namespace GL {

typedef Matrix<float, 3, 1> Vector3f;
typedef Matrix<float, 3, 3> Matrix33f;

typedef Matrix<float, 4, 1> Vector4f;
typedef Matrix<float, 4, 4> Matrix44f;

inline Matrix33f RotateMat33 (float rot_deg, float x, float y, float z) {
  float c = cosf (rot_deg * M_PI / 180.f);
  float s = sinf (rot_deg * M_PI / 180.f);
  return Matrix33f (
      x * x * (1.0f - c) + c,
      y * x * (1.0f - c) + z * s,
      x * z * (1.0f - c) - y * s,

      x * y * (1.0f - c) - z * s,
      y * y * (1.0f - c) + c,
      y * z * (1.0f - c) + x * s,

      x * z * (1.0f - c) + y * s,
      y * z * (1.0f - c) - x * s,
      z * z * (1.0f - c) + c

      );
}


inline Matrix44f RotateMat44 (float rot_deg, float x, float y, float z) {
  float c = cosf (rot_deg * M_PI / 180.f);
  float s = sinf (rot_deg * M_PI / 180.f);
  return Matrix44f (
      x * x * (1.0f - c) + c,
      y * x * (1.0f - c) + z * s,
      x * z * (1.0f - c) - y * s,
      0.f, 

      x * y * (1.0f - c) - z * s,
      y * y * (1.0f - c) + c,
      y * z * (1.0f - c) + x * s,
      0.f,

      x * z * (1.0f - c) + y * s,
      y * z * (1.0f - c) - x * s,
      z * z * (1.0f - c) + c,
      0.f,

      0.f, 0.f, 0.f, 1.f
      );
}

inline Matrix44f TranslateMat44 (float x, float y, float z) {
  return Matrix44f (
      1.f, 0.f, 0.f, 0.f,
      0.f, 1.f, 0.f, 0.f,
      0.f, 0.f, 1.f, 0.f,
        x,   y,   z, 1.f
      );
}

inline Matrix44f ScaleMat44 (float x, float y, float z) {
  return Matrix44f (
        x, 0.f, 0.f, 0.f,
      0.f,   y, 0.f, 0.f,
      0.f, 0.f,   z, 0.f,
      0.f, 0.f, 0.f, 1.f
      );
}

inline Matrix44f Ortho(
    float left, float right,
    float bottom, float top,
    float near, float far) {
  float tx = -(right + left) / (right - left);
  float ty = -(top + bottom) / (top - bottom);
  float tz = -(far + near) / (far - near);
  return Matrix44f(
      2.0f / (right - left), 0.0f, 0.0f, 0.0f,
      0, 2.0f / (top - bottom), 0.0f, 0.0f,
      0.0f, 0.0f, -2.0f / (far - near), 0.0f,
      tx, ty, tz, 1.0f
      );
}

inline Matrix44f Perspective(float fovy, float aspect,
    float near, float far) {
  float x = (fovy * M_PI / 180.0) / 2.0f;
  float f = cos(x) / sin(x);

  return Matrix44f(
      f / aspect, 0.0f, 0.0f, 0.0f,
      0.0f, f, 0.0f, 0.0f,
      0.0f, 0.0f, (far + near) / (near - far), -1.0f,
      0.0f, 0.0f, (2.0f * far * near) / (near - far), 0.0f
      );
}

inline Matrix44f Frustum(float left, float right,
    float bottom, float top,
    float near, float far) {
  float A = (right + left) / (right - left);
  float B = (top + bottom) / (top - bottom);
  float C = -(far + near) / (far - near);
  float D = - (2.0f * far * near) / (far - near);

  return Matrix44f(
      2.0f * near / (right - left), 0.0f, 0.0f, 0.0f,
      0.0f, 2.0f * near / (top - bottom), 0.0f, 0.0f,
      A, B, C, -1.0f,
      0.0f, 0.0f, D, 0.0f
      );
}

inline Matrix44f LookAt(
    const Vector3f& eye,
    const Vector3f& poi,
    const Vector3f& up) {
  Vector3f d = (poi - eye).normalized();
  Vector3f s = d.cross(up.normalized()).normalized();
  Vector3f u = s.cross(d).normalized();

  return TranslateMat44(-eye[0], -eye[1], -eye[2]) * Matrix44f(
      s[0], u[0], -d[0], 0.0f,
      s[1], u[1], -d[1], 0.0f,
      s[2], u[2], -d[2], 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f
      );
}

//
// Quaternion
//
// order: x,y,z,w

class Quaternion : public Vector4f {
  public:
    Quaternion () :
      Vector4f (0.f, 0.f, 0.f, 1.f)
    {}
    Quaternion (const Vector4f vec4) :
      Vector4f (vec4)
    {}
    Quaternion (float x, float y, float z, float w):
      Vector4f (x, y, z, w)
    {}
    // This function is equivalent to multiplicate their corresponding rotation matrices
    Quaternion operator* (const Quaternion &q) const {
      return Quaternion (
          (*this)[3] * q[0] + (*this)[0] * q[3] + (*this)[1] * q[2] - (*this)[2] * q[1],
          (*this)[3] * q[1] + (*this)[1] * q[3] + (*this)[2] * q[0] - (*this)[0] * q[2],
          (*this)[3] * q[2] + (*this)[2] * q[3] + (*this)[0] * q[1] - (*this)[1] * q[0],
          (*this)[3] * q[3] - (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2]
          );
    }
    Quaternion& operator*=(const Quaternion &q) {
      set (
          (*this)[3] * q[0] + (*this)[0] * q[3] + (*this)[1] * q[2] - (*this)[2] * q[1],
          (*this)[3] * q[1] + (*this)[1] * q[3] + (*this)[2] * q[0] - (*this)[0] * q[2],
          (*this)[3] * q[2] + (*this)[2] * q[3] + (*this)[0] * q[1] - (*this)[1] * q[0],
          (*this)[3] * q[3] - (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2]
          );
      return *this;
    }

    static Quaternion fromGLRotate (float angle, float x, float y, float z) {
      float st = sinf (angle * M_PI / 360.f);
      return Quaternion (
            st * x,
            st * y,
            st * z,
            cosf (angle * M_PI / 360.f)
            );
    }

    Quaternion normalize() {
      return Vector4f::normalize();
    }

    Quaternion slerp (float alpha, const Quaternion &quat) const {
      // check whether one of the two has 0 length
      float s = sqrt (squaredNorm() * quat.squaredNorm());

      // division by 0.f is unhealthy!
      assert (s != 0.f);

      float angle = acos (dot(quat) / s);
      if (angle == 0.f || std::isnan(angle)) {
        return *this;
      }
      assert(!std::isnan(angle));

      float d = 1.f / sinf (angle);
      float p0 = sinf ((1.f - alpha) * angle);
      float p1 = sinf (alpha * angle);

      if (dot (quat) < 0.f) {
        return Quaternion( ((*this) * p0 - quat * p1) * d);
      }
      return Quaternion( ((*this) * p0 + quat * p1) * d);
    }

    Matrix44f toGLMatrix() const {
      float x = (*this)[0];
      float y = (*this)[1];
      float z = (*this)[2];
      float w = (*this)[3];
      return Matrix44f (
          1 - 2*y*y - 2*z*z,
          2*x*y + 2*w*z,
          2*x*z - 2*w*y,
          0.f,

          2*x*y - 2*w*z,
          1 - 2*x*x - 2*z*z,
          2*y*z + 2*w*x,
          0.f,

          2*x*z + 2*w*y,
          2*y*z - 2*w*x,
          1 - 2*x*x - 2*y*y,
          0.f,
          
          0.f,
          0.f,
          0.f,
          1.f);
    }

    static Quaternion fromGLMatrix(const Matrix44f &mat) {
      float w = sqrt (1.f + mat(0,0) + mat(1,1) + mat(2,2)) * 0.5f;
      return Quaternion (
          -(mat(2,1) - mat(1,2)) / (w * 4.f),
          -(mat(0,2) - mat(2,0)) / (w * 4.f),
          -(mat(1,0) - mat(0,1)) / (w * 4.f),
          w);
    }

    static Quaternion fromMatrix (const Matrix33f &mat) {
      float w = sqrt (1.f + mat(0,0) + mat(1,1) + mat(2,2)) * 0.5f;
      return Quaternion (
          (mat(2,1) - mat(1,2)) / (w * 4.f),
          (mat(0,2) - mat(2,0)) / (w * 4.f),
          (mat(1,0) - mat(0,1)) / (w * 4.f),
          w);
    }

    static Quaternion fromAxisAngle (const Vector3f &axis, double angle_rad) {
      double d = axis.norm();
      double s2 = std::sin (angle_rad * 0.5) / d;
      return Quaternion (
          axis[0] * s2,
          axis[1] * s2,
          axis[2] * s2,
          std::cos(angle_rad * 0.5)
          );
    }

    static Quaternion fromEulerZYX (const Vector3f &zyx_angles) {
      return Quaternion::fromAxisAngle (Vector3f (0., 0., 1.), zyx_angles[0])
        * Quaternion::fromAxisAngle (Vector3f (0., 1., 0.), zyx_angles[1])
        * Quaternion::fromAxisAngle (Vector3f (1., 0., 0.), zyx_angles[2]); 
    }

    static Quaternion fromEulerYXZ (const Vector3f &yxz_angles) {
      return Quaternion::fromAxisAngle (Vector3f (0., 1., 0.), yxz_angles[0])
        * Quaternion::fromAxisAngle (Vector3f (1., 0., 0.), yxz_angles[1])
        * Quaternion::fromAxisAngle (Vector3f (0., 0., 1.), yxz_angles[2]);
    }

    static Quaternion fromEulerXYZ (const Vector3f &xyz_angles) {
      return Quaternion::fromAxisAngle (Vector3f (0., 0., 01.), xyz_angles[2]) 
        * Quaternion::fromAxisAngle (Vector3f (0., 1., 0.), xyz_angles[1])
        * Quaternion::fromAxisAngle (Vector3f (1., 0., 0.), xyz_angles[0]);
    }
 
    Vector3f toEulerZYX () const {
      return Vector3f (1.0f, 2.0f, 3.0f
          );
    }

    Vector3f toEulerYXZ() const {
      return Vector3f (
          atan2 (-2.f * (*this)[0] * (*this)[2] + 2.f * (*this)[3] * (*this)[1],
            (*this)[2] * (*this)[2] - (*this)[1] * (*this)[1]
            -(*this)[0] * (*this)[0] + (*this)[3] * (*this)[3]),
          asin (2.f * (*this)[1] * (*this)[2] + 2.f * (*this)[3] * (*this)[0]),
          atan2 (-2.f * (*this)[0] * (*this)[1] + 2.f * (*this)[3] * (*this)[2],
            (*this)[1] * (*this)[1] - (*this)[2] * (*this)[2]
            +(*this)[3] * (*this)[3] - (*this)[0] * (*this)[0]
            )
          );
    };

    Matrix33f toMatrix() const {
      float x = (*this)[0];
      float y = (*this)[1];
      float z = (*this)[2];
      float w = (*this)[3];
      return Matrix33f (
          1 - 2*y*y - 2*z*z,
          2*x*y - 2*w*z,
          2*x*z + 2*w*y,

          2*x*y + 2*w*z,
          1 - 2*x*x - 2*z*z,
          2*y*z - 2*w*x,

          2*x*z - 2*w*y,
          2*y*z + 2*w*x,
          1 - 2*x*x - 2*y*y
      );
    }

    Quaternion conjugate() const {
      return Quaternion (
          -(*this)[0],
          -(*this)[1],
          -(*this)[2],
          (*this)[3]);
    }

    Vector3f rotate (const Vector3f &vec) const {
      Vector3f vn (vec);
      Quaternion vec_quat (vn[0], vn[1], vn[2], 0.f), res_quat;

      res_quat = (*this) * vec_quat;
      res_quat = res_quat * conjugate();

      return Vector3f (res_quat[0], res_quat[1], res_quat[2]);
    }
};

} /* namespace GL */


//
// Stream operators
//
template <typename Derived, typename ScalarType, int Rows, int Cols>
inline std::ostream& operator<<(std::ostream& output, const MatrixBase<Derived, ScalarType, Rows, Cols> &matrix) {
  size_t max_width = 0;
  size_t out_width = output.width();

  // get the widest number
  for (size_t i = 0; i < matrix.rows(); i++) {
    for (size_t j = 0; j < matrix.cols(); j++) {
      std::stringstream out_stream;
      out_stream << matrix(i,j);
      max_width = std::max (out_stream.str().size(),max_width);
    }
  }

  // overwrite width if it was explicitly prescribed
  if (out_width != 0) {
    max_width = out_width;
  }

  for (unsigned int i = 0; i < matrix.rows(); i++) {
    output.width(0);
    output.width(out_width);
    for (unsigned int j = 0; j < matrix.cols(); j++) {
      std::stringstream out_stream;
      out_stream.width (max_width);
      out_stream << matrix(i,j);
      output << out_stream.str();

      if (j < matrix.cols() - 1)
        output << " ";
    }
    
    if (matrix.rows() > 1 && i < matrix.rows() - 1)
      output << std::endl;
  }
  return output;
}

}
