/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_EIGENMATH_H
#define RBDL_EIGENMATH_H

/* Exporting templated symbols is tricky when using MSVC. The following lines
 * causes the classes in this file not to be explicitly exported. Instead
 * they are already implicitly exported.
 */
#if defined(WIN32) && defined(rbdl_EXPORTS)
#define RBDL_TEMPLATE_DLLAPI
#else
#define RBDL_TEMPLATE_DLLAPI RBDL_DLLAPI
#endif

class RBDL_TEMPLATE_DLLAPI Vector2_t : public Eigen::Vector2d
{
  public:
    typedef Eigen::Vector2d Base;

    template<typename OtherDerived>
      Vector2_t(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Vector2d(other)
      {}

    template<typename OtherDerived>
      Vector2_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
      {
        this->Base::operator=(other);
        return *this;
      }

    EIGEN_STRONG_INLINE Vector2_t()
    {}

    EIGEN_STRONG_INLINE Vector2_t(
        const double& v0, const double& v1
        )
    {
      Base::_check_template_params();

      (*this) << v0, v1;
    }

    void set(const double& v0, const double& v1)
    {
      Base::_check_template_params();

      (*this) << v0, v1;
    }
};

class RBDL_TEMPLATE_DLLAPI Vector3_t : public Eigen::Vector3d
{
  public:
    typedef Eigen::Vector3d Base;

    template<typename OtherDerived>
      Vector3_t(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Vector3d(other)
      {}

    template<typename OtherDerived>
      Vector3_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
      {
        this->Base::operator=(other);
        return *this;
      }

    EIGEN_STRONG_INLINE Vector3_t()
    {}

    EIGEN_STRONG_INLINE Vector3_t(
        const double& v0, const double& v1, const double& v2
        )
    {
      Base::_check_template_params();

      (*this) << v0, v1, v2;
    }

    void set(const double& v0, const double& v1, const double& v2)
    {
      Base::_check_template_params();

      (*this) << v0, v1, v2;
    }
};

class RBDL_TEMPLATE_DLLAPI Matrix3_t : public Eigen::Matrix3d
{
  public:
    typedef Eigen::Matrix3d Base;

    template<typename OtherDerived>
      Matrix3_t(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Matrix3d(other)
      {}

    template<typename OtherDerived>
      Matrix3_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
      {
        this->Base::operator=(other);
        return *this;
      }

    EIGEN_STRONG_INLINE Matrix3_t()
    {}

    EIGEN_STRONG_INLINE Matrix3_t(
        const double& m00, const double& m01, const double& m02,
        const double& m10, const double& m11, const double& m12,
        const double& m20, const double& m21, const double& m22
        )
    {
      Base::_check_template_params();

      (*this)
        << m00, m01, m02,
        m10, m11, m12,
        m20, m21, m22
          ;
    }
};

class RBDL_TEMPLATE_DLLAPI Vector4_t : public Eigen::Vector4d
{
  public:
    typedef Eigen::Vector4d Base;

    template<typename OtherDerived>
      Vector4_t(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Vector4d(other)
      {}

    template<typename OtherDerived>
      Vector4_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
      {
        this->Base::operator=(other);
        return *this;
      }

    EIGEN_STRONG_INLINE Vector4_t()
    {}

    EIGEN_STRONG_INLINE Vector4_t(
        const double& v0, const double& v1, const double& v2, const double& v3
        )
    {
      Base::_check_template_params();

      (*this) << v0, v1, v2, v3;
    }

    void set(const double& v0, const double& v1, const double& v2, const double& v3)
    {
      Base::_check_template_params();

      (*this) << v0, v1, v2, v3;
    }
};

class RBDL_TEMPLATE_DLLAPI SpatialVector_t : public Eigen::Matrix<double, 6, 1>
{
  public:
    typedef Eigen::Matrix<double, 6, 1> Base;

    template<typename OtherDerived>
      SpatialVector_t(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Matrix<double, 6, 1>(other)
      {}

    template<typename OtherDerived>
      SpatialVector_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
      {
        this->Base::operator=(other);
        return *this;
      }

    EIGEN_STRONG_INLINE SpatialVector_t()
    {}

    EIGEN_STRONG_INLINE SpatialVector_t(
        const double& v0, const double& v1, const double& v2,
        const double& v3, const double& v4, const double& v5
        )
    {
      Base::_check_template_params();

      (*this) << v0, v1, v2, v3, v4, v5;
    }

    void set(
        const double& v0, const double& v1, const double& v2,
        const double& v3, const double& v4, const double& v5
        )
    {
      Base::_check_template_params();

      (*this) << v0, v1, v2, v3, v4, v5;
    }
};

class RBDL_TEMPLATE_DLLAPI SpatialMatrix_t : public Eigen::Matrix<double, 6, 6>
{
  public:
    typedef Eigen::Matrix<double, 6, 6> Base;

    template<typename OtherDerived>
      SpatialMatrix_t(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Matrix<double, 6, 6>(other)
      {}

    template<typename OtherDerived>
      SpatialMatrix_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
      {
        this->Base::operator=(other);
        return *this;
      }

    EIGEN_STRONG_INLINE SpatialMatrix_t()
    {}

    EIGEN_STRONG_INLINE SpatialMatrix_t(
        const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m04, const Scalar& m05,
        const Scalar& m10, const Scalar& m11, const Scalar& m12, const Scalar& m13, const Scalar& m14, const Scalar& m15,
        const Scalar& m20, const Scalar& m21, const Scalar& m22, const Scalar& m23, const Scalar& m24, const Scalar& m25,
        const Scalar& m30, const Scalar& m31, const Scalar& m32, const Scalar& m33, const Scalar& m34, const Scalar& m35,
        const Scalar& m40, const Scalar& m41, const Scalar& m42, const Scalar& m43, const Scalar& m44, const Scalar& m45,
        const Scalar& m50, const Scalar& m51, const Scalar& m52, const Scalar& m53, const Scalar& m54, const Scalar& m55
        )
    {
      Base::_check_template_params();

      (*this)
        << m00, m01, m02, m03, m04, m05
        , m10, m11, m12, m13, m14, m15
        , m20, m21, m22, m23, m24, m25
        , m30, m31, m32, m33, m34, m35
        , m40, m41, m42, m43, m44, m45
        , m50, m51, m52, m53, m54, m55
        ;
    }

    void set(
        const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m04, const Scalar& m05,
        const Scalar& m10, const Scalar& m11, const Scalar& m12, const Scalar& m13, const Scalar& m14, const Scalar& m15,
        const Scalar& m20, const Scalar& m21, const Scalar& m22, const Scalar& m23, const Scalar& m24, const Scalar& m25,
        const Scalar& m30, const Scalar& m31, const Scalar& m32, const Scalar& m33, const Scalar& m34, const Scalar& m35,
        const Scalar& m40, const Scalar& m41, const Scalar& m42, const Scalar& m43, const Scalar& m44, const Scalar& m45,
        const Scalar& m50, const Scalar& m51, const Scalar& m52, const Scalar& m53, const Scalar& m54, const Scalar& m55
        )
    {
      Base::_check_template_params();

      (*this)
        << m00, m01, m02, m03, m04, m05
        , m10, m11, m12, m13, m14, m15
        , m20, m21, m22, m23, m24, m25
        , m30, m31, m32, m33, m34, m35
        , m40, m41, m42, m43, m44, m45
        , m50, m51, m52, m53, m54, m55
        ;
    }
};

/* _RBDL_EIGENMATH_H */
#endif
