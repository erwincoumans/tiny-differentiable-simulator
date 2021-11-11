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

#ifndef _TINY_VECTOR3_H
#define _TINY_VECTOR3_H

#include <assert.h>
#include <stdio.h>

#include <string>

namespace TINY
{
    typedef void (*TinySubmitProfileTiming)(const std::string& profileName);

    template <typename TinyScalar, typename TinyConstants>
    struct TinyVector3 {
        TinyScalar m_x;
        TinyScalar m_y;
        TinyScalar m_z;

        int m_size{ 3 };

        // TODO(ericheiden) remove unused rows argument (used for MatrixXxX)
        explicit TinyVector3(int unused = 0) {}

        TinyVector3(const TinyVector3& rhs) {
            m_x = rhs.m_x;
            m_y = rhs.m_y;
            m_z = rhs.m_z;
        }

        TinyVector3& operator=(const TinyVector3& rhs) {
            m_x = rhs.m_x;
            m_y = rhs.m_y;
            m_z = rhs.m_z;
            return *this;
        }

        TinyVector3(TinyScalar x, TinyScalar y, TinyScalar z)
            : m_x(x), m_y(y), m_z(z) {}

        TinyScalar x() const { return m_x; }

        TinyScalar& x() { return m_x; }

        TinyScalar getX() const { return m_x; }

        void setX(TinyScalar x) { m_x = x; }

        TinyScalar y() const { return m_y; }

        TinyScalar& y() { return m_y; }

        TinyScalar getY() const { return m_y; }
        void setY(TinyScalar y) { m_y = y; }

        TinyScalar z() const { return m_z; }
        TinyScalar& z() { return m_z; }
        TinyScalar getZ() const { return m_z; }
        void setZ(TinyScalar z) { m_z = z; }
        void setValue(const TinyScalar& x, const TinyScalar& y, const TinyScalar& z) {
            m_x = x;
            m_y = y;
            m_z = z;
        }

        void set_zero() {
            setValue(TinyConstants::zero(), TinyConstants::zero(),
                TinyConstants::zero());
        }

        static TinyVector3 zero() {
            TinyVector3 res(TinyConstants::zero(), TinyConstants::zero(),
                TinyConstants::zero());
            return res;
        }

        static TinyVector3 makeUnitZ() {
            TinyVector3 res;
            res.setValue(TinyConstants::zero(), TinyConstants::zero(),
                TinyConstants::one());
            return res;
        }
        static TinyVector3 makeUnitX() {
            TinyVector3 res;
            res.setValue(TinyConstants::one(), TinyConstants::zero(),
                TinyConstants::zero());
            return res;
        }

        static TinyVector3 makeUnitY() {
            TinyVector3 res;
            res.setValue(TinyConstants::zero(), TinyConstants::one(),
                TinyConstants::zero());
            return res;
        }
        static TinyVector3 create(const TinyScalar& x, const TinyScalar& y,
            const TinyScalar& z) {
            TinyVector3<TinyScalar, TinyConstants> res;
            res.setValue(x, y, z);
            return res;
        }

        inline TinyScalar dot(const TinyVector3& other) const {
            TinyScalar res =
                getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
            return res;
        }

        static TinyScalar dot2(const TinyVector3& a, const TinyVector3& other) {
            TinyScalar res = a.getX() * other.getX() + a.getY() * other.getY() +
                a.getZ() * other.getZ();
            return res;
        }

        inline TinyVector3 cross(const TinyVector3& v) const {
            return TinyVector3::create(m_y * v.m_z - m_z * v.m_y,
                m_z * v.m_x - m_x * v.m_z,
                m_x * v.m_y - m_y * v.m_x);
        }

        static TinyVector3 cross2(const TinyVector3& a, const TinyVector3& v) {
            return TinyVector3::create(a.m_y * v.m_z - a.m_z * v.m_y,
                a.m_z * v.m_x - a.m_x * v.m_z,
                a.m_x * v.m_y - a.m_y * v.m_x);
        }

        inline TinyScalar length() const {
            TinyScalar res = (*this).dot(*this);
            res = TinyConstants::sqrt1(res);
            return res;
        }

        inline TinyScalar length_squared() const {
            TinyScalar res = (*this).dot(*this);
            return res;
        }

        inline void normalize() { *this = *this * (TinyConstants::one() / length()); }

        inline TinyVector3 normalized() const {
            TinyVector3 tmp = *this;
            tmp.normalize();
            return tmp;
        }

        inline TinyScalar sqnorm() const { return m_x * m_x + m_y * m_y + m_z * m_z; }

        inline TinyVector3& operator+=(const TinyVector3& v) {
            m_x += v.m_x;
            m_y += v.m_y;
            m_z += v.m_z;
            return *this;
        }

        inline TinyVector3& operator-=(const TinyVector3& v) {
            m_x -= v.m_x;
            m_y -= v.m_y;
            m_z -= v.m_z;
            return *this;
        }

        inline TinyVector3& operator*=(TinyScalar x) {
            m_x *= x;
            m_y *= x;
            m_z *= x;
            return *this;
        }

        inline TinyVector3 operator-() const {
            TinyVector3 v = TinyVector3::create(-getX(), -getY(), -getZ());
            return v;
        }

        inline TinyScalar& operator[](int i) {
            switch (i) {
            case 0: {
                return m_x;
            }
            case 1: {
                return m_y;
            }
            case 2: {
                return m_z;
            }
            default: {
            }
            }
            assert(0);
            return m_x;
        }

        inline const TinyScalar& operator[](int i) const {
            switch (i) {
            case 0: {
                return m_x;
            }
            case 1: {
                return m_y;
            }
            case 2: {
                return m_z;
            }
            default: {
            }
            }
            assert(0);
            return m_x;
        }

        /**
         * Treat this vector as normal vector of a plane and compute two
         * orthogonal direction vectors of that plane.
         * p and q will be unit vectors, the normal vector does not need to be unit
         * length.
         */
        inline void plane_space(TinyVector3& p, TinyVector3& q) const {
            const TinyVector3& n = *this;
            if (n.m_z * n.m_z > TinyConstants::half()) {
                // choose p in y-z plane
                TinyScalar a = n.m_y * n.m_y + n.m_z * n.m_z;
                TinyScalar k = TinyConstants::sqrt1(a);
                p.m_x = TinyConstants::zero();
                p.m_y = -n.m_z * k;
                p.m_z = n.m_y * k;
                // set q = n x p
                q.m_x = a * k;
                q.m_y = -n.m_x * p.m_z;
                q.m_z = n.m_x * p.m_y;
            }
            else {
                // choose p in x-y plane
                TinyScalar a = n.m_x * n.m_x + n.m_y * n.m_y;
                TinyScalar k = TinyConstants::sqrt1(a);
                p.m_x = -n.m_y * k;
                p.m_y = n.m_x * k;
                p.m_z = TinyConstants::zero();
                // set q = n x p
                q.m_x = -n.m_z * p.m_y;
                q.m_y = n.m_z * p.m_x;
                q.m_z = a * k;
            }
        }

        void print(const char* txt) const {
            printf("%s\n", txt);
            for (int c = 0; c < 3; c++) {
                TinyScalar val = (*this)[c];
                double v = TinyConstants::getDouble(val);
                printf("%f, ", v);
            }
            printf("\n");
        }
        
        inline TinyScalar triple(const TinyVector3& v1, const TinyVector3& v2) const
        {
            return m_x * (v1.m_y * v2.m_z - v1.m_z * v2.m_y) +
                m_y * (v1.m_z * v2.m_x - v1.m_x * v2.m_z) +
                m_z * (v1.m_x * v2.m_y - v1.m_y * v2.m_x);
        }
    };

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> operator*(
        const TinyVector3<TinyScalar, TinyConstants>& a,
        const TinyVector3<TinyScalar, TinyConstants>& b) {
        TinyVector3<TinyScalar, TinyConstants> res =
            TinyVector3<TinyScalar, TinyConstants>::create(
                a.getX() * b.getX(), a.getY() * b.getY(), a.getZ() * b.getZ());
        return res;
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> operator*(
        const TinyScalar& a, const TinyVector3<TinyScalar, TinyConstants>& b) {
        TinyVector3<TinyScalar, TinyConstants> res =
            TinyVector3<TinyScalar, TinyConstants>::create(a * b.getX(), a * b.getY(),
                a * b.getZ());
        return res;
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> operator*(
        const TinyVector3<TinyScalar, TinyConstants>& b, const TinyScalar& a) {
        TinyVector3<TinyScalar, TinyConstants> res =
            TinyVector3<TinyScalar, TinyConstants>::create(a * b.getX(), a * b.getY(),
                a * b.getZ());
        return res;
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> operator-(
        const TinyVector3<TinyScalar, TinyConstants>& a,
        const TinyVector3<TinyScalar, TinyConstants>& b) {
        TinyVector3<TinyScalar, TinyConstants> res =
            TinyVector3<TinyScalar, TinyConstants>::create(
                a.getX() - b.getX(), a.getY() - b.getY(), a.getZ() - b.getZ());
        return res;
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> operator+(
        const TinyVector3<TinyScalar, TinyConstants>& a,
        const TinyVector3<TinyScalar, TinyConstants>& b) {
        TinyVector3<TinyScalar, TinyConstants> res =
            TinyVector3<TinyScalar, TinyConstants>::create(
                a.getX() + b.getX(), a.getY() + b.getY(), a.getZ() + b.getZ());
        return res;
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> copySignPerElem(
        const TinyVector3<TinyScalar, TinyConstants>& vec0,
        const TinyVector3<TinyScalar, TinyConstants>& vec1) {
        return TinyVector3<TinyScalar, TinyConstants>::create(
            (vec1.getX() < TinyConstants::zero()) ? -Fix64Abs(vec0.getX())
            : Fix64Abs(vec0.getX()),
            (vec1.getY() < TinyConstants::zero()) ? -Fix64Abs(vec0.getY())
            : Fix64Abs(vec0.getY()),
            (vec1.getZ() < TinyConstants::zero()) ? -Fix64Abs(vec0.getZ())
            : Fix64Abs(vec0.getZ()));
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> mulPerElem(
        const TinyVector3<TinyScalar, TinyConstants>& vec0,
        const TinyVector3<TinyScalar, TinyConstants>& vec1) {
        return TinyVector3<TinyScalar, TinyConstants>::create(
            (vec0.getX() * vec1.getX()), (vec0.getY() * vec1.getY()),
            (vec0.getZ() * vec1.getZ()));
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyVector3<TinyScalar, TinyConstants> absPerElem(
        const TinyVector3<TinyScalar, TinyConstants>& vec) {
        return TinyVector3<TinyScalar, TinyConstants>::create(
            Fix64Abs(vec.getX()), Fix64Abs(vec.getY()), Fix64Abs(vec.getZ()));
    }
 
    template <typename TinyScalar, typename TinyConstants>
    inline TinyScalar btTriple(const TinyVector3<TinyScalar, TinyConstants>& v1,
        const TinyVector3<TinyScalar, TinyConstants>& v2, 
        const TinyVector3<TinyScalar, TinyConstants>& v3)
    {
        return v1.triple(v2, v3);
    }
};
#endif  // _TINY_VECTOR3_H
