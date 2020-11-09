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

#ifndef _TINY_QUATERNION_H
#define _TINY_QUATERNION_H


#include "tiny_vector3.h"

namespace TINY
{
    template <typename TinyScalar, typename TinyConstants>
    struct TinyQuaternion {
        typedef ::TINY::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
        TinyScalar m_x;
        TinyScalar m_y;
        TinyScalar m_z;
        TinyScalar m_w;

        TinyQuaternion() = default;

        TinyQuaternion(TinyScalar x, TinyScalar y, TinyScalar z, TinyScalar w)
            : m_x(x), m_y(y), m_z(z), m_w(w) {
            if (x == TinyConstants::zero() && y == TinyConstants::zero() &&
                z == TinyConstants::zero() && w == TinyConstants::zero()) {
                fprintf(stderr,
                    "Error: cannot construct a quaternion with x = y = z = w = 0.");
                assert(0);
            }
        }

        const TinyScalar& getX() const { return m_x; }
        const TinyScalar& getY() const { return m_y; }
        const TinyScalar& getZ() const { return m_z; }
        const TinyScalar& getW() const { return m_w; }
        const TinyScalar& x() const { return m_x; }
        const TinyScalar& y() const { return m_y; }
        const TinyScalar& z() const { return m_z; }
        const TinyScalar& w() const { return m_w; }

        void setValue(const TinyScalar& x, const TinyScalar& y, const TinyScalar& z,
            const TinyScalar& w) {
            m_x = x;
            m_y = y;
            m_z = z;
            m_w = w;
        }

        void set_identity() {
            setValue(TinyConstants::zero(), TinyConstants::zero(),
                TinyConstants::zero(), TinyConstants::one());
        }

        static TinyQuaternion create(const TinyScalar& x, const TinyScalar& y,
            const TinyScalar& z, const TinyScalar& w) {
            TinyQuaternion res;
            res.setValue(x, y, z, w);
            return res;
        }

        inline TinyQuaternion operator-() const {
            const TinyQuaternion& q2 = *this;
            return TinyQuaternion::create(-q2.getX(), -q2.getY(), -q2.getZ(),
                -q2.getW());
        }

        TinyQuaternion inversed() const {
            return TinyQuaternion::create(-m_x, -m_y, -m_z, m_w);
        }

        TinyScalar dot(const TinyQuaternion& q) const {
            return m_x * q.getX() + m_y * q.getY() + m_z * q.getZ() + m_w * q.getW();
        }

        TinyScalar length2() const { return dot(*this); }

        TinyQuaternion& operator*=(const TinyQuaternion& q) {
            setValue(m_w * q.getX() + m_x * q.m_w + m_y * q.getZ() - m_z * q.getY(),
                m_w * q.getY() + m_y * q.m_w + m_z * q.getX() - m_x * q.getZ(),
                m_w * q.getZ() + m_z * q.m_w + m_x * q.getY() - m_y * q.getX(),
                m_w * q.m_w - m_x * q.getX() - m_y * q.getY() - m_z * q.getZ());
            return *this;
        }
        inline TinyQuaternion operator*(const TinyScalar& s) const {
            return TinyQuaternion::create(getX() * s, getY() * s, getZ() * s,
                getW() * s);
        }

        TinyQuaternion& operator*=(const TinyScalar& s) {
            m_x = m_x * s;
            m_y = m_y * s;
            m_z = m_z * s;
            m_w = m_w * s;
            return *this;
        }

        TinyQuaternion& operator/=(const TinyScalar& s) {
            assert(s != TinyConstants::zero());
            return *this *= (TinyConstants::one() / s);
        }

        inline TinyQuaternion& operator+=(const TinyQuaternion& q) {
            m_x += q.getX();
            m_y += q.getY();
            m_z += q.getZ();
            m_w += q.getW();
            return *this;
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
            case 3: {
                return m_w;
            }

            default: {}
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
            case 3: {
                return m_w;
            }

            default: {}
            }
            assert(0);
            return m_x;
        }

        inline TinyScalar length() const {
            TinyScalar res = (*this).dot(*this);
            res = TinyConstants::sqrt1(res);
            return res;
        }

        inline TinyVector3 rotate(const TinyVector3& v) const {
            TinyQuaternion q = (*this) * v;
            q *= this->inversed();

            return TinyVector3::create(q.m_x, q.m_y, q.m_z);
        }

        void setRotation(const TinyVector3& axis, const TinyScalar& _angle) {
            TinyScalar d = axis.length();
            TinyScalar s = TinyConstants::sin1(_angle * TinyConstants::half()) / d;
            setValue(axis.x() * s, axis.y() * s, axis.z() * s,
                TinyConstants::cos1(_angle * TinyConstants::half()));
        }

        /**@brief Set the quaternion using euler angles, compatible with PyBullet/ROS/Gazebo
         * @param yaw Angle around Z
         * @param pitch Angle around Y
         * @param roll Angle around X */
        void set_euler_rpy(const TinyVector3& rpy) {
            TinyScalar phi, the, psi;
            TinyScalar roll = rpy[0];
            TinyScalar pitch = rpy[1];
            TinyScalar yaw = rpy[2];
            phi = roll * TinyConstants::half();
            the = pitch * TinyConstants::half();
            psi = yaw * TinyConstants::half();
            setValue(   sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
                        cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
                        cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
                        cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi));
            normalize();
        }

        /**@brief Receive the euler angles from a quaternion, compatible with PyBullet/ROS/Gazebo
        */
        TinyVector3 get_euler_rpy() const {

            TinyVector3 rpy;
            TinyScalar sarg;
            TinyScalar sqx = m_x * m_x;
            TinyScalar sqy = m_y * m_y;
            TinyScalar sqz = m_z * m_z;
            TinyScalar squ = m_w * m_w;
            sarg = -TinyConstants::two() * (m_x * m_z - m_w * m_y);

            // If the pitch angle is PI/2 or -PI/2, we can only compute
            // the sum roll + yaw.  However, any combination that gives
            // the right sum will produce the correct orientation, so we
            // set rollX = 0 and compute yawZ.

            if (sarg <= TinyConstants::fraction(-99999, 100000))
            {
                rpy[0] = TinyConstants::zero();
                rpy[1] = -TinyConstants::half_pi();
                rpy[2] = TinyConstants::two() * TinyConstants::atan2(m_x, -m_y);
            }
            else if (sarg >= TinyConstants::fraction(99999, 100000))
            {
                rpy[0] = TinyConstants::zero();
                rpy[1] = TinyConstants::half_pi();
                rpy[2] = TinyConstants::two() * TinyConstants::atan2(-m_x, m_y);
            }
            else
            {
                rpy[0] = TinyConstants::atan2(TinyConstants::two() * (m_y * m_z + m_w * m_x), squ - sqx - sqy + sqz);
                rpy[1] = TinyConstants::asin(sarg);
                rpy[2] = TinyConstants::atan2(TinyConstants::two() * (m_x * m_y + m_w * m_z), squ + sqx - sqy - sqz);
            }
            return rpy;
        }


        TinyVector3 get_euler_rpy2() const {
            // Adapted from ETHZ kindr

            // First convert quaternion to rotation matrix
            TinyScalar m00, m01, m02;
            TinyScalar m10, m11, m12;
            TinyScalar m20, m21, m22;
            const TinyScalar tx = TinyConstants::two() * m_x;
            const TinyScalar ty = TinyConstants::two() * m_y;
            const TinyScalar tz = TinyConstants::two() * m_z;
            const TinyScalar twx = tx * m_w;
            const TinyScalar twy = ty * m_w;
            const TinyScalar twz = tz * m_w;
            const TinyScalar txx = tx * m_x;
            const TinyScalar txy = ty * m_x;
            const TinyScalar txz = tz * m_x;
            const TinyScalar tyy = ty * m_y;
            const TinyScalar tyz = tz * m_y;
            const TinyScalar tzz = tz * m_z;
            m00 = TinyConstants::one() - (tyy + tzz);
            m01 = txy - twz;
            m02 = txz + twy;
            m10 = txy + twz;
            m11 = TinyConstants::one() - (txx + tzz);
            m12 = tyz - twx;
            m20 = txz - twy;
            m21 = tyz + twx;
            m22 = TinyConstants::one() - (txx + tyy);

            // Next extract Euler angles
            TinyVector3 rpy;

            //  const Index i = a0;
            //  const Index j = (a0 + 1 + odd)%3;
            //  const Index k = (a0 + 2 - odd)%3;
            rpy[0] = TinyConstants::atan2(m12, m22);
            TinyScalar c2 = TinyConstants::sqrt1(m00 * m00 + m01 * m01);
            if (rpy[0] > TinyConstants::zero()) {
                rpy[0] -= TinyConstants::pi();
                rpy[1] = -TinyConstants::atan2(-m02, -c2);
            }
            else {
                rpy[1] = -TinyConstants::atan2(-m02, c2);
            }
            TinyScalar s1 = TinyConstants::sin1(rpy[0]);
            TinyScalar c1 = TinyConstants::cos1(rpy[0]);
            rpy[0] = -rpy[0];
            rpy[2] = -TinyConstants::atan2(s1 * m20 - c1 * m10, c1 * m11 - s1 * m21);

            return rpy;
        }

        TinyQuaternion& normalize() { return *this /= length(); }

        void print(const std::string& label) const {
            printf("%s (xyzw): \t", label.c_str());
            printf("%.6f  %.6f  %.6f  %.6f\n", TinyConstants::getDouble(m_x),
                TinyConstants::getDouble(m_y), TinyConstants::getDouble(m_z),
                TinyConstants::getDouble(m_w));
        }
    };

    template <typename TinyScalar, typename TinyConstants>
    inline TinyQuaternion<TinyScalar, TinyConstants> operator*(
        const TinyVector3<TinyScalar, TinyConstants>& w,
        const TinyQuaternion<TinyScalar, TinyConstants>& q) {
        return TinyQuaternion<TinyScalar, TinyConstants>::create(
            w.getX() * q.getW() + w.getY() * q.getZ() - w.getZ() * q.getY(),
            w.getY() * q.getW() + w.getZ() * q.getX() - w.getX() * q.getZ(),
            w.getZ() * q.getW() + w.getX() * q.getY() - w.getY() * q.getX(),
            -w.getX() * q.getX() - w.getY() * q.getY() - w.getZ() * q.getZ());
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyQuaternion<TinyScalar, TinyConstants> operator*(
        const TinyQuaternion<TinyScalar, TinyConstants>& q1,
        const TinyQuaternion<TinyScalar, TinyConstants>& q2) {
        return TinyQuaternion<TinyScalar, TinyConstants>::create(
            q1.getW() * q2.getX() + q1.getX() * q2.getW() + q1.getY() * q2.getZ() -
            q1.getZ() * q2.getY(),
            q1.getW() * q2.getY() + q1.getY() * q2.getW() + q1.getZ() * q2.getX() -
            q1.getX() * q2.getZ(),
            q1.getW() * q2.getZ() + q1.getZ() * q2.getW() + q1.getX() * q2.getY() -
            q1.getY() * q2.getX(),
            q1.getW() * q2.getW() - q1.getX() * q2.getX() - q1.getY() * q2.getY() -
            q1.getZ() * q2.getZ());
    }

    template <typename TinyScalar, typename TinyConstants>
    inline TinyQuaternion<TinyScalar, TinyConstants> operator*(
        const TinyQuaternion<TinyScalar, TinyConstants>& q,
        const TinyVector3<TinyScalar, TinyConstants>& w) {
        return TinyQuaternion<TinyScalar, TinyConstants>::create(
            q.getW() * w.getX() + q.getY() * w.getZ() - q.getZ() * w.getY(),
            q.getW() * w.getY() + q.getZ() * w.getX() - q.getX() * w.getZ(),
            q.getW() * w.getZ() + q.getX() * w.getY() - q.getY() * w.getX(),
            -q.getX() * w.getX() - q.getY() * w.getY() - q.getZ() * w.getZ());
    }
};
#endif  // _TINY_QUATERNION_H
