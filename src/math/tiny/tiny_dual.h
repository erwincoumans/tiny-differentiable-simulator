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

#ifndef __TINYDUAL_H_
#define __TINYDUAL_H_

#include <math.h>

namespace TINY
{
template <typename Scalar>
class TinyDual {
 public:
  explicit TinyDual(Scalar real = Scalar(), Scalar dual = Scalar())
      : m_real(real), m_dual(dual) {}
      
  const Scalar& real() const { return m_real; }
  const Scalar& dual() const { return m_dual; }

  void set_real(const Scalar& real) { m_real = real; }
  void set_dual(const Scalar& dual) { m_dual = dual; }

  inline friend TinyDual operator+(const TinyDual& lhs, const TinyDual& rhs) {
    return TinyDual(lhs.real() + rhs.real(), lhs.dual() + rhs.dual());
  }

  inline friend TinyDual operator-(const TinyDual& lhs, const TinyDual& rhs) {
    return TinyDual(lhs.real() - rhs.real(), lhs.dual() - rhs.dual());
  }

  inline friend TinyDual operator*(const TinyDual& lhs, const TinyDual& rhs) {
    return TinyDual(rhs.real() * lhs.real(),
                    rhs.dual() * lhs.real() + rhs.real() * lhs.dual());
  }
  inline friend TinyDual operator/(const TinyDual& lhs, const TinyDual& rhs) {
    return TinyDual(lhs.real() / rhs.real(),
                    (lhs.dual() * rhs.real() - lhs.real() * rhs.dual()) /
                        (rhs.real() * rhs.real()));
  }

  inline friend bool operator<(const TinyDual& lhs, const TinyDual& rhs) {
    return lhs.real() < rhs.real();
  }
  inline friend bool operator<=(const TinyDual& lhs, const TinyDual& rhs) {
    return lhs.real() <= rhs.real();
  }
  inline friend bool operator>=(const TinyDual& lhs, const TinyDual& rhs) {
    return lhs.real() >= rhs.real();
  }
  inline friend bool operator>(const TinyDual& lhs, const TinyDual& rhs) {
    return lhs.real() > rhs.real();
  }
  
  inline friend bool operator==(const TinyDual& lhs, const TinyDual& rhs) {
    return lhs.real() == rhs.real();
  }
  inline friend bool operator!=(const TinyDual& lhs, const TinyDual& rhs) {
    return lhs.real() != rhs.real();
  }

  inline friend TinyDual operator-(const TinyDual& rhs) {
    return TinyDual(-rhs.real(), -rhs.dual());
  }

  inline TinyDual& operator+=(const TinyDual& lhs) {
    m_real += lhs.real();
    m_dual += lhs.dual();
    return *this;
  }

  inline TinyDual& operator-=(const TinyDual& lhs) {
    m_real -= lhs.real();
    m_dual -= lhs.dual();
    return *this;
  }

  inline TinyDual& operator*=(const TinyDual& rhs) {
    m_dual = rhs.dual() * m_real + rhs.real() * m_dual;
    m_real *= rhs.real();
    return *this;
  }
  inline TinyDual& operator/=(const TinyDual& rhs) {
    m_dual =
        (m_dual * rhs.real() - m_real * rhs.dual()) / (rhs.real() * rhs.real());
    m_real /= rhs.real();
    return *this;
  }

 private:
  Scalar m_real;
  Scalar m_dual;
};

template <typename Scalar>
inline TinyDual<Scalar> sin(const TinyDual<Scalar>& z) {
  return TinyDual<Scalar>(::sin(z.real()), z.dual() * ::cos(z.real()));
}

template <typename Scalar>
inline TinyDual<Scalar> cos(const TinyDual<Scalar>& z) {
  return TinyDual<Scalar>(::cos(z.real()), -z.dual() * ::sin(z.real()));
}
};
#endif  // __TINYDUAL_H_
