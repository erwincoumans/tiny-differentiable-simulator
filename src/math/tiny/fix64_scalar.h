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

#ifndef FIX64_SCALAR_H
#define FIX64_SCALAR_H

#include <assert.h>
#include <stdio.h>

#include <cstdlib>

#include "fix64_types.h"
#include "fix64_sin_lookup_table.h"

namespace TINY {
const smInt64_t INVALID_VALUE = 9223372036854775807;
const smInt64_t MAX_VALUE = 9223372036854775806;
const smInt64_t LARGE_VALUE = 3037000499;
const smInt64_t MIN_VALUE = -(smInt64_t)9223372036854775807LL;

const smInt64_t SQRT_MAX_VALUE = 111669149696000000;

const int NUM_BITS = 64;
const int FRACTIONAL_PLACES = 32;
const smInt64_t ONE = smInt64_t(1) << FRACTIONAL_PLACES;
const smInt64_t PI_TIMES_2 = 0x6487ED511;
const smInt64_t PI = 0x3243F6A88;
const smInt64_t PI_OVER_2 = 0x1921FB544;
const smInt64_t PI_OVER_4 = 0xC90FDAA2;
const smInt64_t E_RAW = 0x2B7E15162;
const smInt64_t EPOW4 = 0x3699205C4E;
const smInt64_t LN2 = 0xB17217F7;
const smInt64_t LOG2MAX = 0x1F00000000;
const smInt64_t LOG2MIN = -0x2000000000;
const int LUT_SIZE = (int)(PI_OVER_2 >> 15);
//#define LutInterval (Fix64Scalar::fromRawInt64(884273636704256))

static int CountLeadingZeroes(smInt64_t x) {
  int result = 0;
  while ((x & 0xF000000000000000) == 0) {
    result += 4;
    x <<= 4;
  }
  while ((x & 0x8000000000000000) == 0) {
    result += 1;
    x <<= 1;
  }
  return result;
}

struct Fix64Scalar {
  smInt64_t m_rawValue1;
#ifdef BT_DEBUG
  double m_debugValue;
#endif
  void setRawValue(smInt64_t val) {
    m_rawValue1 = val;
#ifdef BT_DEBUG
    m_debugValue = double(m_rawValue1) / double(ONE);
#endif
  }

  double getScalar() const { return double(m_rawValue1) / double(ONE); }

  template <class T>
  static double getDouble(T v) {
    return (double)v.getScalar();
  }

  void setScalar(double fl) { setRawValue(fl * ONE); }

  static void FullAssert(bool a) {
    if (!a) {
      printf("FullAssert!");
      assert(0);
      exit(0);
    }
  }

  static Fix64Scalar fromRegularInt(int i) {
    smInt64_t fpi = i;
    Fix64Scalar res;
    res.setRawValue(fpi * ONE);
    return res;
  }

  static Fix64Scalar divide(Fix64Scalar x, Fix64Scalar y) {
    Fix64Scalar res;
    res.setRawValue(MAX_VALUE);

    smInt64_t xl = x.m_rawValue1;
    smInt64_t yl = y.m_rawValue1;

    if (y.m_rawValue1 == 0) {
      return res;
      // throw new DivideByZeroException();
    }

    smUint64_t remainder = (smUint64_t)(xl >= 0 ? xl : -xl);
    smUint64_t divider = (smUint64_t)(yl >= 0 ? yl : -yl);
    smUint64_t quotient = smUint64_t(0);
    smInt64_t bitPos = NUM_BITS / 2 + 1;

    // If the divider is divisible by 2^n, take advantage of it.
    while ((divider & 0xF) == 0 && bitPos >= 4) {
      divider >>= 4;
      bitPos -= 4;
    }

    while (remainder != 0 && bitPos >= 0) {
      int shift = CountLeadingZeroes(remainder);
      if (shift > bitPos) {
        shift = bitPos;
      }
      remainder <<= shift;
      bitPos -= shift;

      smInt64_t div = remainder / divider;
      remainder = remainder % divider;
      quotient += div << bitPos;

      // Detect overflow
      if ((div & ~(0xFFFFFFFFFFFFFFFF >> bitPos)) != 0) {
        res.setRawValue(((xl ^ yl) & MIN_VALUE) == 0 ? MAX_VALUE : MIN_VALUE);
        return res;
      }

      remainder <<= 1;
      --bitPos;
    }

    // rounding
    ++quotient;
    smInt64_t result = (smInt64_t)(quotient >> 1);
    // if (((xl ^ yl) & MIN_VALUE) != 0)
    if (SignI(x) != SignI(y)) {
      result = -result;
    }

    res.setRawValue(result);
#ifdef BT_DEBUG
    double check = x.m_debugValue / y.m_debugValue;
    double diff = fabs(check - res.m_debugValue);
    if (diff > 0.000001) {
      assert(0);
    }
#endif

    return res;
  }

  template <class T>
  static Fix64Scalar convert(T) = delete;  // C++11

  static Fix64Scalar convert(int value) { return fromRegularInt(value); }

  template <class T>
  static double fraction(T, T) = delete;  // C++11

  static Fix64Scalar fraction(int num, int denom) {
    return Fix64Scalar::divide(fromRegularInt(num),
                               Fix64Scalar::fromRegularInt(denom));
  }
  static Fix64Scalar fraction_internal(int num, int denom) {
    return Fix64Scalar::divide(fromRegularInt(num),
                               Fix64Scalar::fromRegularInt(denom));
  }

  static Fix64Scalar fromScalar(double fl) {
    Fix64Scalar res;
    res.setRawValue(fl * ONE);
    //    printf("converted fromScalar %f to %ld\n", fl,res.m_rawValue1);
    return res;
  }

  static Fix64Scalar zero() {
    Fix64Scalar res;
    res.setRawValue(0);
    return res;
  }

  static Fix64Scalar pi() {
    Fix64Scalar res;
    res.setRawValue(PI);
    return res;
  }

  static Fix64Scalar half_pi() {
    Fix64Scalar res;
    res.setRawValue(PI_OVER_2);
    return res;
  }

  // todo: fixed conversion
  static Fix64Scalar twopi() {
    Fix64Scalar res;
    res.setRawValue(PI_TIMES_2);
    return res;
  }

  bool isZero() const { return (m_rawValue1 == 0); }

  static Fix64Scalar half() {
    Fix64Scalar res;
    res.setRawValue(2147483648);  // 0.5*ONE);
    return res;
  }

  static Fix64Scalar contactSlop() {
    Fix64Scalar res;
    res.setRawValue(4294967);  // 0.001
    return res;
  }

  static Fix64Scalar oneOver60() {
    Fix64Scalar res;
    res.setRawValue(68719480);  // 1/60.
    return res;
  }

  static Fix64Scalar ten() {
    Fix64Scalar res;
    res.setRawValue(10 * ONE);
    return res;
  }

  static Fix64Scalar fudge2() {
    Fix64Scalar res;
    res.setRawValue(42949);  // 1.0e-5f
    return res;
  }

  static Fix64Scalar halfPi() {
    Fix64Scalar res;
    res.setRawValue(0.5 * 3.1415926535897932384626433832795029 * ONE);
    return res;
  }

  static Fix64Scalar minusOne() {
    Fix64Scalar res;
    res.setRawValue(-ONE);
    return res;
  }

  static Fix64Scalar one() {
    Fix64Scalar res;
    res.setRawValue(ONE);
    return res;
  }
  static Fix64Scalar maxValue() {
    Fix64Scalar res;
    res.setRawValue(MAX_VALUE);
    return res;
  }

  static Fix64Scalar largeValue() {
    Fix64Scalar res;
    res.setRawValue(LARGE_VALUE);
    return res;
  }

  static Fix64Scalar invalidValue() {
    Fix64Scalar res;
    res.setRawValue(INVALID_VALUE);
    return res;
  }

  bool isInvalidValue() const { return this->m_rawValue1 == INVALID_VALUE; }

  static Fix64Scalar minValue() {
    Fix64Scalar res;
    res.setRawValue(MIN_VALUE);
    return res;
  }

  static Fix64Scalar two() {
    Fix64Scalar res;
    res.setRawValue(2 * ONE);
    return res;
  }

  // todo: use fixed point values, to avoid cross-platform issues of conversion
  // from float 3 to fixed point 3
  static Fix64Scalar three() {
    Fix64Scalar res;
    res.setRawValue(3 * ONE);
    return res;
  }

  static Fix64Scalar six() {
    Fix64Scalar res;
    res.setRawValue(6 * ONE);
    return res;
  }

  static Fix64Scalar epsilon() {
    Fix64Scalar res;
    res.setRawValue(2);
    return res;
  }

  bool exceedsSqrtRange() const {
    return (m_rawValue1 < 0 || m_rawValue1 > SQRT_MAX_VALUE);
  }

  inline Fix64Scalar operator-() const {
    Fix64Scalar v = *this;
    v.setRawValue(-v.m_rawValue1);
#ifdef BT_DEBUG
    double check = -this->m_debugValue;
    double diff = fabs(check - v.m_debugValue);
    if (diff > 0.000001) {
      assert(0);
    }
#endif
    return v;
  }

  inline Fix64Scalar& operator+=(const Fix64Scalar& b) {
    Fix64Scalar a = *this;
#ifdef BT_DEBUG
    double check = a.m_debugValue + b.m_debugValue;
#endif
    setRawValue(m_rawValue1 + b.m_rawValue1);
    Fix64Scalar sum = *this;
    bool opequal = SignI(a) == SignI(b);
    bool sumEqual = SignI(a) == SignI(sum);
    // if signs of operands are equal and signs of sum and x are different
    // if (((~(a.m_rawValue1 ^ b.m_rawValue1) & (a.m_rawValue1 ^ m_rawValue1)) &
    // MIN_VALUE) != 0)
    if (opequal && !sumEqual) {
      setRawValue(a.m_rawValue1 > 0 ? MAX_VALUE : MIN_VALUE);
    }
#ifdef BT_DEBUG
    double diff = fabs(check - sum.m_debugValue);
    if (diff > 0.000001) {
      assert(0);
    }
#endif
    return *this;
  }

  static Fix64Scalar sqrt1(const Fix64Scalar& x) {
    /// digit by digit calculation, Binary numeral system (base 2)
    // https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Digit-by-digit_calculation
    // https://web.archive.org/web/20120306040058/http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
    /*
     *	Square root by abacus algorithm, Martin Guy @ UKC, June 1985.
     *	From a book on programming abaci by Mr C. Woo.
     *	Argument is a positive integer, as is result.
     *
     *	I have formally proved that on exit:
     *		   2		   2		   2
     *		res  <= x < (res+1)	and	res  + op == x
     *
     *	This is also nine times faster than the library routine (-lm).
     *
     *  In the imlpementation below, num stands for op
     */

    Fix64Scalar res = Fix64Scalar::invalidValue();
    smInt64_t xl = x.m_rawValue1;

    if (xl < 0 || xl > SQRT_MAX_VALUE) {
      // We cannot represent infinities like Single and Double, and Sqrt is
      // mathematically undefined for x < 0. So we just throw an exception.
      assert(0);
      return res;
    }

    smInt64_t num = (smUint64_t)xl;
    smInt64_t result = 0UL;

    // second-to-top bit
    smInt64_t bit = smUint64_t(1UL) << (NUM_BITS - 2);

    while (bit > num) {
      bit >>= 2;
    }

    // The main part is executed twice, in order to avoid
    // using 128 bit values in computations.
    for (smInt64_t i = 0; i < 2; ++i) {
      // First we get the top 48 bits of the answer.
      while (bit != 0) {
        if (num >= result + bit) {
          num -= result + bit;
          result = (result >> 1) + bit;
        } else {
          result = result >> 1;
        }
        bit >>= 2;
      }

      if (i == 0) {
        // Then process it again to get the lowest 16 bits.
        if (num > (smUint64_t(1UL) << (NUM_BITS / 2)) - 1) {
          // The remainder 'num' is too large to be shifted left
          // by 32, so we have to add 1 to result manually and
          // adjust 'num' accordingly.
          // num = a - (result + 0.5)^2
          //       = num + result^2 - (result + 0.5)^2
          //       = num - result - 0.5
          num -= result;
          num = (num << (NUM_BITS / 2)) - 0x80000000UL;
          result = (result << (NUM_BITS / 2)) + 0x80000000UL;
        } else {
          num <<= (NUM_BITS / 2);
          result <<= (NUM_BITS / 2);
        }

        bit = 1UL << (NUM_BITS / 2 - 2);
      }
    }
    // Finally, if next bit would have been 1, round the result upwards.
    if (num > result) {
      ++result;
    }
    res.setRawValue(result);

#ifdef BT_DEBUG
    double check = ::sqrt(x.m_debugValue);
    double diff = fabs(check - res.m_debugValue);
    if (diff > 0.000001) {
      assert(0);
    }
#endif

    return res;
  }

  static Fix64Scalar fromRawInt64(smInt64_t raw) {
    Fix64Scalar res;
    res.setRawValue(raw);
    return res;
  }

  bool operator>(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    bool res = a.m_rawValue1 > b.m_rawValue1;
#ifdef BT_DEBUG
    bool check = a.m_debugValue > b.m_debugValue;
    assert(check == res);
#endif
    return res;
  }

  bool operator<(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    bool res = a.m_rawValue1 < b.m_rawValue1;
#ifdef BT_DEBUG
    bool check = a.m_debugValue < b.m_debugValue;
    assert(check == res);
#endif
    return res;
  }

  bool operator<=(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    return a.m_rawValue1 <= b.m_rawValue1;
  }

  bool operator>=(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    return a.m_rawValue1 >= b.m_rawValue1;
  }

  bool operator==(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    return a.m_rawValue1 == b.m_rawValue1;
  }

  bool operator!=(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    return a.m_rawValue1 != b.m_rawValue1;
  }

  Fix64Scalar operator+(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    Fix64Scalar c;
    c.setRawValue(a.m_rawValue1 + b.m_rawValue1);

    bool opequal = SignI(a) == SignI(b);
    bool sumEqual = SignI(a) == SignI(c);
    int opequal2 = (~(a.m_rawValue1 ^ b.m_rawValue1));
    int sumEqual2 = (a.m_rawValue1 ^ c.m_rawValue1);

    // if signs of operands are equal and signs of sum and x are different
    // if (((~(a.m_rawValue1 ^ b.m_rawValue1) & (a.m_rawValue1 ^ c.m_rawValue1))
    // & MIN_VALUE) != 0)
    if (opequal && !sumEqual) {
      c.setRawValue(a.m_rawValue1 > 0 ? MAX_VALUE : MIN_VALUE);
    }

#ifdef BT_DEBUG
    double check = a.m_debugValue + b.m_debugValue;
    double diff = check - c.m_debugValue;
    if (diff > 0.000001) {
      assert(0);
    }
#endif
    return c;
  }

  Fix64Scalar operator-(const Fix64Scalar& b) const {
    const Fix64Scalar& a = *this;
    Fix64Scalar c;
    c.setRawValue(a.m_rawValue1 - b.m_rawValue1);
#ifdef BT_DEBUG
    double check = a.m_debugValue - b.m_debugValue;
    double diff = check - c.m_debugValue;
    if (diff > 0.000001) {
      assert(0);
    }
#endif
    return c;
  }

  Fix64Scalar operator-=(const Fix64Scalar& y) {
    Fix64Scalar& x = *this;
    x.setRawValue(x.m_rawValue1 - y.m_rawValue1);
    return x;
  }

  Fix64Scalar operator/(const Fix64Scalar& y) const {
    const Fix64Scalar& x = *this;
    return Fix64Scalar::divide(x, y);
  }

  Fix64Scalar operator/=(const Fix64Scalar& y) {
    Fix64Scalar& x = *this;
    x = Fix64Scalar::divide(x, y);
    return x;
  }

  Fix64Scalar operator*(const Fix64Scalar& y) const {
    const Fix64Scalar& x = *this;
    smInt64_t xl = x.m_rawValue1;
    smInt64_t yl = y.m_rawValue1;

    smUint64_t xlo = (smUint64_t)(xl & smUint64_t(0x00000000FFFFFFFF));
    smUint64_t xhi = xl >> FRACTIONAL_PLACES;
    smUint64_t ylo = (smUint64_t)(yl & smUint64_t(0x00000000FFFFFFFF));
    smUint64_t yhi = yl >> FRACTIONAL_PLACES;

    smUint64_t lolo = xlo * ylo;
    smUint64_t lohi = xlo * yhi;
    smUint64_t hilo = xhi * ylo;
    smUint64_t hihi = xhi * yhi;

    smUint64_t loResult = lolo >> FRACTIONAL_PLACES;
    smUint64_t midResult1 = lohi;
    smUint64_t midResult2 = hilo;
    smUint64_t hiResult = hihi << FRACTIONAL_PLACES;

    smInt64_t sum = (smUint64_t)loResult + midResult1 + midResult2 + hiResult;

    Fix64Scalar res;
    res.setRawValue(sum);
#ifdef BT_DEBUG
    double check = x.m_debugValue * y.m_debugValue;
    double diff = check - res.m_debugValue;
    if (diff > 0.000001) {
      assert(0);
    }
#endif
    return res;
  }

  inline Fix64Scalar& operator*=(const Fix64Scalar& b) {
    Fix64Scalar& a = *this;
    Fix64Scalar res = a * b;
    a.setRawValue(res.m_rawValue1);
    return a;
  }

  static smInt64_t Fix64ClampSinValue(smInt64_t angle, bool& flipHorizontal,
                                      bool& flipVertical) {
    // Clamp value to 0 - 2*PI using modulo; this is very slow but there's no
    // better way AFAIK
    smInt64_t clamped2Pi = angle % PI_TIMES_2;
    if (angle < 0) {
      clamped2Pi += PI_TIMES_2;
    }

    // The LUT contains values for 0 - PiOver2; every other value must be
    // obtained by vertical or horizontal mirroring
    flipVertical = clamped2Pi >= PI;
    // obtain (angle % PI) from (angle % 2PI) - much faster than doing another
    // modulo
    smInt64_t clampedPi = clamped2Pi;
    while (clampedPi >= PI) {
      clampedPi -= PI;
    }
    flipHorizontal = clampedPi >= PI_OVER_2;
    // obtain (angle % PI_OVER_2) from (angle % PI) - much faster than doing
    // another modulo
    smInt64_t clampedPiOver2 = clampedPi;
    if (clampedPiOver2 >= PI_OVER_2) {
      clampedPiOver2 -= PI_OVER_2;
    }
    return clampedPiOver2;
  }

  static Fix64Scalar Floor(Fix64Scalar value) {
    // Just zero out the fractional part
    return Fix64Scalar::fromRawInt64(
        ((smUint64_t)value.m_rawValue1 & 0xFFFFFFFF00000000));
  }

  static Fix64Scalar Fix64Round(Fix64Scalar value) {
    smInt64_t fractionalPart = value.m_rawValue1 & 0x00000000FFFFFFFF;
    Fix64Scalar integralPart = Floor(value);
    if (fractionalPart < 0x80000000) {
      return integralPart;
    }
    if (fractionalPart > 0x80000000) {
      return integralPart + Fix64Scalar::one();
    }
    // if number is halfway between two values, round to the nearest even number
    // this is the method used by System.Math.Round().
    return (integralPart.m_rawValue1 & ONE) == 0
               ? integralPart
               : integralPart + Fix64Scalar::one();
  }

  static int SignI(Fix64Scalar value) {
    return value.m_rawValue1 < 0 ? -1 : value.m_rawValue1 > 0 ? 1 : 0;
  }

  static Fix64Scalar Sign(Fix64Scalar v) {
    long raw = v.m_rawValue1;
    return raw < 0 ? Fix64Scalar::minusOne()
                   : raw > 0 ? Fix64Scalar::one() : Fix64Scalar::zero();
  }

  static Fix64Scalar Fix64Abs(Fix64Scalar value) {
    if (value.m_rawValue1 == MIN_VALUE) {
      return Fix64Scalar::maxValue();
    }

    // branchless implementation, see
    // http://www.strchr.com/optimized_abs_function
    smInt64_t mask = value.m_rawValue1 >> 63;
    return Fix64Scalar::fromRawInt64((value.m_rawValue1 + mask) ^ mask);
  }

  static Fix64Scalar sin1(const Fix64Scalar x) {
    bool flipHorizontal, flipVertical;
    smInt64_t clampedL =
        Fix64ClampSinValue(x.m_rawValue1, flipHorizontal, flipVertical);
    Fix64Scalar clamped;
    clamped.setRawValue(clampedL);
    static Fix64Scalar PiOver2 = Fix64Scalar::fromRawInt64(PI_OVER_2);
    static Fix64Scalar LutInterval =
        Fix64Scalar::fromScalar(LUT_SIZE - 1) / PiOver2;

    // Find the two closest values in the LUT and perform linear interpolation
    // This is what kills the performance of this function on x86 - x64 is fine
    // though
    Fix64Scalar rawIndex = clamped * LutInterval;
    Fix64Scalar roundedIndex = Fix64Round(rawIndex);
    Fix64Scalar indexError = rawIndex - roundedIndex;

    int sinlutlen = sizeof(Fix64SinLookupTable) / sizeof(smInt64_t);

    int idx = flipHorizontal ? sinlutlen - 1 - (int)roundedIndex.getScalar()
                             : (int)roundedIndex.getScalar();
    Fix64Scalar nearestValue =
        Fix64Scalar::fromRawInt64(Fix64SinLookupTable[idx]);
    int idx2 =
        flipHorizontal
            ? sinlutlen - 1 - (int)roundedIndex.getScalar() - SignI(indexError)
            : (int)roundedIndex.getScalar() + SignI(indexError);
    Fix64Scalar secondNearestValue =
        Fix64Scalar::fromRawInt64(Fix64SinLookupTable[idx2]);

    smInt64_t delta =
        (indexError * Fix64Abs(nearestValue - secondNearestValue)).m_rawValue1;
    smInt64_t interpolatedValue =
        nearestValue.m_rawValue1 + (flipHorizontal ? -delta : delta);
    smInt64_t finalValue =
        flipVertical ? -interpolatedValue : interpolatedValue;
    Fix64Scalar res = Fix64Scalar::fromRawInt64(finalValue);
#ifdef BT_DEBUG
    double check = ::sin(x.m_debugValue);
    double diff = check - res.m_debugValue;
    if (diff > 0.000001) {
      assert(0);
    }
#endif
    return res;
  }

  static Fix64Scalar cos1(Fix64Scalar x) {
    smInt64_t xl = x.m_rawValue1;
    smInt64_t rawAngle = xl + (xl > 0 ? -PI - PI_OVER_2 : PI_OVER_2);
    Fix64Scalar res = sin1(Fix64Scalar::fromRawInt64(rawAngle));
#ifdef BT_DEBUG
    double check = ::cos(x.m_debugValue);
    double diff = check - res.m_debugValue;
    if (diff > 0.000001) {
      assert(0);
    }
#endif
    return res;
  }

  static Fix64Scalar Fix64Max(Fix64Scalar a, Fix64Scalar b) {
    if (a > b) return a;
    return b;
  }

  static Fix64Scalar Fix64Min(Fix64Scalar a, Fix64Scalar b) {
    if (a < b) return a;
    return b;
  }
};
}

#endif  // FIX64_SCALAR_H
