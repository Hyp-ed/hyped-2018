/*
 * Author: Branislav Pilnan,  Adithya Sireesh
 * Organisation: HYPED
 * Date: 3 March 2018
 * Description: This is a class for the implementation of quaternions.
 *              Quaternions have 4 components, and are mathematically represented as
 *                         q = a + bi + cj + dk.
 *              Internally, the Quaternion is stored as a 4D vector.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_QUATERNION_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_QUATERNION_HPP_

#include <initializer_list>
#include <array>
#include <cmath>

#include "vector.hpp"

using hyped::utils::math::Vector;

namespace hyped {
namespace utils {
namespace math {

template <typename T>
class Quaternion
{
 public:
    /**
     * @brief    Constructors for the class for a zero Quaternion.
     */
    Quaternion();
    explicit Quaternion(const T element);

    /**
     * @brief    Conversion from a vector to a quaternion.
     */
    template <typename U>
    explicit Quaternion(const Vector<U, 4>& rhs);

    template <typename U>
    Quaternion(const Vector<U, 3>& rhs);

    template <typename U1, typename U2>
    Quaternion(const U1 scalar, const Vector<U2, 3>& rhs);

    /**
     * @brief    Creates quaternion from 4 numbers.
     */
    Quaternion(T a, T b, T c, T d);

    /**
     * @brief    Conversion from one Quaternion type to another.
     */
     template <typename U>
     Quaternion(const Quaternion<U>& rhs);

    /**
     * @brief    For assigning values to entries in a quaternion.
     */
    T &operator[] (int index);

    /**
     * @brief    For accessing entries in the quaternion.
     */
    T operator[] (int index) const;

    Quaternion<T> operator-() const;

    template <typename U>
    Quaternion<T>& operator+=(const Quaternion<U>& rhs);

    template <typename U>
    Quaternion<T>& operator-=(const Quaternion<U>& rhs);

    /**
     * @brief    Addition or subtraction of every element by the parameter.
     */
    Quaternion<T>& operator+=(const T rhs);
    Quaternion<T>& operator-=(const T rhs);


    /**
     * @brief   Quaternion multiplication
     */
    Quaternion<T>& operator*=(const Quaternion<T>& rhs);

    /**
     * @brief    Scalar multiplication/division of vectors.
     */
    Quaternion<T>& operator*=(const T rhs);
    Quaternion<T>& operator/=(const T rhs);

    template <typename T1, typename T2>
    friend bool operator==(const Quaternion<T1>& lhs, const Quaternion<T2>& rhs);

    /**
     * @brief    Calculates the magnitude of a Quaternion.
     */
    double norm();

    Vector<T, 4> get_elements() const;

 private:
    Vector<T, 4> elements_;
};

template <typename T>
Quaternion<T>::Quaternion()
{
  // TODO(Adi): Check this
  // elements_ = new Vector<T, 4>();
}

template <typename T>
Quaternion<T>::Quaternion(const T element)
{
  elements_ *= 0;
  Vector<T, 4> quaternion(element);
  elements_ += quaternion;
}

template <typename T>
template <typename U>
Quaternion<T>::Quaternion(const Vector<U, 4>& rhs)
{
  elements_ *= 0;
  Vector<T, 4> quaternion(rhs);
  elements_ += quaternion;
}

template <typename T>
template <typename U>
Quaternion<T>::Quaternion(const Vector<U, 3>& rhs)
{
  elements_[0] = T(0);
  elements_[1] = T(rhs[0]);
  elements_[2] = T(rhs[1]);
  elements_[3] = T(rhs[2]);
}

template <typename T>
template <typename U1, typename U2>
Quaternion<T>::Quaternion(const U1 scalar, const Vector<U2, 3>& rhs)
{
  elements_[0] = T(scalar);
  elements_[1] = T(rhs[0]);
  elements_[2] = T(rhs[1]);
  elements_[3] = T(rhs[2]);
}

template <typename T>
Quaternion<T>::Quaternion(T a, T b, T c, T d)
{
  elements_*= 0;
  Vector<T, 4> quaternion({a, b, c, d});
  elements_ += quaternion;
}

template <typename T>
template <typename U>
Quaternion<T>::Quaternion(const Quaternion<U>& rhs)
{
  elements_ = rhs.get_elements();
}

template <typename T>
T &Quaternion<T>::operator[](int index)
{
  return elements_[index];
}

template <typename T>
T Quaternion<T>::operator[](int index) const
{
  return elements_[index];
}

template <typename T>
Quaternion<T> Quaternion<T>::operator-() const
{
  return (new Quaternion(-elements_));
}

template <typename T>
template <typename U>
Quaternion<T>& Quaternion<T>::operator+=(const Quaternion<U>& rhs)
{
  elements_ += rhs.get_elements();
  return *this;
}

template <typename T>
template <typename U>
Quaternion<T>& Quaternion<T>::operator-=(const Quaternion<U>& rhs)
{
  elements_-= rhs.get_elements();
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator+=(const T rhs)
{
  elements_ += rhs;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator-=(const T rhs)
{
  elements_ -= rhs;
  return *this;
}


template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const Quaternion<T>& rhs)
{
  T a = rhs[0]*elements_[0] - rhs[1]*elements_[1] -
    rhs[2]*elements_[2] - rhs[3]*elements_[3];
  T b = rhs[0]*elements_[1] + rhs[1]*elements_[0] -
    rhs[2]*elements_[3] + rhs[3]*elements_[2];
  T c = rhs[0]*elements_[2] + rhs[1]*elements_[3] +
    rhs[2]*elements_[0] - rhs[3]*elements_[1];
  T d = rhs[0]*elements_[3] - rhs[1]*elements_[2] +
    rhs[2]*elements_[1] + rhs[3]*elements_[0];

  elements_[0] = a;
  elements_[1] = b;
  elements_[2] = c;
  elements_[3] = d;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const T rhs)
{
  elements_ *= rhs;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator/=(const T rhs)
{
  elements_ /= rhs;
  return *this;
}

template <typename T>
double Quaternion<T>::norm()
{
  return elements_.norm();
}

template <typename T1, typename T2>
auto operator+(const Quaternion<T1>& lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]+rhs[0])>
{
  Quaternion<decltype(lhs[0]+rhs[0])> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator+(const Quaternion<T1>& lhs, const T2 rhs)
    -> Quaternion<decltype(lhs[0]+rhs)>
{
  Quaternion<decltype(lhs[0]+rhs)> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator+(const T1 lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs+rhs[0])>
{
  Quaternion<decltype(lhs+rhs[0])> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator-(const Quaternion<T1>& lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]-rhs[0])>
{
  Quaternion<decltype(lhs[0]-rhs[0])> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator-(const Quaternion<T1>& lhs, const T2 rhs)
    -> Quaternion<decltype(lhs[0]-rhs)>
{
  Quaternion<decltype(lhs[0]-rhs)> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator-(const T1 lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs-rhs[0])>
{
  Quaternion<decltype(lhs-rhs[0])> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator*(const Quaternion<T1>& lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]*rhs[0])>
{
  Quaternion<decltype(lhs[0]*rhs[0])> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator*(const Quaternion<T1>& lhs, const T2 rhs)
    -> Quaternion<decltype(lhs[0]*rhs)>
{
  Quaternion<decltype(lhs[0]*rhs)> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator*(const T1 lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs*rhs[0])>
{
  Quaternion<decltype(lhs*rhs[0])> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1, typename T2>
auto operator/(const Quaternion<T1>& lhs, const T2 rhs)
    -> Quaternion<decltype(lhs[0]*rhs)>
{
  Quaternion<decltype(lhs[0]*rhs)> ans(lhs);
  ans /= rhs;
  return ans;
}

template <typename T1, typename T2>
bool operator==(const Quaternion<T1>& lhs, const Quaternion<T2>& rhs)
{
  return (lhs.elements_ == rhs.elements_);
}

template <typename T>
Vector<T, 4> Quaternion<T>::get_elements() const
{
  return elements_;
}

}}}  // hyped::utils::math

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_QUATERNION_HPP_
