/*
 * Author: Branislav Pilnan,  Adithya Sireesh
 * Organisation: HYPED
 * Date: 3 March 2018
 * Description: This is a class for the implementation of quaternions.
 *              Quaternions have 4 components, and are mathematically
 *              represented as q = a + bi + cj + dk. Internally, the
 *              Quaternion is stored as a 4D vector.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License,  Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing,  software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,  either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_QUATERNION_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_QUATERNION_HPP_

#include "vector.hpp"

#include <initializer_list>
#include <array>
#include <cmath>

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
    Quaternion(const T element);

    /**
     * @brief    Conversion from a vector to a quaternion.
     */
    template <typename U>
    Quaternion(const Vector<U,4>& rhs);

    template <typename U>
    Quaternion(const Vector<U,3>& rhs);

    template <typename U1, typename U2>
    Quaternion(const U1 scalar, const Vector<U2,3>& rhs);

    /**
     * @brief    Creates quaternion from 4 numbers.
     */
    Quaternion(T a, T b, T c, T d);

    /**
     * @brief    Conversion from a Quaternion type to another.
     */
     template <typename U>
     Quaternion(const Quaternion<U>& rhs);
  
    /**
     * @brief    For assigning values to entries in a vector.
     */
    T &operator[] (int index);

    /**
     * @brief    For accessing entries in the vector.
     */
    T operator[] (int index) const;

    Quaternion<T> operator-() const;
  
    template <typename U>
    Quaternion<T>& operator+=(const Quaternion<U>& rhs);
  
    template <typename U>
    Quaternion<T>& operator-=(const Quaternion<U>& rhs);

    /**
     * @brief    Addition or subtraction of by a
     *           Vector.
     */
    Quaternion<T>& operator+=(const Vector<T,4>& rhs);
    Quaternion<T>& operator-=(const Vector<T,4>& rhs);

    /**
     * @brief    Addition or subtraction of every
     *           entry by the parameter.
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

    /**
     * @brief    Calculates the magnitude of a Quaternion.
     */
    double norm();

    Vector<T,4> get_quaternion() const;

  private:
    Vector<T,4> quaternion_;
};

template <typename T>
Quaternion<T>::Quaternion()
{
  //this->quaternion_ = new Vector<T,4>();
}

template <typename T>
Quaternion<T>::Quaternion(const T element)
{
  this->quaternion_*=0;
  Vector<T,4> quaternion(element);
  this->quaternion_+=quaternion;
}

template <typename T>
template <typename U>
Quaternion<T>::Quaternion(const Vector<U,4>& rhs)
{
  this->quaternion_*=0;
  Vector<T,4> quaternion(rhs);
  this->quaternion_+=quaternion;
}

template <typename T>
template <typename U>
Quaternion<T>::Quaternion(const Vector<U,3>& rhs)
{
  this->quaternion_[0] = T(0);
  this->quaternion_[1] = T(rhs[0]);
  this->quaternion_[2] = T(rhs[1]);
  this->quaternion_[3] = T(rhs[2]);
}

template <typename T>
template <typename U1, typename U2>
Quaternion<T>::Quaternion(const U1 scalar, const Vector<U2,3>& rhs)
{
  this->quaternion_[0] = T(scalar);
  this->quaternion_[1] = T(rhs[0]);
  this->quaternion_[2] = T(rhs[1]);
  this->quaternion_[3] = T(rhs[2]);
}

template <typename T>
Quaternion<T>::Quaternion(T a, T b, T c, T d)
{
  this->quaternion_*=0;
  Vector<T,4> quaternion({a,b,c,d});
  this->quaternion_+=quaternion;
}

template <typename T>
template <typename U>
Quaternion<T>::Quaternion(const Quaternion<U>& rhs)
{
  this->quaternion_*=0;
  this->quaternion_+=rhs.get_quaternion();
}

template <typename T>
T &Quaternion<T>::operator[](int index)
{
  return this->quaternion_[index];
}

template <typename T>
T Quaternion<T>::operator[](int index) const
{
  return this->quaternion_[index];
}

template <typename T>
Quaternion<T> Quaternion<T>::operator-() const
{
  return (new Quaternion(-this->quaternion_));
}

template <typename T>
template <typename U>
Quaternion<T>& Quaternion<T>::operator+=(const Quaternion<U>& rhs)
{
  this->quaternion_ +=rhs.get_quaternion();
  return *this;
}

template <typename T>
template <typename U>
Quaternion<T>& Quaternion<T>::operator-=(const Quaternion<U>& rhs)
{
  this->quaternion_-=rhs.get_quaternion();
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator+=(const Vector<T,4>& rhs)
{
  this->quaternion_+=rhs;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator-=(const Vector<T,4>& rhs)
{
  this->quaternion_-=rhs;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator+=(const T rhs)
{
  this->quaternion_+=rhs;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator-=(const T rhs)
{
  this->quaternion_-=rhs;
  return *this;
}


template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const Quaternion<T>& rhs)
{
  T a = rhs[0]*this->quaternion_[0] - rhs[1]*this->quaternion_[1] - rhs[2]*this->quaternion_[2] - rhs[3]*this->quaternion_[3];
  T b = rhs[0]*this->quaternion_[1] + rhs[1]*this->quaternion_[0] - rhs[2]*this->quaternion_[3] + rhs[3]*this->quaternion_[2];
  T c = rhs[0]*this->quaternion_[2] + rhs[1]*this->quaternion_[3] + rhs[2]*this->quaternion_[0] - rhs[3]*this->quaternion_[1];
  T d = rhs[0]*this->quaternion_[3] - rhs[1]*this->quaternion_[2] + rhs[2]*this->quaternion_[1] + rhs[3]*this->quaternion_[0];
  this->quaternion_[0] = a;
  this->quaternion_[1] = b;
  this->quaternion_[2] = c;
  this->quaternion_[3] = d;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const T rhs)
{
  this->quaternion_*=rhs;
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator/=(const T rhs)
{
  this->quaternion_/=rhs;
  return *this;
}

template <typename T>
double Quaternion<T>::norm()
{
  return this->quaternion_.norm();
}

template <typename T1,  typename T2>
auto operator+(const Quaternion<T1>& lhs,  const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]+rhs[0])>
{
  Quaternion<decltype(lhs[0]+rhs[0])> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator+(const Quaternion<T1>& lhs,  const T2 rhs)
    -> Quaternion<decltype(lhs[0]+rhs)>
{
  Quaternion<decltype(lhs[0]+rhs)> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator+(const T1 lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs+rhs[0])>
{
  Quaternion<decltype(lhs+rhs[0])> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator+(const Quaternion<T1>& lhs, const Vector<T2,4>& rhs)
    -> Quaternion<decltype(lhs[0]+rhs[0])>
{
  Quaternion<decltype(lhs[0]+rhs[0])> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator+(const Vector<T1,4>& lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]+rhs[0])>
{
  Quaternion<decltype(lhs[0]+rhs[0])> ans(lhs);
  ans += rhs;
  return ans;
}


template <typename T1,  typename T2>
auto operator-(const Quaternion<T1>& lhs,  const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]-rhs[0])>
{
  Quaternion<decltype(lhs[0]-rhs[0])> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator-(const Quaternion<T1>& lhs,  const T2 rhs)
    -> Quaternion<decltype(lhs[0]-rhs)>
{
  Quaternion<decltype(lhs[0]-rhs)> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator-(const T1 lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs-rhs[0])>
{
  Quaternion<decltype(lhs-rhs[0])> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator-(const Quaternion<T1>& lhs, const Vector<T2,4>& rhs)
    -> Quaternion<decltype(lhs[0]-rhs[0])>
{
  Quaternion<decltype(lhs[0]-rhs[0])> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator-(const Vector<T1,4>& lhs, const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]-rhs[0])>
{
  Quaternion<decltype(lhs[0]-rhs[0])> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator*(const Quaternion<T1>& lhs,  const Quaternion<T2>& rhs)
    -> Quaternion<decltype(lhs[0]*rhs[0])>
{
  Quaternion<decltype(lhs[0]*rhs[0])> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator*(const Quaternion<T1>& lhs,  const T2 rhs)
    -> Quaternion<decltype(lhs[0]*rhs)>
{
  Quaternion<decltype(lhs[0]*rhs)> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator*(const T1 lhs,  const Quaternion<T2>& rhs)
    -> Vector<decltype(lhs*rhs[0]),4>
{
  Vector<decltype(lhs*rhs[0]),4> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1,  typename T2>
auto operator/(const Quaternion<T1>& lhs,  const T2 rhs)
    -> Quaternion<decltype(lhs[0]*rhs)>
{
  Quaternion<decltype(lhs[0]*rhs)> ans(lhs);
  ans /= rhs;
  return ans;
}

template <typename T1, typename T2>
bool operator==(const Quaternion<T1>& lhs, const Quaternion<T2>& rhs)
{
  return (lhs.get_quaternion() == rhs.get_quaternion());
}

template <typename T>
Vector<T,4> Quaternion<T>::get_quaternion() const
{
  return this->quaternion_;
}
    }
  }
}


#endif
