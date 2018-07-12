
/*
 * Author: Branislav Pilnan,  Uday Patel
 * Organisation: HYPED
 * Date: 17 February 2018
 * Description: K-dimensional vector class that supports addition,  substraction,  scalar
 *              multiplication, and scalar division.
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

#include <initializer_list>

#include <array>
#include <cmath>

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_VECTOR_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_VECTOR_HPP_

namespace hyped {
namespace utils {
namespace math {

template <typename T, int dimension>
class Vector {
 public:
  static_assert(dimension > 0,  "Dimension must be greater than zero.");

  /**
   * @brief    Constructors for the class for a zero vector.
   */
  Vector();
  explicit Vector(const T element);

  /**
   * @brief    Constructors for the class for a particular vector.
   */
  Vector(const std::array<T, dimension>& vector);
  explicit Vector(const std::initializer_list<T> elements);

  /**
   * @brief    Conversion from one vector type to another.
   */
  template <typename U>
  Vector(const Vector<U, dimension>& rhs);

  /**
   * @brief    For assigning values to entries in a vector.
   */
  T& operator[] (int index);

  /**
   * @brief    For accessing entries in the vector.
   */
  T operator[] (int index) const;

  Vector<T, dimension> operator-() const;
  Vector<T, dimension>& operator+=(const Vector<T, dimension>& rhs);
  Vector<T, dimension>& operator-=(const Vector<T, dimension>& rhs);

  /**
   * @brief    Addition or subtraction of every element by a constant.
   */
  Vector<T, dimension>& operator+=(const T rhs);
  Vector<T, dimension>& operator-=(const T rhs);

  /**
   * @brief    Scalar multiplication/division of vectors.
   */
  Vector<T, dimension>& operator*=(const T rhs);
  Vector<T, dimension>& operator/=(const T rhs);

  /**
   * @brief    Element-wise multiplication and division of vectors.
   */
  Vector<T, dimension>& operator*=(const Vector<T, dimension>& rhs);
  Vector<T, dimension>& operator/=(const Vector<T, dimension>& rhs);

  /**
   * @brief    Calculates the magnitude of a vector.
   */
  double norm();

  /**
   * @brief Element-wise square root
   *
   * @return Vector<T, dimension> A new Vector with sqrt applied
   */
  Vector<T, dimension> sqrt();

  /**
   * @brief    Creates a new vector of magnitude one.
   */
  Vector<double, dimension> toUnitVector();

 private:
  std::array<T, dimension> elements_;
};

template <typename T, int dimension>
Vector<T, dimension>::Vector()
{
  for (int i = 0; i < dimension; i++)
    elements_[i] = 0;
}

template <typename T, int dimension>
Vector<T, dimension>::Vector(const T element)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] = element;
}

template <typename T, int dimension>
Vector<T, dimension>::Vector(const std::array<T, dimension>& vector)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] = vector[i];
}

template <typename T, int dimension>
Vector<T, dimension>::Vector(const std::initializer_list<T> elements)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] = *(elements.begin()+i);
}

template <typename T, int dimension>
template <typename U>
Vector<T, dimension>::Vector(const Vector<U, dimension>& rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] = T(rhs[i]);
}

template <typename T, int dimension>
T& Vector<T, dimension>::operator[](int index)
{
  return elements_[index];
}

template <typename T, int dimension>
T Vector<T, dimension>::operator[](int index) const
{
  return elements_[index];
}

template <typename T, int dimension>
Vector<T, dimension> Vector<T, dimension>::operator-() const
{
  return (Vector<T, dimension>() - *this);
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator+=(const Vector<T, dimension>& rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] += rhs[i];
  return *this;
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator-=(const Vector<T, dimension>& rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] -= rhs[i];
  return *this;
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator+=(const T rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] += rhs;
  return *this;
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator-=(const T rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] -= rhs;
  return *this;
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator*=(const T rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] *= rhs;
  return *this;
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator*=(const Vector<T, dimension>& rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] *= rhs[i];
  return *this;
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator/=(const T rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] /= rhs;
  return *this;
}

template <typename T, int dimension>
Vector<T, dimension>& Vector<T, dimension>::operator/=(const Vector<T, dimension>& rhs)
{
  for (int i = 0; i < dimension; i++)
    elements_[i] /= rhs[i];
  return *this;
}

template <typename T, int dimension>
double Vector<T, dimension>::norm()
{
  double ans = 0;
  for (int i = 0; i < dimension; i++)
    ans += elements_[i]*elements_[i];
  return std::sqrt(ans);
}

template <typename T, int dimension>
Vector<T, dimension> Vector<T, dimension>::sqrt()
{
  auto ans = *this;
  for (int i = 0; i < dimension; i++)
    ans[i] = std::sqrt(ans[i]);
  return ans;
}


template <typename T, int dimension>
Vector<double, dimension> Vector<T, dimension>::toUnitVector()
{
  Vector<double, dimension> ans(*this);
  ans /= this->norm();
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator+(const Vector<T1, dimension>& lhs,  const Vector<T2, dimension>& rhs)
    -> Vector<decltype(lhs[0]+rhs[0]), dimension>
{
  Vector<decltype(lhs[0]+rhs[0]), dimension> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator+(const Vector<T1, dimension>& lhs,  const T2 rhs)
    -> Vector<decltype(lhs[0]+rhs), dimension>
{
  Vector<decltype(lhs[0]+rhs), dimension> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator+(const T1 lhs,  const Vector<T2, dimension>& rhs)
    -> Vector<decltype(lhs+rhs[0]), dimension>
{
  Vector<decltype(lhs+rhs[0]), dimension> ans(lhs);
  ans += rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator-(const Vector<T1, dimension>& lhs,  const Vector<T2, dimension>& rhs)
    -> Vector<decltype(lhs[0]-rhs[0]), dimension>
{
  Vector<decltype(lhs[0]-rhs[0]), dimension> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator-(const Vector<T1, dimension>& lhs,  const T2 rhs)
    -> Vector<decltype(lhs[0]-rhs), dimension>
{
  Vector<decltype(lhs[0]-rhs), dimension> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator-(const T1 lhs,  const Vector<T2, dimension>& rhs)
    -> Vector<decltype(lhs-rhs[0]), dimension>
{
  Vector<decltype(lhs-rhs[0]), dimension> ans(lhs);
  ans -= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator*(const Vector<T1, dimension>& lhs,  const Vector<T2, dimension>& rhs)
    -> Vector<decltype(lhs[0]*rhs[0]), dimension>
{
  Vector<decltype(lhs[0]*rhs[0]), dimension> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator*(const Vector<T1, dimension>& lhs,  const T2 rhs)
    -> Vector<decltype(lhs[0]*rhs), dimension>
{
  Vector<decltype(lhs[0]*rhs), dimension> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator*(const T1 lhs,  const Vector<T2, dimension>& rhs)
    -> Vector<decltype(lhs*rhs[0]), dimension>
{
  Vector<decltype(lhs*rhs[0]), dimension> ans(lhs);
  ans *= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator/(const Vector<T1, dimension>& lhs,  const T2 rhs)
    -> Vector<decltype(lhs[0]*rhs), dimension>
{
  Vector<decltype(lhs[0]/rhs), dimension> ans(lhs);
  ans /= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
auto operator/(const Vector<T1, dimension>& lhs,  const Vector<T2, dimension>& rhs)
    -> Vector<decltype(lhs[0]*rhs[0]), dimension>
{
  Vector<decltype(lhs[0]/rhs[0]), dimension> ans(lhs);
  ans /= rhs;
  return ans;
}

template <typename T1,  typename T2,  int dimension>
bool operator==(const Vector<T1, dimension>& lhs,  const Vector<T2, dimension>& rhs)
{
  for (int i = 0; i < dimension; i++)
    if (lhs[i] != rhs[i])
      return false;
  return true;
}

}}}  // hyped::utils::math

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_VECTOR_HPP_
