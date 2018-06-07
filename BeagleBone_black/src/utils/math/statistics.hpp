/*
 * Author: Branislav Pilnan
 * Organisation: HYPED
 * Date: 06/06/2018
 * Description: Algorithms for computing online/rolling mean, variance, and standard deviation
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

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_STATISTICS_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_STATISTICS_HPP_

#include <cassert>
#include <cmath>
#include <list>
#include <queue>

namespace hyped {
namespace utils {
namespace math {

template <typename T>
class Statistics {
 public:
  Statistics();
  virtual void update(T new_value) = 0;
  T getSum() {return sum_;}
  T getMean() {return mean_;}
  T getVariance() {return variance_;}
  T getStdDev() {return std::sqrt(variance_);}

 protected:
  T sum_;
  T mean_;
  T variance_;
};

template <typename T>
Statistics<T>::Statistics() : sum_(0), mean_(0), variance_(0)
{}


template <typename T>
class OnlineStatistics : public Statistics<T> {
 public:
  OnlineStatistics();
  void update(T new_value) override;

 private:
  int n_;
  T s_;  // sum of squared differences from mean
};

template <typename T>
OnlineStatistics<T>::OnlineStatistics() : n_(0), s_(0)
{}

template <typename T>
void OnlineStatistics<T>::update(T new_value)
{
  T delta = new_value - this->mean_;
  n_++;
  this->sum_ += new_value;
  this->mean_ = this->sum_/n_;
  s_ += delta * (new_value - this->mean_);
  this->variance_ = s_ / (n_ - 1);
}


template <typename T>
class RollingStatistics : public Statistics<T> {
  public:
  explicit RollingStatistics(std::size_t window_size);
  void update(T new_value) override;

 private:
  const std::size_t window_size_;
  std::queue<T, std::list<T>> window_;
  OnlineStatistics<T> online_;
};

template <typename T>
RollingStatistics<T>::RollingStatistics(std::size_t window_size) : window_size_(window_size)
{}

template <typename T>
void RollingStatistics<T>::update(T new_value)
{
  if (window_.size() < window_size_) {
    window_.push(new_value);
    online_.update(new_value);
    this->sum_      = online_.getSum();
    this->mean_     = online_.getMean();
    this->variance_ = online_.getVariance();
  } else {
    assert(window_.size() == window_size_);
    this->sum_ = this->sum_ + new_value - window_.front();
    T new_mean = this->sum_ / window_size_;
    this->variance_ += (new_value - window_.front())
                        * (new_value - new_mean + window_.front() - this->mean_)
                        / (window_size_ - 1);
    this->mean_ = new_mean;
    window_.pop();
    window_.push(new_value);
  }
}

}}}  // namespace hyped::utils::math

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_STATISTICS_HPP_
