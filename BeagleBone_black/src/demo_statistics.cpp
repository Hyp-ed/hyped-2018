#include <algorithm>  // std::copy
#include <array>
#include <iostream>
#include <iterator>  // std::ostream_iterator
#include <random>

#include "utils/math/statistics.hpp"

using hyped::utils::math::OnlineStatistics;
using hyped::utils::math::RollingStatistics;
using hyped::utils::math::Statistics;

// For printing arrays. Taken from https://stackoverflow.com/a/19152438
template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& o, const std::array<T, N>& arr)
{
    copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
    return o;
}

template <typename T>
void printStatistics(Statistics<T>& s)
{
  std::cout << "\t" << "sum=" << s.getSum() << ", mean=" << s.getMean()
            << ", variance=" << s.getVariance() << ", stddev=" << s.getStdDev() << std::endl;
}

int main()
{
  std::array<float, 1> a1{ {1.0} };
  std::array<float, 6> a2{ {-3, 3, 2, -2, 1, -1} };
  std::array<float, 50000> a3;
  std::default_random_engine generator;
  std::normal_distribution<float> distribution(5.0, 2.0);
  for (std::size_t i = 0; i < a3.size(); ++i)
    a3[i] = distribution(generator);

  OnlineStatistics<float> os1, os21, os22, os30;
  std::array<OnlineStatistics<float>, 5> os3x;
  RollingStatistics<float> rs1(1), rs2(4), rs3(5000);

  os1.update(a1[0]);
  std::cout << "OnlineStats for [" << a1 << "]:" << std::endl;
  printStatistics(os1);
  rs1.update(a1[0]);
  std::cout << "RollingStats for [" << a1 << "]:" << std::endl;
  printStatistics(rs1);
  std::cout << std::endl << std::endl;

  for (auto x : a2) {
    os21.update(x);
    rs2.update(x);
  }
  std::cout << "OnlineStats for [" << a2 << "]:" << std::endl;
  printStatistics(os21);
  std::cout << "RollingStats for [" << a2 << "]:" << std::endl;
  printStatistics(rs2);
  for (std::size_t i = 2; i < a2.size(); ++i)
    os22.update(a2[i]);
  std::cout << "os22:" << std::endl;
  printStatistics(os22);
  std::cout << std::endl << std::endl;

  for (std::size_t i = 0; i < a3.size(); ++i) {
    os30.update(a3[i]);
    rs3.update(a3[i]);
    if ((i/5000)%2 == 1)
      os3x[i/10000].update(a3[i]);
    if ((i+1)%10000 == 0) {
      std::cout << "OnlineStats for a3[" << i - 4999 << ":" << i + 1 << "] (mu=5, std=2):\n";
      printStatistics(os3x[i/10000]);
      std::cout << "RollingStats for a3[" << i - 4999 << ":" << i + 1 << "] (mu=5, std=2):\n";
      printStatistics(rs3);
      std::cout << std::endl;
    }
  }
  std::cout << "OnlineStats for a3 (mu=5, std=2):" << std::endl;
  printStatistics(os30);
  std::cout << "RollingStats for a3 (mu=5, std=2):" << std::endl;
  printStatistics(rs3);
}
