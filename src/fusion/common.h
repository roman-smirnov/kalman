//
// Created by Roman Smirnov on 2019-04-30.
//

#ifndef COMMON_H_
#define COMMON_H_

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace kalman::fusion::internal {

// machine epsilon of type double (IMPORTANT: not smallest possible positive double)
static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// i.e compile time double constant version of M_PI
static constexpr double kPi = 3.1415926535897932385L;

// i.e 2*M_PI, also known as Tau
static constexpr double kTwoPi = 2.0 * kPi;

// multiply to convert seconds to micro-seconds
static constexpr double kSecToMicrosecFactor = 1.0e-6;

// shorthand for an MxN dimensional matrix of double scalars
template<int M, int N = M>
using Matrix = Eigen::Matrix<double, M, N>;

// column vector
template<int M>
using Vector = Matrix<M, 1>;

// row vector
template<int N>
using RowVector = Matrix<1, N>;

// Returns a matrix constructed from the input submatrices
template<int M, int N = M>
constexpr auto MakeMatrix = [](auto m, auto... ms) { return ((Matrix<M, N>() << m), ..., ms).finished(); };

// Returns an array which elements are the rows of the input matrix
template<int M, int N, typename RowArray = std::array<RowVector<N>, M>>
static constexpr RowArray GetRows(const Matrix<M, N> &m) {
  RowArray rows{};
  for (int i = 0; i < M; ++i) rows[i] = m.row(i);
  return rows;
}

// TODO: constexpr version, and func instead of lambda
constexpr auto Square = [](double x) { return std::pow(x, 2); };
constexpr auto Sqrt = [](double x) { return std::sqrt(x); };

}

#endif //COMMON_H_