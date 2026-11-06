#pragma once

/*
 * 固定维度矩阵数学库：编译期确定大小、无动态内存、不依赖 CMSIS-DSP。
 * 加减要求两矩阵维度相同，乘法要求左矩阵列数等于右矩阵行数；维度错误
 * 会触发 static_assert。矩阵除法不提供，需使用 inverse() 或 solve()。
 *
 * 快速用法：
 *   using Mat = mr::component::math::Matrix<2, 2>;
 *   using Vec = mr::component::math::Matrix<2, 1>;
 *   namespace mm = mr::component::math::matrix;
 *
 *   Mat A = mm::identity<2>();       // 单位矩阵
 *   Vec x;                           // 默认初始化为零
 *   A(0, 1) = 2.0f;                  // 元素访问：A(row, col)
 *   A[0][1] = 2.0f;                  // 下标访问
 *   Mat B = A.transpose();           // 转置
 *   Mat C = A + B;                   // 加法
 *   Mat D = A - B;                   // 减法
 *   Mat E = A * B;                   // 矩阵乘法（维度必须匹配）
 *   Mat F = 2.0f * A;                // 标量乘法
 *   Mat G = A / 2.0f;                // 标量除法
 *   Mat H = -A;                      // 取负
 *
 * 常用工具：
 *   mm::zeros<Rows, Cols>()          // 零矩阵
 *   mm::ones<Rows, Cols>()           // 全 1 矩阵
 *   mm::identity<N>()                // N×N 单位矩阵
 *   mm::diagonal(vector)             // 由列向量生成对角矩阵
 *   mm::is_finite(A)                 // 检查元素是否有限
 *   mm::determinant(A)               // 行列式
 *   A.trace() / A.norm()             // 迹 / Frobenius 范数
 *   A.row(i) / A.col(j)              // 提取行 / 列
 *   A.block<R, C>(r, c)              // 提取子矩阵
 *
 * MATLAB 风格简写（位于 mr::component::math）：
 *   eye<N>(), zeros<R, C>(), ones<R, C>(), diag(v), inv(A, &output)
 *   transpose(A), element_mul(A, B), element_div(A, B)
 *   solve(A, b, &x)                  // 对应 MATLAB 的 A \\ b
 *
 * 求逆和解方程：
 *   Mat inverse;
 *   if (!mm::inverse(A, &inverse)) {}
 *
 *   Vec b, solution;
 *   if (!mm::solve(A, b, &solution)) {}
 *
 * 求逆和求解失败时返回 false；输出会被清零，调用方应检查返回值。
 */

#include <cmath>
#include <cstddef>

#include "component/math/scalar.hpp"

namespace mr::component::math {

// Default absolute pivot threshold shared by the linear solver and filters.
constexpr Scalar kDefaultMinPivot = kDefaultEpsilon;

template <int Rows, int Cols>
class Matrix {
 public:
  static_assert(Rows > 0 && Cols > 0, "Matrix dimensions must be positive");

  Matrix() { fill(0.0f); }

  explicit Matrix(const Scalar* values) {
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        (*this)(row, col) = values[row * Cols + col];
      }
    }
  }

  constexpr int rows() const { return Rows; }
  constexpr int cols() const { return Cols; }

  Scalar& operator()(int row, int col) { return data_[row * Cols + col]; }

  const Scalar& operator()(int row, int col) const { return data_[row * Cols + col]; }

  Scalar* operator[](int row) { return &data_[row * Cols]; }
  const Scalar* operator[](int row) const { return &data_[row * Cols]; }

  Scalar* data() { return data_; }
  const Scalar* data() const { return data_; }

  void fill(Scalar value) {
    for (int i = 0; i < Rows * Cols; ++i) {
      data_[i] = value;
    }
  }

  template <int OtherRows, int OtherCols>
  Matrix& operator+=(const Matrix<OtherRows, OtherCols>& rhs) {
    static_assert(Rows == OtherRows && Cols == OtherCols, "matrix += requires equal dimensions");
    if constexpr (Rows == OtherRows && Cols == OtherCols) {
      for (int i = 0; i < Rows * Cols; ++i) {
        data_[i] += rhs.data_[i];
      }
    }
    return *this;
  }

  template <int OtherRows, int OtherCols>
  Matrix& operator-=(const Matrix<OtherRows, OtherCols>& rhs) {
    static_assert(Rows == OtherRows && Cols == OtherCols, "matrix -= requires equal dimensions");
    if constexpr (Rows == OtherRows && Cols == OtherCols) {
      for (int i = 0; i < Rows * Cols; ++i) {
        data_[i] -= rhs.data_[i];
      }
    }
    return *this;
  }

  Matrix& operator*=(Scalar scalar) {
    for (Scalar& value : data_) value *= scalar;
    return *this;
  }

  Matrix& operator/=(Scalar scalar) {
    for (Scalar& value : data_) value /= scalar;
    return *this;
  }

  Matrix operator-() const {
    Matrix result;
    for (int i = 0; i < Rows * Cols; ++i) result.data_[i] = -data_[i];
    return result;
  }

  template <int OtherCols>
  Matrix<Rows, OtherCols> multiply(const Matrix<Cols, OtherCols>& rhs) const {
    Matrix<Rows, OtherCols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < OtherCols; ++col) {
        Scalar sum = 0.0f;
        for (int k = 0; k < Cols; ++k) {
          sum += (*this)(row, k) * rhs(k, col);
        }
        result(row, col) = sum;
      }
    }
    return result;
  }

  Matrix<Cols, Rows> transpose() const {
    Matrix<Cols, Rows> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(col, row) = (*this)(row, col);
      }
    }
    return result;
  }

  template <int BlockRows, int BlockCols>
  Matrix<BlockRows, BlockCols> block(int start_row, int start_col) const {
    Matrix<BlockRows, BlockCols> result;
    for (int row = 0; row < BlockRows; ++row) {
      for (int col = 0; col < BlockCols; ++col) {
        result(row, col) = (*this)(start_row + row, start_col + col);
      }
    }
    return result;
  }

  Matrix<1, Cols> row(int index) const { return block<1, Cols>(index, 0); }

  Matrix<Rows, 1> col(int index) const { return block<Rows, 1>(0, index); }

  Scalar trace() const {
    static_assert(Rows == Cols, "trace requires a square matrix");
    Scalar result = 0.0f;
    for (int i = 0; i < Rows; ++i) result += (*this)(i, i);
    return result;
  }

  Scalar squared_norm() const {
    Scalar result = 0.0f;
    for (const Scalar value : data_) result += value * value;
    return result;
  }

  Scalar norm() const { return std::sqrt(squared_norm()); }

 private:
  Scalar data_[Rows * Cols];
};

template <int Rows, int Cols, int OtherRows, int OtherCols>
Matrix<Rows, Cols> operator+(Matrix<Rows, Cols> lhs, const Matrix<OtherRows, OtherCols>& rhs) {
  static_assert(Rows == OtherRows && Cols == OtherCols, "matrix + requires equal dimensions");
  if constexpr (Rows == OtherRows && Cols == OtherCols) lhs += rhs;
  return lhs;
}

template <int Rows, int Cols, int OtherRows, int OtherCols>
Matrix<Rows, Cols> operator-(Matrix<Rows, Cols> lhs, const Matrix<OtherRows, OtherCols>& rhs) {
  static_assert(Rows == OtherRows && Cols == OtherCols, "matrix - requires equal dimensions");
  if constexpr (Rows == OtherRows && Cols == OtherCols) lhs -= rhs;
  return lhs;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> operator*(Matrix<Rows, Cols> matrix, Scalar scalar) {
  matrix *= scalar;
  return matrix;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> operator*(Scalar scalar, Matrix<Rows, Cols> matrix) {
  matrix *= scalar;
  return matrix;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> operator/(Matrix<Rows, Cols> matrix, Scalar scalar) {
  matrix /= scalar;
  return matrix;
}

template <int Rows, int Cols, int OtherRows, int OtherCols>
Matrix<Rows, Cols> operator/(const Matrix<Rows, Cols>&, const Matrix<OtherRows, OtherCols>&) {
  static_assert(Rows == 0, "matrix / matrix is not supported; use inverse() or solve()");
  return Matrix<Rows, Cols>();
}

template <int Rows, int Inner, int OtherRows, int Cols>
Matrix<Rows, Cols> operator*(const Matrix<Rows, Inner>& lhs, const Matrix<OtherRows, Cols>& rhs) {
  static_assert(Inner == OtherRows, "matrix * requires lhs columns == rhs rows");
  if constexpr (Inner == OtherRows) {
    return lhs.template multiply<Cols>(rhs);
  } else {
    return Matrix<Rows, Cols>();
  }
}

namespace matrix {

template <int Rows, int Cols>
Matrix<Rows, Cols> zeros() {
  return Matrix<Rows, Cols>();
}

template <int Rows, int Cols>
Matrix<Rows, Cols> ones() {
  Matrix<Rows, Cols> result;
  result.fill(1.0f);
  return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> identity_rect() {
  Matrix<Rows, Cols> result;
  constexpr int diagonal_count = Rows < Cols ? Rows : Cols;
  for (int i = 0; i < diagonal_count; ++i) result(i, i) = 1.0f;
  return result;
}

template <int N>
Matrix<N, N> identity() {
  return identity_rect<N, N>();
}

template <int Rows>
Matrix<Rows, Rows> diagonal(const Matrix<Rows, 1>& values) {
  Matrix<Rows, Rows> result;
  for (int i = 0; i < Rows; ++i) result(i, i) = values(i, 0);
  return result;
}

template <int Rows, int Cols>
bool is_finite(const Matrix<Rows, Cols>& matrix) {
  for (int row = 0; row < Rows; ++row) {
    for (int col = 0; col < Cols; ++col) {
      if (!is_finite_scalar(matrix(row, col))) return false;
    }
  }
  return true;
}

template <int Rows, int Cols>
Scalar dot(const Matrix<Rows, Cols>& lhs, const Matrix<Rows, Cols>& rhs) {
  Scalar result = 0.0f;
  for (int row = 0; row < Rows; ++row) {
    for (int col = 0; col < Cols; ++col) {
      result += lhs(row, col) * rhs(row, col);
    }
  }
  return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> hadamard(const Matrix<Rows, Cols>& lhs, const Matrix<Rows, Cols>& rhs) {
  Matrix<Rows, Cols> result;
  for (int row = 0; row < Rows; ++row) {
    for (int col = 0; col < Cols; ++col) {
      result(row, col) = lhs(row, col) * rhs(row, col);
    }
  }
  return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> element_div(const Matrix<Rows, Cols>& lhs, const Matrix<Rows, Cols>& rhs) {
  Matrix<Rows, Cols> result;
  for (int row = 0; row < Rows; ++row) {
    for (int col = 0; col < Cols; ++col) {
      result(row, col) = lhs(row, col) / rhs(row, col);
    }
  }
  return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> symmetrize(const Matrix<Rows, Cols>& input) {
  static_assert(Rows == Cols, "symmetrize requires a square matrix");
  return 0.5f * (input + input.transpose());
}

template <int N>
Scalar determinant(const Matrix<N, N>& input, Scalar pivot_epsilon = kDefaultEpsilon) {
  Matrix<N, N> work = input;
  Scalar determinant_value = 1.0f;
  int sign = 1;
  for (int pivot = 0; pivot < N; ++pivot) {
    int pivot_row = pivot;
    Scalar pivot_abs = abs_scalar(work(pivot, pivot));
    for (int row = pivot + 1; row < N; ++row) {
      const Scalar candidate = abs_scalar(work(row, pivot));
      if (candidate > pivot_abs) {
        pivot_abs = candidate;
        pivot_row = row;
      }
    }
    if (!is_finite_scalar(pivot_abs) || pivot_abs <= pivot_epsilon) return 0.0f;
    if (pivot_row != pivot) {
      for (int col = 0; col < N; ++col) {
        const Scalar value = work(pivot, col);
        work(pivot, col) = work(pivot_row, col);
        work(pivot_row, col) = value;
      }
      sign = -sign;
    }
    const Scalar diagonal = work(pivot, pivot);
    determinant_value *= diagonal;
    for (int row = pivot + 1; row < N; ++row) {
      const Scalar factor = work(row, pivot) / diagonal;
      for (int col = pivot + 1; col < N; ++col) {
        work(row, col) -= factor * work(pivot, col);
      }
    }
  }
  return static_cast<Scalar>(sign) * determinant_value;
}

// Partial-pivot elimination, with a fixed absolute pivot threshold. Inputs may alias output.
// All failures clear output; no partially computed or non-finite result escapes.
template <int N, int Cols>
[[nodiscard]] bool solve(const Matrix<N, N>& coefficients, const Matrix<N, Cols>& rhs,
                         Matrix<N, Cols>* solution, Scalar pivot_epsilon = kDefaultEpsilon) {
  if (solution == nullptr) return false;
  Matrix<N, N> work = coefficients;
  Matrix<N, Cols> result = rhs;
  *solution = zeros<N, Cols>();
  if (!is_finite(work) || !is_finite(result) || !is_finite_scalar(pivot_epsilon) ||
      pivot_epsilon <= 0)
    return false;
  for (int k = 0; k < N; ++k) {
    int best = k;
    for (int r = k + 1; r < N; ++r)
      if (abs_scalar(work(r, k)) > abs_scalar(work(best, k))) best = r;
    if (!is_finite_scalar(work(best, k)) || abs_scalar(work(best, k)) <= pivot_epsilon)
      return false;
    for (int c = k; c < N; ++c) {
      const Scalar tmp = work(k, c);
      work(k, c) = work(best, c);
      work(best, c) = tmp;
    }
    for (int c = 0; c < Cols; ++c) {
      const Scalar tmp = result(k, c);
      result(k, c) = result(best, c);
      result(best, c) = tmp;
    }
    for (int r = k + 1; r < N; ++r) {
      const Scalar factor = work(r, k) / work(k, k);
      work(r, k) = 0;
      for (int c = k + 1; c < N; ++c) work(r, c) -= factor * work(k, c);
      for (int c = 0; c < Cols; ++c) result(r, c) -= factor * result(k, c);
    }
    if (!is_finite(work) || !is_finite(result)) return false;
  }
  for (int r = N - 1; r >= 0; --r)
    for (int c = 0; c < Cols; ++c) {
      for (int k = r + 1; k < N; ++k) result(r, c) -= work(r, k) * result(k, c);
      result(r, c) /= work(r, r);
    }
  if (!is_finite(result)) return false;
  *solution = result;
  return true;
}

template <int N>
[[nodiscard]] bool inverse(const Matrix<N, N>& input, Matrix<N, N>* output,
                           Scalar pivot_epsilon = kDefaultEpsilon) {
  return matrix::solve(input, identity<N>(), output, pivot_epsilon);
}

// Symmetric positive semidefinite covariance check using a pivoted Schur complement.
// Tolerance permits floating point roundoff, not arbitrary negative variances.
template <int N>
bool is_covariance(const Matrix<N, N>& input, Scalar tolerance = kDefaultEpsilon) {
  if (!is_finite(input) || !is_finite_scalar(tolerance) || tolerance <= 0) return false;
  Scalar scale = 1;
  for (int i = 0; i < N; ++i) scale = std::fmax(scale, abs_scalar(input(i, i)));
  const Scalar tol = tolerance * scale;
  if (!is_finite_scalar(tol)) return false;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      if (abs_scalar(input(i, j) - input(j, i)) > tol) return false;
  Matrix<N, N> work = input;
  for (int k = 0; k < N; ++k) {
    int best = k;
    for (int i = k + 1; i < N; ++i)
      if (work(i, i) > work(best, best)) best = i;
    for (int j = 0; j < N; ++j) {
      const Scalar tmp = work(k, j);
      work(k, j) = work(best, j);
      work(best, j) = tmp;
    }
    for (int i = 0; i < N; ++i) {
      const Scalar tmp = work(i, k);
      work(i, k) = work(i, best);
      work(i, best) = tmp;
    }
    if (work(k, k) < -tol) return false;
    if (work(k, k) <= tol) {
      for (int i = k; i < N; ++i)
        for (int j = k; j < N; ++j)
          if (abs_scalar(work(i, j)) > tol) return false;
      return true;
    }
    for (int i = k + 1; i < N; ++i)
      for (int j = i; j < N; ++j) {
        work(i, j) -= work(i, k) * (work(j, k) / work(k, k));
        work(j, i) = work(i, j);
      }
    if (!is_finite(work)) return false;
  }
  return true;
}

}  // namespace matrix

// MATLAB 风格的简洁入口：eye、zeros、ones、diag、inv、solve 等。
template <int Rows, int Cols>
Matrix<Rows, Cols> zeros() {
  return matrix::zeros<Rows, Cols>();
}

template <int Rows, int Cols>
Matrix<Rows, Cols> ones() {
  return matrix::ones<Rows, Cols>();
}

template <int N>
Matrix<N, N> eye() {
  return matrix::identity<N>();
}

template <int Rows>
Matrix<Rows, Rows> diag(const Matrix<Rows, 1>& values) {
  return matrix::diagonal(values);
}

template <int Rows, int Cols>
Matrix<Rows, Cols> element_mul(const Matrix<Rows, Cols>& lhs, const Matrix<Rows, Cols>& rhs) {
  return matrix::hadamard(lhs, rhs);
}

template <int Rows, int Cols>
Matrix<Rows, Cols> element_div(const Matrix<Rows, Cols>& lhs, const Matrix<Rows, Cols>& rhs) {
  return matrix::element_div(lhs, rhs);
}

template <int Rows, int Cols>
Matrix<Cols, Rows> transpose(const Matrix<Rows, Cols>& input) {
  return input.transpose();
}

template <int N>
[[nodiscard]] bool inv(const Matrix<N, N>& input, Matrix<N, N>* output,
                       Scalar pivot_epsilon = kDefaultEpsilon) {
  return matrix::inverse(input, output, pivot_epsilon);
}

template <int N, int Cols>
bool solve(const Matrix<N, N>& coefficients, const Matrix<N, Cols>& rhs, Matrix<N, Cols>* solution,
           Scalar pivot_epsilon = kDefaultEpsilon) {
  return matrix::solve(coefficients, rhs, solution, pivot_epsilon);
}

namespace vector3f {

inline Matrix<3, 3> hat(const Matrix<3, 1>& vector) {
  Matrix<3, 3> result;
  result(0, 1) = -vector(2, 0);
  result(0, 2) = vector(1, 0);
  result(1, 0) = vector(2, 0);
  result(1, 2) = -vector(0, 0);
  result(2, 0) = -vector(1, 0);
  result(2, 1) = vector(0, 0);
  return result;
}

inline Matrix<3, 1> cross(const Matrix<3, 1>& lhs, const Matrix<3, 1>& rhs) {
  Matrix<3, 1> result;
  result(0, 0) = lhs(1, 0) * rhs(2, 0) - lhs(2, 0) * rhs(1, 0);
  result(1, 0) = lhs(2, 0) * rhs(0, 0) - lhs(0, 0) * rhs(2, 0);
  result(2, 0) = lhs(0, 0) * rhs(1, 0) - lhs(1, 0) * rhs(0, 0);
  return result;
}

}  // namespace vector3f

}  // namespace mr::component::math
