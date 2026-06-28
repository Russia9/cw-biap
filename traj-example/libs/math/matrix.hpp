#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

namespace math {

class matrix {
  public:
    size_t rows;
    size_t cols;
    std::vector<std::vector<double>> data;
    // constructors
    matrix(size_t r, size_t c)
        : rows(r), cols(c), data(r, std::vector<double>(c, 0.0)) {}
    matrix(std::initializer_list<std::initializer_list<double>> init) {
        rows = init.size();
        cols = init.begin()->size();
        data.resize(rows, std::vector<double>(cols));

        int i = 0;
        for (auto& row : init) {
            int j = 0;
            for (double val : row)
                data[i][j++] = val;
            i++;
        }
    }

    // element access
    double& operator()(size_t i, size_t j) { return data[i][j]; }
    double operator()(size_t i, size_t j) const { return data[i][j]; }

    // matrix sum
    matrix operator+(const matrix& another) const {
        if ((rows != another.rows) || (cols != another.cols))
            throw std::invalid_argument(
                "Matrix sum - Invalid matrix dimensions");

        // make an answer holder
        matrix answer(rows, cols);

        // iterate trough rows and columns, add respective indices
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                answer(i, j) = data[i][j] + another(i, j);
        return answer;
    }

    // matrix subtraction
    matrix operator-(const matrix& another) const {
        if ((rows != another.rows) || (cols != another.cols))
            throw std::invalid_argument(
                "Matrix subtraction - Invalid matrix dimensions");

        // make an answer holder
        matrix answer(rows, cols);

        // iterate trough rows and columns, subtract respective indices
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                answer(i, j) = data[i][j] - another(i, j);
        return answer;
    }

    // scalar multiplication (matrix*scalar)
    matrix operator*(double scalar) const {
        matrix answer(rows, cols);
        // for each row and column idices multiply by R
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                answer(i, j) = scalar * data[i][j];

        return answer;
    }

    // matrix multiplication
    matrix operator*(matrix another) const {
        if (cols != another.rows)
            throw std::invalid_argument(
                "Matrix multiplication - Invalid matrix dimensions");

        // make an answer holder
        matrix answer(rows, another.cols);

        // iterate over each row and column, add i_th column of first matrix
        // sequentially multiplied by j_th row of second matrix
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < another.cols; j++)
                for (size_t k = 0; k < cols; k++)
                    answer(i, j) += data[i][k] * another(k, j);
        return answer;
    }

    matrix submatrix(size_t i, size_t j) const {
        // refer to a_(i,j) with an index shift of 0 or 1
        size_t shift_cols = 0;
        size_t shift_rows = 0;

        matrix answer(rows - 1, cols - 1);

        // iterate over rows
        for (size_t i1 = 0; i1 < rows - 1; ++i1) {
            // if crossed marked element row add a shift
            if (i1 >= i)
                shift_cols = 1;
            else
                shift_cols = 0;
            // iterate over cols
            for (size_t j1 = 0; j1 < cols - 1; ++j1) {
                // if crossed marked element column add a shift
                if (j1 >= j)
                    shift_rows = 1;
                else
                    shift_rows = 0;
                answer(i1, j1) = data[i1 + shift_cols][j1 + shift_rows];
            }
        }
        return answer;
    }

    matrix transpose() const {
        matrix answer(cols, rows);
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < cols; ++j)
                answer(j, i) = data[i][j];
        return answer;
    }
};

// compare matrices
inline bool approx_equal(const matrix& a, const matrix& b, double tol = 1e-12) {
    if (a.rows != b.rows || a.cols != b.cols)
        return false;
    for (size_t i = 0; i < a.rows; i++)
        for (size_t j = 0; j < a.cols; j++)
            if (fabs(a(i, j) - b(i, j)) > tol)
                return false;
    return true;
}

namespace detail {

inline int sign_from_parity(int n) {
    return (n % 2 == 0) ? 1 : -1;

    // this is equivalent to
    // if (n % 2 == 0)
    //     return 1;
    // else
    //     return -1;
}
} // namespace detail

// scalar multiplication (the other side, scalar*matrix)
inline matrix operator*(double scalar, const matrix& m) { return m * scalar; }

inline matrix identity_matrix(size_t n) {
    // create matrix containing zeros
    matrix answer(n, n);
    // add 1.0 on each element on a diagonal
    for (size_t i = 0; i < n; i++)
        answer(i, i) = 1.0;
    return answer;
}

inline double determinant(const matrix& A) {
    size_t rows = A.rows;
    size_t cols = A.cols;

    if (rows != cols)
        throw std::invalid_argument(
            "determinant - Input has to be a square matrix");

    // if matrix isn't even a matrix
    if (rows == 1)
        return A(0, 0);

    matrix A_copy = A;
    size_t n = rows;
    double det = 1.0;
    int sign = 1;

    for (size_t i = 0; i < n; ++i) {
        size_t pivot = i;
        for (size_t row = i + 1; row < n; ++row)
            if (fabs(A_copy(row, i)) > fabs(A_copy(pivot, i)))
                pivot = row;

        if (fabs(A_copy(pivot, i)) < 1e-12)
            return 0.0;

        if (pivot != i) {
            std::swap(A_copy.data[pivot], A_copy.data[i]);
            sign *= -1;
        }

        det = det * A_copy(i, i);

        for (size_t row = i + 1; row < n; ++row) {
            double factor = A_copy(row, i) / A_copy(i, i);
            for (size_t col = i; col < n; ++col)
                A_copy(row, col) -= factor * A_copy(i, col);
        }
    }

    return det * sign;
}

// Get inverse of a matrix A (A^-1)
inline matrix inverse(const matrix& A) {
    size_t rows = A.rows;
    size_t cols = A.cols;
    // check if inverse exists (the easy part)
    if (rows != cols)
        throw std::invalid_argument(
            "Inverse matrix - input is a non-square matrix");

    // check if inverse exists (the hard part)
    double det_A = determinant(A);
    if (fabs(det_A) < 1e-12)
        throw std::invalid_argument(
            "Inverse matrix - inverse matrix does not exist");

    matrix C(rows, cols);

    // iterate over rows
    for (size_t i = 0; i < rows; i++)
        // iterate over cols
        for (size_t j = 0; j < rows; j++)
            C(j, i) = detail::sign_from_parity(i + j) *
                      determinant(A.submatrix(i, j));

    return (1.0 / det_A) * C;
}

// Type conversion from column vector to matrix
inline matrix vector_to_matrix(const std::vector<double>& A) {
    size_t len = A.size();
    // make a column vector (which is a matrix to cpp)
    matrix answer(len, 1);

    // fill vector with values
    for (size_t i = 0; i < len; i++)
        answer(i, 0) = A[i];
    return answer;
}

// Type conversion from matrix to column vector
inline std::vector<double> matrix_to_vector(const matrix& A) {
    size_t rows = A.rows;

    for (size_t i = 0; i < rows; i++)
        if (A.data[i].size() > 1 || A.data[i].size() == 0)
            throw std::invalid_argument(
                "matrix_to_vector - no conversion exists from this matrix to "
                "vector (matrix has more than one element in a row).");

    std::vector<double> answer(rows);
    // fill vector with values
    for (size_t i = 0; i < rows; i++)
        answer[i] = A(i, 0);
    return answer;
}

// solves Ax = b using Gaussian elimination with partial pivoting
// returns false if matrix is singular
inline bool gaussian_elimination(matrix A, std::vector<double> b,
                          std::vector<double>& x) {

    // entirety of this has been vibecoded!

    int n = A.rows;
    x.resize(n);

    for (int col = 0; col < n; col++) {
        // find pivot
        int pivot = col;
        for (int row = col + 1; row < n; row++)
            if (fabs(A(row, col)) > fabs(A(pivot, col)))
                pivot = row;

        if (fabs(A(pivot, col)) < 1e-12)
            return false;

        // swap rows
        for (int j = 0; j < n; j++)
            std::swap(A(col, j), A(pivot, j));
        std::swap(b[col], b[pivot]);

        // eliminate below
        for (int row = col + 1; row < n; row++) {
            double factor = A(row, col) / A(col, col);
            for (int j = col; j < n; j++)
                A(row, j) -= factor * A(col, j);
            b[row] -= factor * b[col];
        }
    }

    // back substitution
    for (int i = n - 1; i >= 0; i--) {
        x[i] = b[i];
        for (int j = i + 1; j < n; j++)
            x[i] -= A(i, j) * x[j];
        x[i] /= A(i, i);
    }

    return true;
}

} // namespace math
