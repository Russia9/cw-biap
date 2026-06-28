#pragma once

#include <cmath>
#include <stdexcept>

#include "types.hpp"

// commonly used vector operations

namespace math {

// Compare vectors
inline bool approx_equal(const std::vector<double>& a,
                         const std::vector<double>& b, double tol = 1e-12) {
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); i++)
        if (fabs(a[i] - b[i]) > tol)
            return false;
    return true;
}

// Compute dot product
inline double dot(const std::vector<double>& x, const std::vector<double>& y) {
    size_t x_dim = x.size();
    size_t y_dim = y.size();
    if (x_dim != y_dim)
        throw std::invalid_argument(
            "dot product - vectors x and y must be of same dimensions");

    double ans = 0.0;
    for (size_t i = 0; i < x_dim; i++)
        ans += x[i] * y[i];
    return ans;
}

// Compute norm (magnitude) of a vector using Euclidean norm
inline double norm(const std::vector<double>& vec) {
    size_t vec_dim = vec.size();
    double ans = 0;
    for (size_t i = 0; i < vec_dim; i++)
        ans += vec[i] * vec[i];
    return sqrt(ans);
}

// Normalize vector coordinates
inline void normalize(std::vector<double>& vec) {
    size_t len = vec.size();
    double mag = norm(vec);
    for (size_t i = 0; i < len; i++)
        vec[i] = vec[i]/mag;
}

// add vector A to vector B
inline std::vector<double> add(const std::vector<double>& A,
                        const std::vector<double>& B) {
    size_t dim_A = A.size();
    size_t dim_B = B.size();
    if (dim_A != dim_B)
        throw std::invalid_argument(
            "vector addition - vector dimensions must match.");

    std::vector<double> ans = A;
    for (size_t i = 0; i < dim_A; i++)
        ans[i] += B[i];
    return ans;
}

// subtract vector B from vector A
inline std::vector<double> subtract(const std::vector<double>& A,
                             const std::vector<double>& B) {
    size_t dim_A = A.size();
    size_t dim_B = B.size();
    if (dim_A != dim_B)
        throw std::invalid_argument(
            "vector subtraction - vector dimensions must match.");

    std::vector<double> ans = A;
    for (size_t i = 0; i < dim_A; i++)
        ans[i] -= B[i];
    return ans;
}

inline std::vector<double> scale(double s, const std::vector<double>& A) {
    size_t dim_A = A.size();

    std::vector<double> ans = A;
    for (size_t i = 0; i < dim_A; i++)
        ans[i] *= s;
    return ans;
}

// compute value of a vector function V at point
inline std::vector<double> Vector_func_value(const math::vector_function& V,
                                      const std::vector<double>& point) {
    size_t v_dim = V.size();
    size_t p_dim = point.size();
    if (v_dim != p_dim)
        throw std::invalid_argument(
            "Vector function value - Dimensions of vector function and its "
            "point must match.\n");

    std::vector<double> answer(p_dim);
    for (size_t i = 0; i < v_dim; i++)
        answer[i] = V[i](point);
    return answer;
}

} // namespace math
