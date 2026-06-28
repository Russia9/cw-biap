#pragma once

#include <optional>
#include <stdexcept>

#include "matrix.hpp"
#include "types.hpp"

namespace math {

// Compute partial derivative of function F with respect to index i at point x
inline double derivative_vec(const math::multi_variable_single_value_function& F,
                      const std::vector<double>& x, std::size_t i,
                      double h = 1e-7) {
    std::vector<double> x_plus = x;
    std::vector<double> x_minus = x;
    x_plus[i] += h;
    x_minus[i] -= h;
    double f_plus = F(x_plus);
    double f_minus = F(x_minus);
    return (f_plus - f_minus) / (2.0 * h);
}

// Compute Jacobian of function V at point
inline math::matrix Jacobian_matrix(const math::vector_function& V,
                             const std::vector<double>& point) {
    size_t v_dim = V.size();
    size_t p_dim = point.size();
    if (v_dim != p_dim)
        throw std::invalid_argument("Jacobian matrix - Dimensions of vector "
                                    "function and its point must match.\n");

    math::matrix J(v_dim, v_dim);
    for (size_t i = 0; i < v_dim; i++)
        for (size_t j = 0; j < v_dim; j++) {
            J(i, j) = derivative_vec(V[i], point, j);
        }
    return J;
}

// Compute gradient of function F at point, supports custom steps for each point coordinate
inline std::vector<double> gradient(math::multi_variable_single_value_function F, size_t F_dim,
         const std::vector<double>& point,
         std::optional<std::vector<double>> h = std::nullopt) {
    if (!h.has_value()) {
        h = std::vector<double>(F_dim, 1e-7);
    }

    std::vector<double> answer(F_dim);
    for (size_t i = 0; i < F_dim; i++)
        answer[i] = derivative_vec(F, point, i, h.value()[i]);
    return answer;
}

} // namespace math
