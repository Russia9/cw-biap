#pragma once

#include "derivative.hpp"
#include "types.hpp"
#include "vector.hpp"
#include "qp.hpp"

namespace math {

namespace detail {

// updates BFGS hessian approximation B given step s and gradient change y
// returns false if update is skipped (curvature condition not satisfied)
inline bool BFGSUpdate(math::matrix& B, const std::vector<double>& s, const std::vector<double>& y) {
    double sy = math::dot(s, y);

    // curvature condition — if not satisfied, skip update to keep B positive
    // definite
    if (sy <= 1e-10)
        return false;

    int n = s.size();
    std::vector<double> Bs(n, 0.0);

    // compute B*s
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Bs[i] += B(i, j) * s[j];

    double sBs = math::dot(s, Bs);

    if (std::fabs(sBs) < 1e-10)
        return false;

    // BFGS formula: B = B + (y*y^T)/(y^T*s) - (B*s*s^T*B)/(s^T*B*s)
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            B(i, j) += y[i] * y[j] / sy - Bs[i] * Bs[j] / sBs;

    return true;
}

// Compute lagrangian gradient at point x
inline std::vector<double> lagGrad(math::multi_variable_single_value_function f,
                            std::vector<math::multi_variable_single_value_function> eqConstraints,
                            std::vector<math::multi_variable_single_value_function> ineqConstraints,
                            const std::vector<double>& x, const std::vector<double>& lambda_eq,
                            const std::vector<double>& mu_ineq) {
    size_t dim = x.size();

    std::vector<double> ans = math::gradient(f, dim, x);

    // add equality constraints
    for (size_t i = 0; i < eqConstraints.size(); i++) {
        std::vector<double> grad_g = math::gradient(eqConstraints[i], dim, x);

        for (size_t j = 0; j < dim; j++) {
            ans[j] += lambda_eq[i] * grad_g[j];
        }
    }

    // subtract inequality constraints
    for (size_t i = 0; i < ineqConstraints.size(); i++) {
        std::vector<double> grad_h = math::gradient(ineqConstraints[i], dim, x);

        for (size_t j = 0; j < dim; j++) {
            ans[j] -= mu_ineq[i] * grad_h[j];
        }
    }

    return ans;
}

} // namespace detail

/// @brief Sequential least squares quadratic programming algorithm for finding minimum of function f,
/// subject to equality and inequality constraints.
/// @param f                Multi variable function with single value.
/// @param x0               Initial guess.
/// @param deltaX           Step sizes for computing derivatives for each point coordinate.
/// @param eqConstraints    Vector of equality constraints.
/// @param ineqConstraints  Vector of inequality constraints.
/// @param tol              Solution tolerance.
/// @param max_iterations   Maximum allowed number of iterations.
/// @param observer         Function for tracking optimization progress.
/// @return                 Point x, at which f is minimal with all constraints applied.
inline std::vector<double> SLSQP(math::multi_variable_single_value_function f, const std::vector<double>& x0,
                          const std::vector<double>& deltaX,
                          std::vector<math::multi_variable_single_value_function> eqConstraints,
                          std::vector<math::multi_variable_single_value_function> ineqConstraints, double tol,
                          size_t max_iterations,
                          std::function<void(size_t, double, double)> observer = nullptr) {

    size_t n = x0.size();
    size_t len_eq = eqConstraints.size();
    size_t len_ineq = ineqConstraints.size();

    auto x = x0;

    // approximate Lagrangian Hessian with identity matrix
    math::matrix B = math::identity_matrix(n);

    // множители Лагранжа
    std::vector<double> lambda_eq(len_eq, 0.0);
    std::vector<double> mu_ineq(len_ineq, 0.0);

    // матрицы и векторы ограничений
    math::matrix A_eq(len_eq, n);
    math::matrix A_ineq(len_ineq, n);

    std::vector<double> g_eq(len_eq, 0.0);
    std::vector<double> h_ineq(len_ineq, 0.0);

    for (size_t iter = 0; iter < max_iterations; iter++) {
        auto grad_f = math::gradient(f, n, x, deltaX);
        for (size_t i = 0; i < len_eq; i++) {
            g_eq[i] = eqConstraints[i](x);
            auto grad = math::gradient(eqConstraints[i], n, x, deltaX);
            for (size_t j = 0; j < n; j++) {
                A_eq(i, j) = grad[j];
            }
        }

        for (size_t i = 0; i < len_ineq; i++) {
            h_ineq[i] = ineqConstraints[i](x);
            auto grad = math::gradient(ineqConstraints[i], n, x, deltaX);
            for (size_t j = 0; j < n; j++) {
                A_ineq(i, j) = grad[j];
            }
        }

        auto grad_L = detail::lagGrad(f, eqConstraints, ineqConstraints, x, lambda_eq, mu_ineq);

        // A*d = -g,  A*d >= -h
        std::vector<double> b_eq(len_eq);
        std::vector<double> b_ineq(len_ineq);
        for (size_t i = 0; i < len_eq; i++)
            b_eq[i] = -g_eq[i];
        for (size_t i = 0; i < len_ineq; i++)
            b_ineq[i] = -h_ineq[i];

        double f_val = f(x);

        std::vector<double> d(n, 0.0);
        std::vector<double> lambda_qp, mu_qp;
        bool qp_norm = QPActiveSet(B, grad_f, A_eq, b_eq, A_ineq, b_ineq, d, lambda_qp, mu_qp);
        if (observer) observer(iter, f_val, math::norm(d));
        if (!qp_norm)
            break;

        if (math::norm(d) < tol) {
            lambda_eq = lambda_qp;
            mu_ineq = mu_qp;
            break;
        }

        double merit_old = f_val;
        for (size_t i = 0; i < len_eq; i++)
            merit_old += 100.0 * std::fabs(g_eq[i]);
        for (size_t i = 0; i < len_ineq; i++)
            if (h_ineq[i] < 0)
                merit_old += 100.0 * std::fabs(h_ineq[i]);

        double alpha = 1.0;
        std::vector<double> x_new(n);
        bool step_norm = false;

        for (int ls = 0; ls < 20; ls++) {
            for (size_t i = 0; i < n; i++)
                x_new[i] = x[i] + alpha * d[i];

            double merit_new = f(x_new);
            for (size_t i = 0; i < len_eq; i++)
                merit_new += 100.0 * std::fabs(eqConstraints[i](x_new));
            for (size_t i = 0; i < len_ineq; i++) {
                double val = ineqConstraints[i](x_new);
                if (val < 0)
                    merit_new += 100.0 * std::fabs(val);
            }

            if (merit_new < merit_old) {
                step_norm = true;
                break;
            }
            alpha *= 0.5;
        }
        if (!step_norm)
            break;

        std::vector<double> s(n);
        for (size_t i = 0; i < n; i++)
            s[i] = x_new[i] - x[i];

        auto grad_L_new = detail::lagGrad(f, eqConstraints, ineqConstraints, x_new, lambda_qp, mu_qp);

        std::vector<double> y(n);
        for (size_t i = 0; i < n; i++)
            y[i] = grad_L_new[i] - grad_L[i];
        detail::BFGSUpdate(B, s, y);
        x = x_new;
        lambda_eq = lambda_qp;
        mu_ineq = mu_qp;
    }
    return x;
}

} // namespace math
