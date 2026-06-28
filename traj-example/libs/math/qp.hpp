#pragma once

#include "matrix.hpp"

namespace math {
// solves equality constrained QP:
// minimize  (1/2) d^T B d + c^T d
// subject to A_eq d = b_eq
// returns step d and multipliers lambda via KKT system
inline bool solve_equality_qp(const matrix& B, const std::vector<double>& c,
                       const matrix& A_eq, const std::vector<double>& b_eq,
                       std::vector<double>& d, std::vector<double>& lambda) {

    int n = B.rows;
    int m = A_eq.rows;
    int total = n + m;

    // build KKT system:
    // [B   A^T] [d]      [-c]
    // [A   0  ] [lambda] = [b]
    matrix KKT(total, total);
    std::vector<double> rhs(total);

    // top left: B
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            KKT(i, j) = B(i, j);

    // top right: A^T
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            KKT(i, n + j) = A_eq(j, i);

    // bottom left: A
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            KKT(n + i, j) = A_eq(i, j);

    // bottom right: 0 (already zero from matrix constructor)

    // rhs: [-c, b]
    for (int i = 0; i < n; i++)
        rhs[i] = -c[i];
    for (int i = 0; i < m; i++)
        rhs[n + i] = b_eq[i];

    std::vector<double> sol;
    if (!gaussian_elimination(KKT, rhs, sol))
        return false;

    // extract d and lambda
    d.assign(sol.begin(), sol.begin() + n);
    lambda.assign(sol.begin() + n, sol.end());

    return true;
}

// solves QP with both equality and inequality constraints using active set
// method: minimize  (1/2) d^T B d + c^T d subject to A_eq d = b_eq (always
// active)
//            A_ineq d >= b_ineq   (active set managed)
// returns step d, equality multipliers lambda, inequality multipliers mu
inline bool QPActiveSet(const matrix& B, const std::vector<double>& c,
                 const matrix& A_eq, const std::vector<double>& b_eq,
                 const matrix& A_ineq, const std::vector<double>& b_ineq,
                 std::vector<double>& d, std::vector<double>& lambda,
                 std::vector<double>& mu, int max_iter = 100) {

    int n = B.rows;
    int n_eq = A_eq.rows;
    int n_ineq = A_ineq.rows;

    // initialize d to zero
    d.assign(n, 0.0);
    mu.assign(n_ineq, 0.0);

    // solve unconstrained QP first to get initial d
    std::vector<double> d_init, lambda_init;
    math::matrix A_empty(0, n);
    std::vector<double> b_empty;
    solve_equality_qp(B, c, A_empty, b_empty, d_init, lambda_init);

    // use d_init to seed the active set check
    d = d_init;

    std::vector<bool> active(n_ineq, false);

    for (int iter = 0; iter < max_iter; iter++) {

        // count active inequality constraints
        int n_active = 0;
        for (bool a : active)
            if (a)
                n_active++;

        // build combined equality system from fixed equalities + active
        // inequalities
        int n_combined = n_eq + n_active;
        matrix A_combined(n_combined, n);
        std::vector<double> b_combined(n_combined);

        // fixed equality constraints
        for (int i = 0; i < n_eq; i++) {
            for (int j = 0; j < n; j++)
                A_combined(i, j) = A_eq(i, j);
            b_combined[i] = b_eq[i];
        }

        // active inequality constraints
        int row = n_eq;
        std::vector<int> active_indices;
        for (int j = 0; j < n_ineq; j++) {
            if (active[j]) {
                for (int k = 0; k < n; k++)
                    A_combined(row, k) = A_ineq(j, k);
                b_combined[row] = b_ineq[j];
                active_indices.push_back(j);
                row++;
            }
        }

        // solve equality QP with current active set
        std::vector<double> d_new, multipliers;
        if (!solve_equality_qp(B, c, A_combined, b_combined, d_new,
                               multipliers))
            return false;

        // extract equality and active inequality multipliers
        // solve_equality_qp returns λ satisfying Bd + A^Tλ = -c, so λ = -μ
        // for inequality constraints; negate to recover the standard μ >= 0
        lambda.assign(multipliers.begin(), multipliers.begin() + n_eq);
        std::vector<double> mu_active(multipliers.begin() + n_eq,
                                      multipliers.end());
        for (auto& m : mu_active) m = -m;

        // check inequality multipliers — if any negative, remove from active
        // set negative multiplier means constraint is pulling wrong way,
        // shouldn't be active
        int most_negative = -1;
        double most_negative_val = -1e-10;
        for (int k = 0; k < (int)active_indices.size(); k++) {
            if (mu_active[k] < most_negative_val) {
                most_negative_val = mu_active[k];
                most_negative = k;
            }
        }

        if (most_negative >= 0) {
            // remove constraint with most negative multiplier
            active[active_indices[most_negative]] = false;
            continue;
        }

        // check if any inactive constraint is violated by d_new
        // if so add most violated one to active set
        int most_violated = -1;
        double most_violated_val = 1e-10;
        for (int j = 0; j < n_ineq; j++) {
            if (active[j])
                continue;

            double val = 0.0;
            for (int k = 0; k < n; k++)
                val += A_ineq(j, k) * d_new[k];

            double violation = b_ineq[j] - val;
            if (violation > most_violated_val) {
                most_violated_val = violation;
                most_violated = j;
            }
        }

        if (most_violated >= 0) {
            active[most_violated] = true;
            d = d_new;
            continue;
        }

        // no negative multipliers, no violated constraints — solution found
        d = d_new;

        // assemble full mu vector
        mu.assign(n_ineq, 0.0);
        for (int k = 0; k < (int)active_indices.size(); k++)
            mu[active_indices[k]] = mu_active[k];
        return true;
    }

    // did not converge — return best d found
    return false;
}
} // namespace math
