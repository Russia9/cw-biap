#pragma once

#include <functional>

namespace math {

// searches for step size satisfying Armijo (sufficient decrease) condition
// phi:     merit function along search direction, phi(0) is current value
// phi0:    phi(0)
// dphi0:   directional derivative at alpha=0, must be negative for descent
// alpha:   initial step size to try
// c:       sufficient decrease constant (typically 0.1 to 0.5)
// max_iter: maximum number of backtracking steps
// returns accepted step size, or smallest tried if condition never met
inline double LineSearchArmijo(std::function<double(double)> phi, double phi0,
                        double dphi0, double alpha, double c, int max_iter) {
    for (int i = 0; i < max_iter; i++) {
        // armijo condition: sufficient decrease
        if (phi(alpha) <= phi0 + c * alpha * dphi0)
            return alpha;

        // backtrack by halving
        alpha *= 0.5;
    }

    return alpha;
}

} // namespace math
