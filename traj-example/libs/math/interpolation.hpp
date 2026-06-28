#pragma once
#include <functional>
#include <vector>
#include <stdexcept>
#include "types.hpp"

namespace math
{
    // Make a linear interpolating function from two points on a plane.
    std::function<double(double)> linear_function_from2points(double x1, double x2, double y1, double y2)
    {
        double k = (y2 - y1) / (x2 - x1);
        double b = y1 - k * x1;
        return [k, b](double x)
        {
            return k * x + b;
        };
    };

    std::function<double(double)> CubicSpline(math::points pts)
    {
        if (pts.x.size() != pts.y.size())
            throw std::invalid_argument("CubicSpline - point coordinate vectors size mismatch");

        size_t n = pts.x.size();

        // compute interval lengths

        std::vector<double> h(n - 1);
        for (size_t i = 0; i < n - 1; ++i)
        {
            h[i] = pts.x[i + 1] - pts.x[i];
            if (h[i] <= 0)
                throw std::invalid_argument("CubicSpline - x coordinate must be strictly increasing");
        }

        std::vector<double> a(n), b(n), c(n), d(n);

        b[0] = 1.0;
        d[0] = 0.0;

        for (size_t i = 1; i < n - 1; ++i)
        {
            a[i] = h[i - 1];
            b[i] = 2.0 * (h[i - 1] + h[i]);
            c[i] = h[i];
            d[i] = 6.0 * ((pts.y[i + 1] - pts.y[i]) / h[i] - (pts.y[i] - pts.y[i - 1]) / h[i - 1]);
        }

        b[n - 1] = 1.0;
        d[n - 1] = 0.0;

        std::vector<double> M(pts.x.size());

        std::vector<double> c_prime(n), d_prime(n);

        c_prime[0] = c[0] / b[0];
        d_prime[0] = d[0] / b[0];

        for (size_t i = 1; i < n; ++i)
        {
            double m = b[i] - a[i] * c_prime[i - 1];
            c_prime[i] = c[i] / m;
            d_prime[i] = (d[i] - a[i] * d_prime[i - 1]) / m;
        }

        M[n - 1] = d_prime[n - 1];
        for (int i = n - 2; i >= 0; --i)
        {
            M[i] = d_prime[i] - c_prime[i] * M[i + 1];
        }

        return [M, pts](double x)
        {
            if (x < pts.x.front() || x > pts.x.back())
                throw std::out_of_range("x out of bounds");

            size_t i = std::upper_bound(pts.x.begin(), pts.x.end(), x) - pts.x.begin() - 1;

            double h = pts.x[i + 1] - pts.x[i];
            double xi = pts.x[i];
            double xi1 = pts.x[i + 1];

            double term1 = M[i] * (xi1 - x) * (xi1 - x) * (xi1 - x) / (6.0 * h);
            double term2 = M[i + 1] * (x - xi) * (x - xi) * (x - xi) / (6.0 * h);
            double term3 = (pts.y[i] / h - M[i] * h / 6.0) * (xi1 - x);
            double term4 = (pts.y[i + 1] / h - M[i + 1] * h / 6.0) * (x - xi);

            return term1 + term2 + term3 + term4;
        };
    }
}