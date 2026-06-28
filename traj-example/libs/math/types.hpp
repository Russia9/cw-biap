#pragma once

#include <functional>
#include <vector>

// Header for all types used in other headers.

// technically these are not types, but aliases
namespace math {
// A column vector of functions, which all depend on same vector
using vector_function =
    std::vector<std::function<double(const std::vector<double>&)>>;

// In a differential equation system a singular equation is usually a function,
// which is equal to some derivative d(...)/dt Left hand side (LHS) would be the
// d(...)/dt, the derivative Right hand side (RHS) would be the function that
// computes the derivative thus, an equation is LHS = RHS LHS gets called during
// solver loop, LHS points solver to RHS
using equation_RHS = std::function<double(double, const std::vector<double>&)>;

// an equation system is a list of equations
using equation_system = std::vector<equation_RHS>;

// a scalar function that depends on a vector
using multi_variable_single_value_function =
    std::function<double(const std::vector<double>&)>;

// an event for handling non-smooth functions in ODE solver
struct event {
    // An event detector function, its value has to cross 0 at some point in
    // time. when it crosses 0, solver will see the change and attempt to find
    // the point where zero crossing happens after that, it'll step right to
    // that crossing point, then do whatever is said in on_trigger.
    std::function<double(double, const std::vector<double>&)> g;
    // on_trigger will be executed each time an event is triggered (g crossed 0)
    std::function<void(double&, std::vector<double>&)> on_trigger;
    bool is_active = true;
};

struct points
{
    std::vector<double> x;
    std::vector<double> y;
};

} // namespace math
