#pragma once

#include <cmath>
#include <optional>

#include "types.hpp"

namespace math {

// functions in this namespace do one singular step with a chosen method
// for now unused, since solver is hardcoded to use RK4
namespace steppers {
// generic RK4 stepper
inline std::vector<double> RK4(const math::equation_system& system, double t, const std::vector<double>& y,
                               double t_step) {

    size_t n = system.size();
    std::vector<double> k1(n), k2(n), k3(n), k4(n);
    std::vector<double> y_temp(n);

    // k1
    for (size_t i = 0; i < n; i++)
        k1[i] = system[i](t, y);

    // k2
    for (size_t i = 0; i < n; i++)
        y_temp[i] = y[i] + 0.5 * t_step * k1[i];
    for (size_t i = 0; i < n; i++)
        k2[i] = system[i](t + 0.5 * t_step, y_temp);

    // k3
    for (size_t i = 0; i < n; i++)
        y_temp[i] = y[i] + 0.5 * t_step * k2[i];
    for (size_t i = 0; i < n; i++)
        k3[i] = system[i](t + 0.5 * t_step, y_temp);

    // k4
    for (size_t i = 0; i < n; i++)
        y_temp[i] = y[i] + t_step * k3[i];
    for (size_t i = 0; i < n; i++)
        k4[i] = system[i](t + t_step, y_temp);

    // compute the resulting step
    std::vector<double> y_next(n);
    for (size_t i = 0; i < n; i++)
        y_next[i] = y[i] + (t_step / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    return y_next;
};
} // namespace steppers

namespace detail {
inline bool crossed_zero(double before, double after) {
    constexpr double tol = 1e-12;
    return before * after < 0.0 || std::fabs(after) <= tol;
}

// find point of time when an event g occurs in an interval [t_left, t_right]
// with tolerance tol
static double bisect_event(const math::equation_system& equation_system,
                           const std::function<double(double, const std::vector<double>&)>& g, double t_left,
                           // state at t_left
                           const std::vector<double>& state_left, double t_right, double tol = 1e-10) {
    double g_left = g(t_left, state_left);
    double left_boundary = t_left;
    double right_boundary = t_right;
    std::vector<double> state_mid;

    double t_mid;
    while (right_boundary - left_boundary > tol) {
        t_mid = 0.5 * (right_boundary + left_boundary);
        state_mid = steppers::RK4(equation_system, t_left, state_left, t_mid - t_left);
        double g_mid = g(t_mid, state_mid);

        if (g_left * g_mid < 0)
            right_boundary = t_mid;
        else {
            left_boundary = t_mid;
            g_left = g_mid;
        }
    }

    return 0.5 * (left_boundary + right_boundary);
}
} // namespace detail

/**
     * @brief Generic RK4 ODE system solver with event support
     *
     * @param equation_system       ODE equation system which gets solved, may be
     * swapped trough events.
     * @param t_0                   Initial time.
     * @param initial_conditions    Initial conditions vector.
     * @param step_size             Fixed integration step size.
     * @param t_max                 Max allowed time for solver to run. Solver will
     * always terminate if t_max is exceeded.
     * @param termination_condition Returns negative sign when solver should stop.
     * This is the intended way to terminate sovler.
     * @param observer              Function which gets triggered on each step and
     * event, with no ability to modify state or time.
     * @param on_each_step          Function which gets triggered on each step with
     * access to step and time modification (e. g. for normalisation or energy conservation)
     * @param events                Optional list of events to detect. Each event
     * monitors zero crossing, will trigger if it happens.
     * @return Pair of time and its state vector at termination call
     * (std::pair<double, std::vector<double>>)
     *
     * @note Event callbacks may modify the equation_system in use. Observer gets
     * triggered on each event and step.
     */
    inline std::pair<double, std::vector<double>>
    RK4_solve(math::equation_system &equation_system, double t_0, const std::vector<double> &initial_conditions,
              double step_size, double t_max,
              std::function<double(double, const std::vector<double> &)> termination_condition,
              std::optional<std::function<void(double, const std::vector<double> &)>> observer = std::nullopt,
              std::optional<std::function<void(double, std::vector<double> &)>> on_each_step = std::nullopt,
              std::optional<std::vector<event>> events = std::nullopt)
    {
        // set initial conditions
        double t = t_0;
        std::vector<double> state = initial_conditions;
        std::vector<double> next_state;

        // apply on_each_step
        if (on_each_step.has_value())
            on_each_step.value()(t, state);

        // show first state to observer
        if (observer.has_value())
            observer.value()(t, state);

        // make a pointer to events vector value if they were passed in
        std::vector<event> *evts_val = events.has_value() ? &events.value() : nullptr;
        // if events are present, find their amount
        size_t len_events = evts_val ? evts_val->size() : 0;

        std::vector<double> events_before(len_events);
        std::vector<double> events_after(len_events);

        // compute events and termination before step
        for (size_t i = 0; i < len_events; i++)
            events_before[i] = (*evts_val)[i].g(t, state);

        double termination_before = termination_condition(t, state);
        double termination_after;

        // this will store all instances of events and terminations triggering
        std::vector<std::pair<double, int>> crossings;

        // t_max is the hard limit that will force a return if nothing else returns
        while (t < t_max)
        {

            // citing std::min, "this does what you think it does"
            double dt = std::min(step_size, t_max - t);

            // do step
            next_state = steppers::RK4(equation_system, t, state, dt);
            double t_next = t + dt;

            // compute events after step
            for (size_t i = 0; i < len_events; i++)
                events_after[i] = (*evts_val)[i].g(t_next, next_state);

            // compute termination after step
            termination_after = termination_condition(t_next, next_state);

            // check if any events should be triggered
            for (size_t i = 0; i < len_events; i++)
            {
                // check if something is happening
                if ((*evts_val)[i].is_active && detail::crossed_zero(events_before[i], events_after[i]))
                {
                    // gather all events and their timings
                    double t_cross = detail::bisect_event(equation_system, (*evts_val)[i].g, t, state, t_next);
                    crossings.push_back({t_cross, (int)i});
                }
            }

            // check if termination should occur
            if (detail::crossed_zero(termination_before, termination_after))
            {
                double t_cross = detail::bisect_event(equation_system, termination_condition, t, state, t_next);
                // mark termination with -1
                crossings.push_back({t_cross, -1});
            }

            // if nothing is happening
            if (crossings.empty())
            {
                // advance one step
                state = next_state;
                t = t_next;

                // recompute all events
                for (size_t i = 0; i < len_events; i++)
                {
                    events_before[i] = events_after[i];
                    // unlock events
                    if (!(*evts_val)[i].is_active)
                    {
                        // since system managed to live at least one step without
                        // triggering events, we'll assume it's okay to activate
                        // everything again
                        (*evts_val)[i].is_active = true;
                    }
                }
                // recompute termination
                termination_before = termination_after;

                // apply on_each_step
                if (on_each_step.has_value())
                    on_each_step.value()(t, state);

                // trigger observer
                if (observer.has_value())
                    observer.value()(t, state);
            }
            else // if something should be happening
            {
                // find what should be triggered first
                std::sort(crossings.begin(), crossings.end());

                // step to first event
                state = steppers::RK4(equation_system, t, state, crossings[0].first - t);
                t = crossings[0].first;
                
                // apply on_each_step
                if (on_each_step.has_value())
                    on_each_step.value()(t, state);
                // trigger observer
                if (observer.has_value())
                    observer.value()(t, state);

                // exit if this was termination
                if (crossings[0].second == -1)
                    return {t, state};

                // trigger event
                (*evts_val)[crossings[0].second].on_trigger(t, state);

                // lock that event to avoid the GMOD ragdoll
                (*evts_val)[crossings[0].second].is_active = false;

                // event trigger could have changed the t, state or equation_system
                // recompute everything
                for (size_t i = 0; i < len_events; i++)
                    events_before[i] = (*evts_val)[i].g(t, state);
                termination_before = termination_condition(t, state);
                crossings.clear();
            }
        }
        return {t, state};
    };

} // namespace math
