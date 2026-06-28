#pragma once
#include <cmath>
#include <stdexcept>
#include "libs/math/types.hpp"
#include "libs/math/interpolation.hpp"
#include "libs/atmosphere/Lueft.hpp"
#include "rocket_calc/rocket_aerodynamics.hpp"
#include <iostream>

const double R_earth_m = 6371000;
const double mu_Earth = 3.986e14;
const double g0 = 9.80665;

enum state_vector
{
    vx,
    vy,
    x,
    y,
    m,
    pitch,
    omega_z,
};

inline double g_x(const std::vector<double> &s)
{
    return -1.0 * ((mu_Earth * s[x]) / (pow((s[x] * s[x] + (s[y] + R_earth_m) * (s[y] + R_earth_m)), 1.5)));
}

inline double g_y(const std::vector<double> &s)
{
    return -1.0 * ((mu_Earth * (s[y] + R_earth_m)) / (pow((s[x] * s[x] + (s[y] + R_earth_m) * (s[y] + R_earth_m)), 1.5)));
}

inline double velocity_mag(const std::vector<double> &s)
{
    return hypot(s[vx], s[vy]);
};

inline double radius(const std::vector<double> &s)
{
    return hypot(s[x], s[y] + R_earth_m);
}

inline double altitude(const std::vector<double> &s)
{
    return hypot(s[x], s[y] + R_earth_m) - R_earth_m;
}

inline double angular_distance(const std::vector<double> &s)
{
    return atan2(s[x], s[y] + R_earth_m);
};

inline double distance(const std::vector<double> &s)
{
    return angular_distance(s) * R_earth_m;
}

inline double flight_angle(const std::vector<double> &s)
{
    return atan2(s[vy], s[vx]);
}

inline double aoa(double pitch, double flight_angle)
{
    return pitch - flight_angle;
}

// inline double thrust(double P_thrust_sea_level, double S_a, double combustion_chamber_pressure, double current_pressure)
// {
//     return (P_thrust_sea_level + S_a * (combustion_chamber_pressure - current_pressure));
// }

inline double thrust_interp(double spec_thrust_sea_level, double spec_thrust_vacuum, double current_pressure, double dmdt)
{
    double p_clamped = std::clamp(current_pressure, 0.0, 101325.0);
    //std::cerr << math::linear_function_from2points(0.0, 101325.0, spec_thrust_vacuum, spec_thrust_sea_level)(p_clamped) * dmdt * g0 << "\n";
    return math::linear_function_from2points(0.0, 101325.0, spec_thrust_vacuum, spec_thrust_sea_level)(p_clamped) * dmdt * g0;
}

// inline double dmdt(double delta_t_burn, double m_full, double m_empty)
// {
//     return (m_empty - m_full) / delta_t_burn;
// }

inline double I_z(double mass, double r, double length)
{
    return 3.0 / 5.0 * mass * (0.25 * r * r + length * length);
}

struct pitch_control
{
    double t_vertical;
    double theta_k1;
    double theta_k2;
    double t_k1;
    double t_k2;
};

double pitch_programmed(double t, pitch_control p)
{
    // if (t > p.t_k2)
    //     std::cerr << t << " out of pitch program range! max t is " << p.t_k2 << "\n";
    t = std::clamp(t, 0.0, p.t_k2);
    if (t <= p.t_vertical)
        return M_PI_2;
    else if (p.t_vertical < t && t <= p.t_k1)
        return (M_PI_2 + p.theta_k1) * 0.5 + (M_PI_2 - p.theta_k1) * 0.5 * cos(M_PI * pow(((t - p.t_vertical) / (p.t_k1 - p.t_vertical)), 1.1));
    else if (p.t_k1 < t && t <= p.t_k2)
        return (p.theta_k1 + p.theta_k2) * 0.5 + (p.theta_k1 - p.theta_k2) * 0.5 * cos(M_PI * pow(((t - p.t_k1) / (p.t_k2 - p.t_k1)), 1.1));
    else
        throw std::out_of_range("t in pitch program out of range (must be in [" + std::to_string(p.t_k1) + ", " + std::to_string(p.t_k2) + "])\n");
}

double q_dynamic_pressure(const std::vector<double> &s)
{
    double v = velocity_mag(s);
    return Lueft::density(altitude(s)) * v * v * 0.5;
}

double mass_rate(double beta)
{
    return beta;
}

double C_x(size_t stage_nr, double Mach, double aoa)
{
    aoa = aoa / M_PI * 180.0;

    switch (stage_nr)
    {
    case 1:
        return C_x_1st_stage(Mach, aoa);
        break;

    case 2:
        return C_x_2nd_stage(Mach, aoa);
        break;

    case 3:
        return C_x_3rd_stage(Mach, aoa);
        break;
    default:
        return C_x_3rd_stage(Mach, aoa);
    }
}

double C_y(size_t stage_nr, double Mach, double aoa)
{
    aoa = aoa / M_PI * 180.0;
    switch (stage_nr)
    {
    case 1:
        return C_y_1st_stage(Mach, aoa);
        break;

    case 2:
        return C_y_2nd_stage(Mach, aoa);
        break;

    case 3:
        return C_y_3rd_stage(Mach, aoa);
        break;
    default:
        return C_y_3rd_stage(Mach, aoa);
    }
}

double m_z(size_t stage_nr, double Mach, double aoa)
{
    aoa = aoa / M_PI * 180.0;
    return m_z_3rd_stage(Mach, aoa);
}

double X_aero(size_t stage_nr, double S_mid, const std::vector<double> &s, double aoa)
{
    double Mach = velocity_mag(s) / Lueft::sonicspeed(altitude(s));
    double q = q_dynamic_pressure(s);
    return C_x(stage_nr, Mach, aoa) * q * S_mid;
}

double Y_aero(size_t stage_nr, double S_mid, const std::vector<double> &s, double aoa)
{
    double Mach = velocity_mag(s) / Lueft::sonicspeed(altitude(s));
    double q = q_dynamic_pressure(s);
    return C_y(stage_nr, Mach, aoa) * q * S_mid;
}

double M_z_aero(size_t stage_nr, double S_mid, double L, const std::vector<double> &s, double aoa)
{
    if (stage_nr != 3)
        throw std::runtime_error("Only stage 3 has aerodynamic moment!\n");
    double Mach = velocity_mag(s) / Lueft::sonicspeed(altitude(s));
    double q = q_dynamic_pressure(s);
    return m_z(stage_nr, Mach, aoa) * q * S_mid * L;
}

double x_accel_atm_w_thrust(double t, const std::vector<double> &s,
                            double spec_thrust_sea_level, double spec_thrust_vacuum, size_t stage_nr, double S_mid, double m_fuel, double t_burn, pitch_control p)
{
    double P = thrust_interp(spec_thrust_sea_level, spec_thrust_vacuum, Lueft::pressure(altitude(s)), mass_rate(m_fuel / t_burn));
    double theta_flight_angle = flight_angle(s);
    double aoa_alpha = aoa(pitch_programmed(t, p), flight_angle(s));
    double X = X_aero(stage_nr, S_mid, s, aoa_alpha);
    double Y = Y_aero(stage_nr, S_mid, s, aoa_alpha);

    return (P * cos(pitch_programmed(t, p)) - X * cos(theta_flight_angle) - Y * sin(theta_flight_angle)) / s[m] + g_x(s);
}

double y_accel_atm_w_thrust(double t, const std::vector<double> &s,
                            double spec_thrust_sea_level, double spec_thrust_vacuum, size_t stage_nr, double S_mid, double m_fuel, double t_burn, pitch_control p)
{
    double P = thrust_interp(spec_thrust_sea_level, spec_thrust_vacuum, Lueft::pressure(altitude(s)), mass_rate(m_fuel / t_burn));
    double theta_flight_angle = flight_angle(s);
    double aoa_alpha = aoa(pitch_programmed(t, p), flight_angle(s));
    double X = X_aero(stage_nr, S_mid, s, aoa_alpha);
    double Y = Y_aero(stage_nr, S_mid, s, aoa_alpha);

    return (P * sin(pitch_programmed(t, p)) - X * sin(theta_flight_angle) + Y * cos(theta_flight_angle)) / s[m] + g_y(s);
}

double x_accel_no_atm_w_thrust(double t, const std::vector<double> &s,
                               double spec_thrust_sea_level, double spec_thrust_vacuum, double m_fuel, double t_burn, pitch_control p)
{
    double P = thrust_interp(spec_thrust_sea_level, spec_thrust_vacuum, 0.0, mass_rate(m_fuel / t_burn));

    return (P * cos(pitch_programmed(t, p))) / s[m] + g_x(s);
}

double y_accel_no_atm_w_thrust(double t, const std::vector<double> &s,
                               double spec_thrust_sea_level, double spec_thrust_vacuum, double m_fuel, double t_burn, pitch_control p)
{
    double P = thrust_interp(spec_thrust_sea_level, spec_thrust_vacuum, 0.0, mass_rate(m_fuel / t_burn));

    return (P * sin(pitch_programmed(t, p))) / s[m] + g_y(s);
}

double x_accel_atm_no_thrust(double t, const std::vector<double> &s,
                             size_t stage_nr, double S_mid, pitch_control p)
{
    double theta_flight_angle = flight_angle(s);
    double aoa_alpha = aoa(s[pitch], flight_angle(s));
    double X = X_aero(stage_nr, S_mid, s, aoa_alpha);
    double Y = Y_aero(stage_nr, S_mid, s, aoa_alpha);

    return (-X * cos(theta_flight_angle) - Y * sin(theta_flight_angle)) / s[m] + g_x(s);
}

double y_accel_atm_no_thrust(double t, const std::vector<double> &s,
                             size_t stage_nr, double S_mid, pitch_control p)
{
    double theta_flight_angle = flight_angle(s);
    double aoa_alpha = aoa(s[pitch], flight_angle(s));
    double X = X_aero(stage_nr, S_mid, s, aoa_alpha);
    double Y = Y_aero(stage_nr, S_mid, s, aoa_alpha);

    return (-X * sin(theta_flight_angle) + Y * cos(theta_flight_angle)) / s[m] + g_y(s);
}

double x_accel_no_atm_no_thrust(double t, const std::vector<double> &s)
{
    return g_x(s);
}

double y_accel_no_atm_no_thrust(double t, const std::vector<double> &s)
{
    return g_y(s);
}

double dxdt(double t, const std::vector<double> &s)
{
    return s[vx];
}

double dydt(double t, const std::vector<double> &s)
{
    return s[vy];
}

double z_angular_accel_atm(double t, const std::vector<double> &s,
                           size_t stage_nr, double S_mid, double L, double pitch_angle)
{
    double r_mid = sqrt(S_mid / M_PI);
    double theta_flight_angle = flight_angle(s);
    double aoa_alpha = aoa(s[pitch], flight_angle(s));
    return M_z_aero(stage_nr, S_mid, L, s, aoa_alpha) / I_z(s[m], r_mid, L);
}

double dthetadt(double t, const std::vector<double> &s)
{
    return s[omega_z];
}

/*
enum state_vector
{
    vx,
    vy,
    x,
    y,
    m,
    pitch,
    omega_z,
};
*/

/// @attention State vector length is 5.
/// @param spec_thrust_sea_level specific thrust at sea level
/// @param spec_thrust_vacuum    specific thrust in vacuum
/// @param stage_nr              stage number (for aerodynamics)
/// @param S_mid                 Midsection area
/// @param m_fuel_total          Total fuel that will be burned by this stage
/// @param t_burn                Burn duration
/// @return ODE equation system for describing powered rocket flight in atmosphere
math::equation_system rocket_atm_w_thrust(double spec_thrust_sea_level, double spec_thrust_vacuum, size_t stage_nr,
                                          double S_mid, double m_fuel_total, double t_burn, pitch_control p)
{
    auto x_accel = [=](double t, const std::vector<double> &s)
    {
        return x_accel_atm_w_thrust(t, s, spec_thrust_sea_level, spec_thrust_vacuum, stage_nr, S_mid, m_fuel_total, t_burn, p);
    };

    auto y_accel = [=](double t, const std::vector<double> &s)
    {
        return y_accel_atm_w_thrust(t, s, spec_thrust_sea_level, spec_thrust_vacuum, stage_nr, S_mid, m_fuel_total, t_burn, p);
    };

    auto dmdt = [m_fuel_total, t_burn](double t, const std::vector<double> &s)
    {
        return -m_fuel_total / t_burn;
    };

    return {x_accel,
            y_accel,
            dxdt,
            dydt,
            dmdt};
}

/// @attention State vector length is 5.
/// @param spec_thrust_sea_level specific thrust at sea level
/// @param spec_thrust_vacuum    specific thrust in vacuum
/// @param m_fuel_total          Total fuel that will be burned by this stage
/// @param t_burn                Burn duration
/// @return ODE equation system for describing powered rocket flight in vacuum
math::equation_system rocket_no_atm_w_thrust(double spec_thrust_sea_level, double spec_thrust_vacuum, double m_fuel_total, double t_burn, pitch_control p)
{
    auto x_accel = [=](double t, const std::vector<double> &s)
    {
        return x_accel_no_atm_w_thrust(t, s, spec_thrust_sea_level, spec_thrust_vacuum, m_fuel_total, t_burn, p);
    };

    auto y_accel = [=](double t, const std::vector<double> &s)
    {
        return y_accel_no_atm_w_thrust(t, s, spec_thrust_sea_level, spec_thrust_vacuum, m_fuel_total, t_burn, p);
    };

    auto dmdt = [m_fuel_total, t_burn](double t, const std::vector<double> &s)
    {
        return -m_fuel_total / t_burn;
    };

    return {x_accel,
            y_accel,
            dxdt,
            dydt,
            dmdt};
}
/// @attention State vector length is 5.
/// @return ODE equation system for describing unpowered rocket flight in vacuum
math::equation_system rocket_no_atm_no_thrust()
{
    auto x_accel = [=](double t, const std::vector<double> &s)
    {
        return x_accel_no_atm_no_thrust(t, s);
    };

    auto y_accel = [=](double t, const std::vector<double> &s)
    {
        return y_accel_no_atm_no_thrust(t, s);
    };

    auto dmdt = [](double t, const std::vector<double> &s)
    {
        return 0.0;
    };

    return {x_accel,
            y_accel,
            dxdt,
            dydt,
            dmdt};
}

/// @attention State vector length is 7.
/// @param stage_nr stage number (for aerodynamics)
/// @param S_mid    Midsection area
/// @param L        Characteristic length
/// @return ODE equation system for describing unpowered rocket flight in atmosphere
math::equation_system rocket_atm_no_thrust(size_t stage_nr, double S_mid, double L, pitch_control p)
{
    auto x_accel = [=](double t, const std::vector<double> &s)
    {
        return x_accel_atm_no_thrust(t, s, stage_nr, S_mid, p);
    };

    auto y_accel = [=](double t, const std::vector<double> &s)
    {
        return y_accel_atm_no_thrust(t, s, stage_nr, S_mid, p);
    };

    auto dpitchdt = [](double t, const std::vector<double> &s)
    {
        return dthetadt(t, s);
    };

    auto domegazdt = [=](double t, const std::vector<double> &s)
    {
        return z_angular_accel_atm(t, s, stage_nr, S_mid, L, s[pitch]);
    };

    auto dmdt = [](double t, const std::vector<double> &s)
    {
        return 0.0;
    };

    return {x_accel,
            y_accel,
            dxdt,
            dydt,
            dmdt,
            dpitchdt,
            domegazdt};
}

std::vector<double> make_init_conditions(double starting_mass)
{
    return {0.0, 0.1, 0.0, 0.1, starting_mass};
}