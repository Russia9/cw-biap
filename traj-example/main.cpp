#include <string>
#include <fstream>
#include <iostream>
#include "libs/math/solver.hpp"
#include "libs/math/derivative.hpp"
#include "model.hpp"

struct stage_info
{
    double spec_thrust_sea_level;
    double spec_thrust_vacuum;
    double burn_time;
    double mass_fuel;
    double mass_stage;
};

double time_elapsed(double t, const std::vector<double> &s, double target_time)
{
    return target_time - t;
}

double target_altitude_reached(double t, const std::vector<double> &s, double target_altitude)
{
    return target_altitude - altitude(s);
}

/// @brief Find nearest "pretty" time t_grid from "uneven" time and find state corresponding to t_grid
/// @note For example, if t = 13.334567 s and step_size = 0.1 s, nearest t_grid would be 13.4 s.
/// @param solver_output pair of double and vector<double> which gets changed to match t_grid
/// @param step_size what step size was used to find solver_output
/// @param eq_system equation system to use for finding state vector matching t_grid
void get_back_to_grid(std::pair<double, std::vector<double>> &solver_output, double step_size, math::equation_system eq_system)
{
    // we are doing a step with size (t_grid - t), which is guaranteed to be less than step_size
    // (internally RK4_solve always does step = min(t_max - t, step_size))
    // thus, we're passing bogus termination condition, because we want solver to terminate due to time running out
    auto this_never_triggers = [](double t, const std::vector<double> &s)
    {
        return -1.0;
    };
    double t_grid = ceil(solver_output.first / step_size) * step_size;
    if (fabs(solver_output.first - t_grid) < step_size)
        solver_output = math::RK4_solve(eq_system, solver_output.first, solver_output.second, step_size, t_grid, this_never_triggers);
    else
        throw std::runtime_error("Couldn't find t_grid");
}

void print_atm(std::ofstream &Tfile,
               double t, const std::vector<double> &s,
               bool is_in_atm, bool has_thrust, size_t stage_nr,
               double S_mid, double L, pitch_control p)
{
    double pitch_angle;
    double Mz;
    double omegaz_for_print;
    if (has_thrust)
    {
        pitch_angle = pitch_programmed(t, p);
        Mz = NAN;
        omegaz_for_print = NAN;
    }
    else
    {
        pitch_angle = s[pitch];
        Mz = M_z_aero(stage_nr, S_mid, L, s, pitch_angle);
        omegaz_for_print = s[omega_z] / M_PI * 180.0;
    }

    Tfile << /* time */ t << ","
          << /* x */ s[x] / 1000 << ","
          << /* y */ s[y] / 1000 << ","
          << /* v_x */ s[vx] << ","
          << /* v_y */ s[vy] << ","
          << /* v */ velocity_mag(s) << ","
          << /* pitch */ pitch_angle / M_PI * 180.0 << ","
          << /* THETA */ flight_angle(s) / M_PI * 180.0 << ","
          << /* aoa */ aoa(pitch_angle, flight_angle(s)) / M_PI * 180.0 << ","
          << /* mass */ s[m] << ","
          << /* Mach */ velocity_mag(s) / Lueft::sonicspeed(altitude(s)) << ","
          << /* Altitude */ altitude(s)/1000 << ","
          << /* Distance */ distance(s) << ","
          << /* Angl. Dist. */ angular_distance(s) / M_PI * 180.0 << ","
          << /* X aero */ X_aero(stage_nr, S_mid, s, pitch_angle) << ","
          << /* Y aero */ Y_aero(stage_nr, S_mid, s, pitch_angle) << ","
          << /* M_z aero */ Mz << ","
          << /* omega_z */ omegaz_for_print << ","
          << /* stage N */ stage_nr << "\n";
}

void print_no_atm(std::ofstream &Tfile,
                  double t, const std::vector<double> &s,
                  bool is_in_atm, bool has_thrust, size_t stage_nr, pitch_control p)
{
    double pitch_angle;
    // double omegaz_for_print;
    if (has_thrust)
    {
        pitch_angle = pitch_programmed(t, p);
        // omegaz_for_print = NAN;
    }
    else
    {
        pitch_angle = s[pitch];
        // omegaz_for_print = s[omega_z] / M_PI * 180.0;
    }

    Tfile << /* time */ t << ","
          << /* x */ s[x] / 1000 << ","
          << /* y */ s[y] / 1000 << ","
          << /* v_x */ s[vx] << ","
          << /* v_y */ s[vy] << ","
          << /* v */ velocity_mag(s) << ","
          << /* pitch */ pitch_angle / M_PI * 180.0 << ","
          << /* THETA */ flight_angle(s) / M_PI * 180.0 << ","
          << /* aoa */ "nan" << ","
          << /* mass */ s[m] << ","
          << /* Mach */ "nan" << ","
          << /* Altitude */ altitude(s)/1000 << ","
          << /* Distance */ distance(s) << ","
          << /* Angl. Dist. */ angular_distance(s) / M_PI * 180.0 << ","
          << /* X aero */ "nan" << ","
          << /* Y aero */ "nan" << ","
          << /* M_z aero */ "nan" << ","
          << /* omega_z */ "nan" << ","
          << /* stage N */ stage_nr << "\n";
}

void print_to_file(std::ofstream &Tfile,
                   double t, const std::vector<double> &s,
                   bool is_in_atm, bool has_thrust, size_t stage_nr,
                   double S_mid, double L, pitch_control p)
{
    if (is_in_atm)
        print_atm(Tfile, t, s, is_in_atm, has_thrust, stage_nr, S_mid, L, p);
    else
        print_no_atm(Tfile, t, s, is_in_atm, has_thrust, stage_nr, p);
}

void print_to_two_files(std::ofstream &Tfile, std::ofstream &TfileTrimmed,
                        int print_ratio, int &main_print_count,
                        double t, const std::vector<double> &s,
                        bool is_in_atm, bool has_thrust, size_t stage_nr,
                        double S_mid, double L, pitch_control p)
{
    print_to_file(Tfile, t, s, is_in_atm, has_thrust, stage_nr, S_mid, L, p);
    if (main_print_count % print_ratio == 0)
        print_to_file(TfileTrimmed, t, s, is_in_atm, has_thrust, stage_nr, S_mid, L, p);
    main_print_count++;
}

double current_max_aoa_before_transsonic_deg = 0.0;
double current_max_aoa_after_transsonic_before_atm_exit_deg = 0.0;
double current_max_pitch_speed = 0.0;
double current_pitch_speed_near_tk1 = 0.0;
double current_max_q = 0.0;

void check_pitch_profile_conditions(double t, const std::vector<double> &s,
                                    pitch_control p)
{
    if ((altitude(s) < 94000.0) && (t < p.t_k2))
    {

        double aoo_alpha_deg = fabs(aoa(pitch_programmed(t, p), flight_angle(s)) / M_PI * 180.0);
        double Mach = velocity_mag(s) / Lueft::sonicspeed(altitude(s));
        double q = Lueft::density(altitude(s)) * velocity_mag(s) * velocity_mag(s) * 0.5;
        if (Mach <= 1.1)
        {
            current_max_aoa_before_transsonic_deg = std::max(current_max_aoa_before_transsonic_deg, aoo_alpha_deg);
        }
        else
            current_max_aoa_after_transsonic_before_atm_exit_deg = std::max(current_max_aoa_after_transsonic_before_atm_exit_deg, aoo_alpha_deg);

        if ((t >= (p.t_k1 - 5.0)) && (t <= (p.t_k1 + 5.0)))
        {
            auto vectorwrapped_pitch_program = [p](std::vector<double> singular_value_vec)
            { return pitch_programmed(singular_value_vec[0], p); };
            current_pitch_speed_near_tk1 = std::max(fabs(math::derivative_vec(vectorwrapped_pitch_program, {t}, 0)), current_pitch_speed_near_tk1);
        }
        current_max_q = std::max(current_max_q, q);
    }

    if (t <= p.t_k2)
    {
        auto vectorwrapped_pitch_program = [p](std::vector<double> singular_value_vec)
        { return pitch_programmed(singular_value_vec[0], p); };
        current_max_pitch_speed = std::max(fabs(math::derivative_vec(vectorwrapped_pitch_program, {t}, 0)), current_max_pitch_speed);
    }
}

int run_simulation(stage_info stage_1, stage_info stage_2, stage_info stage_3, pitch_control p, double S_mid, double L, int print_ratio, double step_size)
{

    math::equation_system stage_1_atm = rocket_atm_w_thrust(stage_1.spec_thrust_sea_level,
                                                            stage_1.spec_thrust_vacuum,
                                                            1, S_mid,
                                                            stage_1.mass_fuel,
                                                            stage_1.burn_time, p);
    math::equation_system stage_2_atm = rocket_atm_w_thrust(stage_2.spec_thrust_sea_level,
                                                            stage_2.spec_thrust_vacuum,
                                                            2, S_mid,
                                                            stage_2.mass_fuel,
                                                            stage_2.burn_time, p);
    math::equation_system stage_2_vac = rocket_no_atm_w_thrust(stage_2.spec_thrust_sea_level,
                                                               stage_2.spec_thrust_vacuum,
                                                               stage_2.mass_fuel,
                                                               stage_2.burn_time, p);
    math::equation_system stage_3_vac = rocket_no_atm_no_thrust();

    math::equation_system stage_3_atm = rocket_atm_no_thrust(3, S_mid, L, p);

    auto hit_ground = [](double t, const std::vector<double> &s)
    {
        return target_altitude_reached(t, s, 0.0);
    };

    auto atmosphere_border_reached = [](double t, const std::vector<double> &s)
    {
        return target_altitude_reached(t, s, 94000.0);
    };

    auto stage_1_separation = [stage_1](double t, const std::vector<double> &s)
    {
        return time_elapsed(t, s, stage_1.burn_time);
    };

    auto stage_2_separation = [stage_1, stage_2](double t, const std::vector<double> &s)
    {
        return time_elapsed(t, s, stage_2.burn_time + stage_1.burn_time);
    };

    std::ofstream Tfile("trajectory.csv");
    std::ofstream TfileTrimmed("trajectory_trimmed.csv");

    int print_count = 0;
    bool is_in_atm = true;
    bool has_thrust = true;
    size_t stage_nr = 1;
    auto obs = [&Tfile, &TfileTrimmed, print_ratio, &print_count, &is_in_atm, &has_thrust, S_mid, L, &stage_nr, p](double t, const std::vector<double> &s)
    {
        print_to_two_files(Tfile, TfileTrimmed, print_ratio, print_count, t, s, is_in_atm, has_thrust, stage_nr, S_mid, L, p);
        check_pitch_profile_conditions(t, s, p);
    };

    double t_grid;
    std::string header = "t, x, y, V_x, V_y, V, pitch, THETA, aoa, mass, Mach, H, L, angl.dist., X aero, Y aero, M_z aero, oemga_z, stage N\n";
    Tfile << header;
    TfileTrimmed << header;

    // STAGE 1
    std::vector<double> init_conds = make_init_conditions(stage_1.mass_stage);
    auto end_state_stage_1_atm = math::RK4_solve(stage_1_atm, 0.0, init_conds, step_size, 3000, stage_1_separation, obs);

    // STAGE 1 SEPARATION
    end_state_stage_1_atm.second[m] = stage_2.mass_stage;
    std::cerr << end_state_stage_1_atm.first << "\n";
    // STAGE 2
    stage_nr = 2;
    get_back_to_grid(end_state_stage_1_atm, step_size, stage_2_atm);

    // continue integrating till atmosphere exit
    auto end_state_stage_2_atm = math::RK4_solve(stage_2_atm, end_state_stage_1_atm.first, end_state_stage_1_atm.second, step_size, 3000, atmosphere_border_reached, obs);
    is_in_atm = false;

    get_back_to_grid(end_state_stage_2_atm, step_size, stage_2_vac);
    std::cerr << end_state_stage_2_atm.first << "\n";

    // continue till stage 2 burn ends
    auto end_state_stage_2_vac = math::RK4_solve(stage_2_vac, end_state_stage_2_atm.first, end_state_stage_2_atm.second, step_size, 3000, stage_2_separation, obs);
    // STAGE 3
    stage_nr = 3;
    end_state_stage_2_vac.second[m] = stage_3.mass_stage;
    has_thrust = false;
    std::cerr << end_state_stage_2_vac.first << "\n";
    get_back_to_grid(end_state_stage_2_vac, step_size, stage_3_vac);

    // continue with stage 3 till atmosphere re-entry
    auto end_state_stage_3_vac = math::RK4_solve(stage_3_vac, end_state_stage_2_vac.first, end_state_stage_2_vac.second, step_size, 3000, atmosphere_border_reached, obs);
    is_in_atm = true;
    auto start_state_stage_3_atm = end_state_stage_3_vac;
    start_state_stage_3_atm.second.push_back(flight_angle(end_state_stage_3_vac.second));
    start_state_stage_3_atm.second.push_back(0.0);
    std::cerr << start_state_stage_3_atm.first << "\n";
    get_back_to_grid(start_state_stage_3_atm, step_size, stage_3_atm);

    // continue till ground hit
    auto end_state_stage_3_atm = math::RK4_solve(stage_3_atm, start_state_stage_3_atm.first, start_state_stage_3_atm.second, step_size, 3000, hit_ground, obs);
    std::cerr << end_state_stage_3_atm.first << "\n";
    Tfile.close();
    TfileTrimmed.close();

    std::cout << "\nSimulation ended, end time " << end_state_stage_3_atm.first << " s, total distance covered " << distance(end_state_stage_3_atm.second) / 1e3 << " km\n";
    std::cout << "Max aoa before transsonic (M <= 1.1) : " << current_max_aoa_before_transsonic_deg << " deg\n"
              << "Max aoa before atmosphere exit : " << current_max_aoa_after_transsonic_before_atm_exit_deg << " deg\n"
              << "Max pitch speed near stage 1 separation : " << current_pitch_speed_near_tk1 << " deg\n"
              << "Overall max pitch speed : " << current_max_pitch_speed << " deg\n"
              << "Max observed q : " << current_max_q << " Pa\n";

    if (fabs(altitude(end_state_stage_3_atm.second)) < 1e-6)
        return 0;
    else
    {
        std::cout << "Warning: trajectory does not reach the ground, stopped at an altitude " << altitude(end_state_stage_3_atm.second) << " m\n";
        return 1;
    }
}

int main()
{
    stage_info stage_1;
    stage_1.burn_time = 51.6;
    stage_1.mass_stage = 12981.7;
    stage_1.mass_fuel = 9016.3;
    stage_1.spec_thrust_sea_level = 231.07;
    stage_1.spec_thrust_vacuum = 263.40;

    stage_info stage_2;
    stage_2.burn_time = 50.5;
    stage_2.mass_stage = 3170.9;
    stage_2.mass_fuel = 2202.4;
    stage_2.spec_thrust_sea_level = 175.07;
    stage_2.spec_thrust_vacuum = 272.70;

    stage_info stage_payload;
    stage_payload.burn_time = 0.0;
    stage_payload.mass_stage = 500.0;
    stage_payload.mass_fuel = 0.0;
    stage_payload.spec_thrust_sea_level = 0.0;
    stage_payload.spec_thrust_vacuum = 0.0;

    pitch_control pitch_program;
    pitch_program.t_k1 = 51.6;
    pitch_program.t_k2 = pitch_program.t_k1 + 50.5;
    pitch_program.t_vertical = 10;
    pitch_program.theta_k1 = 68.0 / 180.0 * M_PI;
    pitch_program.theta_k2 = 54.0 / 180.0 * M_PI;

    // pi * d^2 / 4
    double S_mid = 1.37 * 1.37 * M_PI_4;
    double L = 1.2;
    return run_simulation(stage_1, stage_2, stage_payload, pitch_program, S_mid, L, 1002, 0.1);
}