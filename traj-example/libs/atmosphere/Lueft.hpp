#pragma once
#include <cmath>
#include <stdexcept>

// This is a digitised version of GOST 4401-81, implemented using a struct.
// Provides a way to calculate atmospheric parameters in a convenient and easy to use way.
// It will always update all atmospheric parameters after altitude change.

namespace Lueft
{

    double betas[10] = {-0.0065, -0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.0020, -0.0020, 0.0};
    double Pgrid[10] = {1.27783E+5, 1.01325E+5, 2.26320E+4, 5.47487E+3, 8.68014E+2, 1.10906E+2, 6.69384E+1, 3.95639E0, 0.36341194, 0.07025889};
    double Tgrid[10] = {301.15, 288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.65, 186.525};
    double Hgrid[10] = {-2000, 0, 11000, 20000, 32000, 47000, 51000, 71000, 85000, 94000};

    double geopotAlt(double altitude)
    {
        double ans = (6356767 * altitude) / (6356767 + altitude);
        if (ans - 94000 > 1E-6 || ans + 2000 < 1E-6)
            throw std::runtime_error("Geopotential altitude out of bounds (must be in [-2000, 94000])");
        return ans;
    }

    double grav_accel(double altitude)
    {
        return 9.80665 * (6356767 / (6356767 + altitude)) * (6356767 / (6356767 + altitude));
    }

    double temperature(double altitude)
    { // uses geopotential altitude, defined in [-2000, 94000]
        double geopotential_altitude = geopotAlt(altitude);
        double temperature_K;
        if (fabs(geopotential_altitude - 94000) < 1E-6)
        { // edge case
            temperature_K = 186.525;
        }
        int i{0};
        bool exit = false;
        while (!exit)
        {
            if (geopotential_altitude >= Hgrid[i] && geopotential_altitude < Hgrid[i + 1])
            {
                exit = true;
            }
            if (!exit)
            {
                i++;
            }
        }
        temperature_K = Tgrid[i] + betas[i] * (geopotential_altitude - Hgrid[i]);
        return temperature_K;
    }

    double sonicspeed(double altitude)
    {
        double temperature_K = temperature(altitude);
        return 20.046796 * sqrt(temperature_K);
    }

    double pressure(double altitude)
    { // uses geopot alt and temperature
        double geopotential_altitude = geopotAlt(altitude);
        double temperature_K = temperature(altitude);
        int i{0};
        bool exit = false;
        while (!exit)
        {
            if (geopotential_altitude >= Hgrid[i] && geopotential_altitude < Hgrid[i + 1])
            {
                exit = true;
            }
            if (!exit)
            {
                i++;
            }
        }
        if (fabs(betas[i]) < 1E-6)
        { // betas[i] = 0
            return pow(10.0, log10(Pgrid[i]) - ((0.434294 * 9.80665) / (287.05287 * temperature_K)) * (geopotential_altitude - Hgrid[i]));
        }
        else
        { // betas[i] != 0
            return pow(10.0, log10(Pgrid[i]) - (9.80665 / (betas[i] * 287.05287)) * log10(temperature_K / Tgrid[i]));
        }
    }

    double density(double altitude)
    {
        double p = pressure(altitude);
        double T = temperature(altitude);
        return p / (287.05287 * T);
    }

    struct atmospheric_parameters
    {
        double temperature;
        double pressure;
        double density;
        double grav_accel;
        double sonic_speed;
    };

    atmospheric_parameters atm(double altitude_meters)
    {
        atmospheric_parameters ans;
        ans.density = density(altitude_meters);
        ans.pressure = pressure(altitude_meters);
        ans.grav_accel = grav_accel(altitude_meters);
        ans.sonic_speed = sonicspeed(altitude_meters);
        ans.temperature = temperature(altitude_meters);
        return ans;
    }
}
