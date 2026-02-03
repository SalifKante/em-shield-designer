#ifndef PHYSICSCONSTANTS_H
#define PHYSICSCONSTANTS_H

#include<cmath>

// ============================================================================
// Ensure math constants are available
// ============================================================================
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


namespace EMCore {
    // ============================================================================
    // Physical Constants (SI units)
    // ============================================================================

    constexpr double C_LIGHT = 299792458.0;        // [m/s] Speed of light in vacuum
    constexpr double MU_0 = 4.0 * M_PI * 1e-7;     // [H/m] Permeability of free space
    constexpr double EPS_0 = 1.0 / (MU_0 * C_LIGHT * C_LIGHT); // [F/m] Permittivity
    constexpr double Z_0 = 376.730313668;          // [Ohm] Free-space impedance

    // ============================================================================
    // Conversion Utilities
    // ============================================================================

    inline double dB_to_linear(double dB) {
        return std::pow(10.0, dB / 20.0);
    }

    inline double linear_to_dB(double linear) {
        return 20.0 * std::log10(linear);
    }

    inline double frequency_to_wavenumber(double f_Hz) {
        return 2.0 * M_PI * f_Hz / C_LIGHT;  // k0 = ω/c [rad/m]
    }


}
#endif // PHYSICSCONSTANTS_H
