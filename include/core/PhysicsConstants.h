#ifndef PHYSICSCONSTANTS_H
#define PHYSICSCONSTANTS_H
#include<cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace EMCore {
// ============================================================================
// Physical Constants (SI units)
// ============================================================================

// IMPORTANT: Using c = 3e8 m/s to match MATLAB reference implementation
// (Precise value is 299,792,458 m/s, but MATLAB uses rounded value)
constexpr double C_LIGHT = 3.0e8;              // [m/s] MATLAB-compatible value

constexpr double MU_0 = 4.0 * M_PI * 1e-7;     // [H/m] Permeability of free space
constexpr double EPS_0 = 1.0 / (MU_0 * C_LIGHT * C_LIGHT); // [F/m] Permittivity
// constexpr double Z_0 = 376.730313668;          // [Ohm] Free-space impedance
constexpr double Z_0 = MU_0 * C_LIGHT;  // = 120π ≈ 376.991 Ω (MATLAB-compatible)

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
