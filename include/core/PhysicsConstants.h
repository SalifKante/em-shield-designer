#ifndef PHYSICSCONSTANTS_H
#define PHYSICSCONSTANTS_H

#include <cmath>

// ============================================================================
// M_PI — portable definition
// ============================================================================
// IEEE 754 double precision: π to 20 significant figures.
// The #ifndef guard prevents redefinition on platforms that already define
// M_PI in <cmath> (e.g. MSVC with _USE_MATH_DEFINES, GCC with -D_USE_MATH_DEFINES).
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace EMCore {

// ============================================================================
// FUNDAMENTAL PHYSICAL CONSTANTS (SI units, exact definitions)
// ============================================================================
//
// All constants follow the pre-2019 SI system, which is the system used
// implicitly in the Russian theory (Sections 3.1.2.1 – 3.1.2.4) and in the
// cited IEEE/MDPI references.
//
// Constant hierarchy (do not reorder — EPS_0 and Z_0 are derived):
//
//   c     — exact by definition since 1983 (BIPM)
//   μ₀    — exact in pre-2019 SI (4π × 10⁻⁷ H/m)
//   ε₀    — derived: 1/(μ₀ · c²)                [F/m]
//   Z_0   — derived: μ₀ · c = √(μ₀/ε₀)          [Ω]  (exact physical value)
//   Z_0_SYMBOLIC — 120π Ω                        [Ω]  (theory symbolic value,
//                                                       Eq. 3.12)
//
// ============================================================================
// NOTE ON c AND Z₀ VALUES
// ============================================================================
//
// The exact SI speed of light is c = 299,792,458 m/s.
// Using the rounded value c = 3×10⁸ m/s introduces a +0.069% systematic
// error in every wavelength, wavenumber, and cutoff frequency computation:
//
//   Δf_c  = Δc / (2a) ≈ 346 kHz  for a = 0.3 m
//   Δk₀   = k₀ · (Δc/c) ≈ 0.069% of every computed wavenumber
//
// Over a 40 GHz sweep (Section 3.1.3 test range) this accumulates to a
// non-negligible frequency offset at higher resonances.
//
// The exact c also gives:
//   Z_exact = μ₀ · c = 4π×10⁻⁷ × 299,792,458 ≈ 376.730 Ω
//
// The Russian theory (Eq. 3.12) writes Z₀ = 120π Ω ≈ 376.991 Ω as a
// symbolic approximation. Both values are provided here:
//   Z_0          — exact physical value (376.730 Ω), use for all computation
//   Z_0_SYMBOLIC — 120π Ω, retained for direct cross-reference with Eq. (3.12)
//
// MATLAB's physconst('LightSpeed') returns 299792458.0 exactly.
// ============================================================================

/// Speed of light in vacuum [m/s] — exact SI definition (BIPM 1983)
constexpr double C_LIGHT = 299792458.0;

/// Permeability of free space [H/m] — exact in pre-2019 SI
constexpr double MU_0 = 4.0 * M_PI * 1e-7;

/// Permittivity of free space [F/m] — derived: ε₀ = 1/(μ₀·c²)
constexpr double EPS_0 = 1.0 / (MU_0 * C_LIGHT * C_LIGHT);

/// Wave impedance of free space [Ω] — exact physical value: Z₀ = μ₀·c ≈ 376.730 Ω
/// Use this constant in all branch admittance computations.
constexpr double Z_0 = MU_0 * C_LIGHT;

/// Symbolic wave impedance [Ω] — Z₀ = 120π ≈ 376.991 Ω
/// This is the value written in Eq. (3.12) of the Russian theory as a
/// symbolic approximation. Provided for documentation cross-reference only.
/// Do NOT use in numerical computations — use Z_0 above.
constexpr double Z_0_SYMBOLIC = 120.0 * M_PI;

// ============================================================================
// DERIVED CONVENIENCE CONSTANTS
// ============================================================================

/// Angular frequency coefficient: used as ω = TWO_PI * f_Hz
constexpr double TWO_PI = 2.0 * M_PI;

// ============================================================================
// CONVERSION UTILITIES
// ============================================================================
//
// All dB conversions use the factor-20 convention appropriate for field
// quantities (voltage, E-field, H-field) and for SE as defined in Eq. (3.8):
//   SE [dB] = −20·log₁₀(|2·U_obs / V₀|)
//
// Do NOT use factor-10 (power dB) for SE computations.

/// Convert field quantity from dB to linear scale
/// Example: −40 dB → 0.01
inline double dB_to_linear(double dB) {
    return std::pow(10.0, dB / 20.0);
}

/// Convert field quantity from linear to dB scale
/// Example: 0.01 → −40 dB
/// Precondition: linear > 0 (caller must guard against zero/negative input)
inline double linear_to_dB(double linear) {
    return 20.0 * std::log10(linear);
}

/// Free-space wavenumber k₀ = 2πf/c = ω/c  [rad/m]
/// Consistent with k₀ = 2π/λ used in Eqs. (3.16) and (3.19).
inline double frequency_to_wavenumber(double f_Hz) {
    return TWO_PI * f_Hz / C_LIGHT;
}

/// Free-space wavelength λ = c/f  [m]
inline double frequency_to_wavelength(double f_Hz) {
    return C_LIGHT / f_Hz;
}

} // namespace EMCore

#endif // PHYSICSCONSTANTS_H
