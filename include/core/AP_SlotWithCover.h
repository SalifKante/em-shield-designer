#ifndef AP_SLOTWITHCOVER_H
#define AP_SLOTWITHCOVER_H

#include "AP_SlotAperture.h"
#include "PhysicsConstants.h"
#include <cmath>
#include <complex>
#include <stdexcept>
#include <iostream>

namespace EMCore {

// ============================================================================
// SLOT APERTURE WITH REMOVABLE METALLIC COVER
// ============================================================================
// Models the thin gap formed at the junction between a rectangular enclosure
// body and its removable metallic cover. The gap acts as a quarter-wave
// resonator that can severely degrade shielding effectiveness.
//
// Physical Configuration (Figure 3.9, Section 3.1.2.3):
//   - Main aperture: width l [m], height w [m]
//   - Cover gap:     thickness τ [m]
//   - The cover and front wall are coplanar → gap modelled as a coplanar
//     strip-line resonator
//
// ===========================================================================
// CASE A — Air-filled gap (Equation 3.22, 3.23):
// ===========================================================================
//
//   Z_ap = j·Z₀ₛ·Fₛ·tan[k₀·(l/2 + w/2 − 4τ)]                  (Eq. 3.22)
//
//   Fₛ = √[ 2τ(l + w + 2τ) / (l·w) ]                            (Eq. 3.23)
//
//   Branch admittance (MNA shunt element):
//   Y_ap = (2a/l) / Z_ap
//
//   Coupling factor note: The factor (2a/l) is the TE₁₀-mode coupling
//   coefficient between the enclosure waveguide and the coplanar resonator.
//   Eq. (3.22) provides the intrinsic resonator impedance Z_ap; the coupling
//   factor must be applied here — consistent with Eq. (3.13) for the open
//   slot (Reference [23], Ivanov et al. [1]).
//
// ===========================================================================
// CASE B — Dielectric-filled gap (Equation 3.24):
// ===========================================================================
//
//   Z_ap = j·Cs·[1/(c·√(C·C'))]·tan[k₀·√(C/C')·(l/2 + w/2 − 4τ)]
//                                                                  (Eq. 3.24)
//
//   where:
//     C  = per-unit-length capacitance of coplanar strip line WITH dielectric
//     C' = per-unit-length capacitance of coplanar strip line WITHOUT dielectric
//     c  = speed of light in free space [m/s]
//     Cs = surface capacitance = C·L (L = physical strip length) [F]
//
//   C and C' are computed from the coplanar strip-line model:
//     C' = ε₀ · K(k₀_cpw) / K'(k₀_cpw)          [F/m, air]
//     C  = ε₀·(1 + εᵣ)/2 · K(k₀_cpw) / K'(k₀_cpw) [F/m, dielectric]
//
//   Simplified ratio used in Eq. (3.24):
//     √(C/C') = √((1 + εᵣ)/2)
//
// ===========================================================================

class AP_SlotWithCover : public AP_SlotAperture {
public:

    // ========================================================================
    // CONSTRUCTORS
    // ========================================================================

    /**
     * @brief Air-filled gap constructor — implements Equations (3.22) and (3.23)
     *
     * @param node_from           Starting node index
     * @param node_to             Ending node index
     * @param branch_id           Unique branch identifier
     * @param enclosure_width_a   Enclosure broad dimension [m]
     * @param enclosure_height_b  Enclosure narrow dimension [m]
     * @param aperture_width_l    Aperture width [m]
     * @param aperture_height_w   Aperture height [m]
     * @param wall_thickness_t    Enclosure wall thickness [m]
     * @param slot_thickness_tau  Cover gap thickness τ [m]
     */
    AP_SlotWithCover(int node_from, int node_to, int branch_id,
                     double enclosure_width_a,
                     double enclosure_height_b,
                     double aperture_width_l,
                     double aperture_height_w,
                     double wall_thickness_t,
                     double slot_thickness_tau)
        : AP_SlotAperture(node_from, node_to, branch_id,
                          enclosure_width_a, enclosure_height_b,
                          aperture_width_l, aperture_height_w,
                          wall_thickness_t)
        , m_tau(slot_thickness_tau)
        , m_eps_r(1.0)             // Air-filled gap: εᵣ = 1
        , m_has_dielectric(false)
        , m_Fs(0.0)
    {
        validateGeometry();
        calculateCorrectionCoefficient();
    }

    /**
     * @brief Dielectric-filled gap constructor — implements Equation (3.24)
     *
     * @param node_from           Starting node index
     * @param node_to             Ending node index
     * @param branch_id           Unique branch identifier
     * @param enclosure_width_a   Enclosure broad dimension [m]
     * @param enclosure_height_b  Enclosure narrow dimension [m]
     * @param aperture_width_l    Aperture width [m]
     * @param aperture_height_w   Aperture height [m]
     * @param wall_thickness_t    Enclosure wall thickness [m]
     * @param slot_thickness_tau  Cover gap thickness τ [m]
     * @param relative_permittivity_eps_r  Relative permittivity of gap filler (εᵣ ≥ 1)
     */
    AP_SlotWithCover(int node_from, int node_to, int branch_id,
                     double enclosure_width_a,
                     double enclosure_height_b,
                     double aperture_width_l,
                     double aperture_height_w,
                     double wall_thickness_t,
                     double slot_thickness_tau,
                     double relative_permittivity_eps_r)
        : AP_SlotAperture(node_from, node_to, branch_id,
                          enclosure_width_a, enclosure_height_b,
                          aperture_width_l, aperture_height_w,
                          wall_thickness_t)
        , m_tau(slot_thickness_tau)
        , m_eps_r(relative_permittivity_eps_r)
        , m_has_dielectric(relative_permittivity_eps_r > 1.0 + 1e-9)
        , m_Fs(0.0)
    {
        if (m_eps_r < 1.0) {
            throw std::invalid_argument(
                "Relative permittivity eps_r must be >= 1.0");
        }
        validateGeometry();
        calculateCorrectionCoefficient();
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {

        // Dispatch to the appropriate model based on gap filling
        const Complex Z_ap = m_has_dielectric
                                 ? computeZap_Dielectric(f_Hz)   // Eq. (3.24)
                                 : computeZap_Air(f_Hz);          // Eq. (3.22)

        // Convert intrinsic resonator impedance to branch admittance.
        // Apply TE₁₀ coupling factor (2a/l): same convention as Eq. (3.13).
        Complex Y_ap(0.0, 0.0);
        if (std::abs(Z_ap) > 1e-30) {
            Y_ap = (2.0 * a_ / l_) / Z_ap;
        }
        // If Z_ap ≈ 0 (fully transparent slot): Y_ap → ∞, effectively a
        // short circuit. In MNA this collapses the node voltage to zero,
        // which is the correct physical limit. We leave Y_ap = 0 here;
        // the calling code should handle node collapse if needed.

        return {
            { Y_ap, -Y_ap },
            {-Y_ap,  Y_ap }
        };
    }

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {
        if (!AP_SlotAperture::isValid(error_msg)) return false;

        if (m_tau <= 0.0) {
            error_msg = "Cover gap (tau) must be positive";
            return false;
        }

        const double tan_arg = l_/2.0 + w_/2.0 - 4.0*m_tau;
        if (tan_arg <= 0.0) {
            error_msg = "Cover gap too large: (l/2 + w/2 - 4*tau) must be positive";
            return false;
        }

        if (std::isnan(m_Fs) || m_Fs <= 0.0) {
            error_msg = "Correction coefficient Fs is invalid";
            return false;
        }

        if (m_eps_r < 1.0) {
            error_msg = "Relative permittivity eps_r must be >= 1.0";
            return false;
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "Slot with Cover: l=%.1fmm, w=%.1fmm, tau=%.3fmm, "
                      "eps_r=%.2f, Fs=%.4f, Z0s=%.2f Ohm [%s]",
                      l_ * 1000.0, w_ * 1000.0, m_tau * 1000.0,
                      m_eps_r, m_Fs, Z0s_,
                      m_has_dielectric ? "Eq.3.24-dielectric" : "Eq.3.22-air");
        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    double getCoverGap()                const { return m_tau;            }
    double getCorrectionCoefficient()   const { return m_Fs;             }
    double getRelativePermittivity()    const { return m_eps_r;          }
    bool   hasDielectricFilling()       const { return m_has_dielectric; }

private:
    double m_tau;              // Cover gap thickness [m]
    double m_eps_r;            // Relative permittivity of gap filler (1.0 = air)
    bool   m_has_dielectric;   // True → use Eq. (3.24)
    double m_Fs;               // Correction coefficient Fₛ (Eq. 3.23)

    // ========================================================================
    // GEOMETRY VALIDATION
    // ========================================================================

    void validateGeometry() const {
        if (m_tau <= 0.0) {
            throw std::invalid_argument("Cover gap (tau) must be positive");
        }
        const double tan_arg = l_/2.0 + w_/2.0 - 4.0*m_tau;
        if (tan_arg <= 0.0) {
            throw std::invalid_argument(
                "Cover gap too large: resonator argument (l/2 + w/2 - 4*tau) "
                "must be positive for quarter-wave resonator behavior.");
        }
    }

    // ========================================================================
    // CORRECTION COEFFICIENT Fₛ — Equation (3.23)
    // ========================================================================
    //
    //   Fₛ = √[ 2τ(l + w + 2τ) / (l·w) ]
    //
    //   Limiting behaviour:
    //     τ → 0 : Fₛ → 0  (no cover → no gap resonance)
    //     τ large: Fₛ grows (stronger coupling through gap)

    void calculateCorrectionCoefficient() {
        const double numerator   = 2.0 * m_tau * (l_ + w_ + 2.0 * m_tau);
        const double denominator = l_ * w_;

        if (denominator <= 0.0) {
            throw std::invalid_argument("Fs: aperture dimensions l and w must be positive");
        }

        const double ratio = numerator / denominator;
        if (ratio < 0.0) {
            throw std::logic_error("Fs: numerator is negative — unexpected geometry");
        }

        m_Fs = std::sqrt(ratio);

        // Sanity clamp: Fₛ should be a moderate correction factor
        if (m_Fs > 100.0 || m_Fs < 1e-6) {
            std::cerr << "[AP_SlotWithCover] WARNING: Fs = " << m_Fs
                      << " is outside the expected range [1e-6, 100]. "
                      << "Check tau, l, w values.\n";
        }
    }

    // ========================================================================
    // TAN-SINGULARITY GUARD
    // ========================================================================
    // The quarter-wave resonator has genuine singularities at
    //   arg = π/2 + n·π  (n = 0, 1, 2, ...)
    // At these points Z_ap → ±j∞ → Y_ap → 0 (slot becomes opaque).
    // This is the physically correct anti-resonance behaviour; no numerical
    // correction is needed. We detect proximity and return early with Y = 0.
    //
    // Note: the arg = 0 singularity (tan = 0 → Z_ap = 0 → Y_ap → ∞)
    // corresponds to the resonance: the slot is fully transparent.
    // The Z_ap near-zero guard in computeYParameters handles this.

    static bool isNearTanSingularity(double arg) {
        // Check whether arg ≈ π/2 + n·π
        constexpr double PROXIMITY = 1e-6;   // radians
        const double     reduced   = std::fmod(std::abs(arg), M_PI);
        return std::abs(reduced - M_PI / 2.0) < PROXIMITY;
    }

    // ========================================================================
    // AIR-FILLED GAP — Equation (3.22)
    // ========================================================================
    //
    //   Z_ap = j·Z₀ₛ·Fₛ·tan[k₀·(l/2 + w/2 − 4τ)]
    //
    //   k₀ = 2πf/c   (free-space wavenumber)

    Complex computeZap_Air(double f_Hz) const {
        const double k0  = (2.0 * M_PI * f_Hz) / C_LIGHT;
        const double arg = k0 * (l_/2.0 + w_/2.0 - 4.0 * m_tau);

        // At tan singularity: Z_ap → ∞ → Y_ap → 0 (physically opaque slot)
        if (isNearTanSingularity(arg)) {
            return Complex(0.0, 1e30);   // Represent ∞ as very large impedance
        }

        const Complex j(0.0, 1.0);
        return j * Z0s_ * m_Fs * std::tan(arg);
    }

    // ========================================================================
    // DIELECTRIC-FILLED GAP — Equation (3.24)
    // ========================================================================
    //
    //   Z_ap = j·Cs·[1/(c·√(C·C'))]·tan[k₀·√(C/C')·(l/2 + w/2 − 4τ)]
    //
    //   Capacitance ratio derivation:
    //     C'/unit_length  = ε₀ · (K/K')_cpw           [air]
    //     C /unit_length  = ε₀·(1+εᵣ)/2 · (K/K')_cpw  [with dielectric]
    //
    //   Therefore:
    //     C/C' = (1 + εᵣ)/2
    //     √(C/C') = √((1+εᵣ)/2)                         [effective index]
    //
    //   Surface capacitance Cs:
    //     Cs = C · L,  where L = (l/2 + w/2 − 4τ) is the resonator length.
    //     In the impedance formula, Cs·(1/(c·√(C·C'))) simplifies to
    //     Z₀ₛ/√(εᵣ_eff) · Fₛ where εᵣ_eff = (1+εᵣ)/2 (half-filled geometry).
    //
    //   Practical implementation consistent with Eq. (3.24):
    //     n_eff  = √(C/C') = √((1+εᵣ)/2)
    //     Z_ap   = j · (Z₀ₛ / n_eff) · Fₛ · tan[k₀ · n_eff · resonator_length]

    Complex computeZap_Dielectric(double f_Hz) const {
        const double k0             = (2.0 * M_PI * f_Hz) / C_LIGHT;
        const double resonator_len  = l_/2.0 + w_/2.0 - 4.0 * m_tau;

        // Effective refractive index of half-filled coplanar strip gap
        const double n_eff          = std::sqrt((1.0 + m_eps_r) / 2.0);

        // Loaded electrical length
        const double arg            = k0 * n_eff * resonator_len;

        // At tan singularity: Z_ap → ∞ → Y_ap → 0 (physically opaque)
        if (isNearTanSingularity(arg)) {
            return Complex(0.0, 1e30);
        }

        // Impedance of dielectric-loaded resonator: Z₀ₛ is divided by n_eff
        // because loading lowers the characteristic impedance of the strip line
        const Complex j(0.0, 1.0);
        return j * (Z0s_ / n_eff) * m_Fs * std::tan(arg);
    }
};

} // namespace EMCore

#endif // AP_SLOTWITHCOVER_H
