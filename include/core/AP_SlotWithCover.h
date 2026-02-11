#ifndef AP_SLOTWITHCOVER_H
#define AP_SLOTWITHCOVER_H

#include "AP_SlotAperture.h"  // Inherits from AP_SlotAperture
#include <cmath>
#include <stdexcept>

namespace EMCore {

// ============================================================================
// SLOT APERTURE WITH REMOVABLE COVER
// ============================================================================
// Models a slot aperture at the junction between enclosure wall and removable
// metal cover, forming a quarter-wave resonator.
//
// Physical Configuration (Figure 3.9, Image 6, p.103):
//   - Main aperture: width l, height w
//   - Cover slot: thickness τ (gap between cover and enclosure)
//   - Forms quarter-wave resonator affecting SE
//
// Theory: Russian documentation Section 3.1.2.3
//
// Key Equations:
//   Z_ap = j·Z₀ₛ·Fₛ·tan[k₀(l/2 + w/2 - 4τ)]     (Equation 3.22)
//   Fₛ = √[2τ(l + w + 2τ) / (l·w)]                 (Equation 3.23)
//
// where:
//   τ   = slot thickness (gap between cover and enclosure wall) [m]
//   Z₀ₛ = characteristic impedance of coplanar strip line [Ω]
//   Fₛ  = correction coefficient (dimensionless)
//   l   = aperture width [m]
//   w   = aperture height [m]
//
// The aperture admittance including geometric factor is:
//   Y_ap = (2a/l) · (1/Z_ap)
//
// Note: Equation 3.22 assumes air-filled gap. For dielectric-filled gaps,
//       surface capacitance must be calculated separately (see text after 3.23).
// ============================================================================

class AP_SlotWithCover : public AP_SlotAperture {
public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * @brief Create a slot aperture with removable cover
     * @param node_from Starting node
     * @param node_to Ending node
     * @param branch_id Unique branch identifier
     * @param enclosure_width_a Enclosure width (broad dimension) [m]
     * @param enclosure_height_b Enclosure height (narrow dimension) [m]
     * @param aperture_width_l Aperture width [m]
     * @param aperture_height_w Aperture height (slot opening) [m]
     * @param wall_thickness_t Enclosure wall thickness [m]
     * @param slot_thickness_tau Cover gap thickness [m]
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
    {
        // Validate cover gap
        if (m_tau <= 0.0) {
            throw std::invalid_argument("Cover gap (tau) must be positive");
        }

        // Validate physical constraint: tan argument must be meaningful
        // (l/2 + w/2 - 4τ) should typically be positive for resonator behavior
        double tan_length = aperture_width_l/2.0 + aperture_height_w/2.0 - 4.0*m_tau;
        if (tan_length <= 0.0) {
            throw std::invalid_argument(
                "Cover gap too large: (l/2 + w/2 - 4*tau) must be positive");
        }

        // Calculate correction coefficient Fₛ (Equation 3.23)
        calculateCorrectionCoefficient();
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION (Equations 3.22, 3.23)
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        using namespace std::complex_literals;

        // Get slot-line impedance from parent (Cohn's formula)
        double Z0s = Z0s_;  // Protected member from AP_SlotAperture

        // Free-space wavenumber
        double k0 = (2.0 * M_PI * f_Hz) / C_LIGHT;

        // ----------------------------------------------------------------
        // Equation 3.22: Z_ap = j·Z₀ₛ·Fₛ·tan[k₀(l/2 + w/2 - 4τ)]
        // ----------------------------------------------------------------
        // [FIX #1 - CRITICAL] Argument is l/2 + w/2 - 4τ (PLUS, not minus)
        double arg = k0 * (l_/2.0 + w_/2.0 - 4.0*m_tau);

        // Handle numerical issues near tan singularities
        if (std::abs(arg) > 100.0) {
            arg = std::copysign(100.0, arg);
        }

        Complex Z_ap = 1.0i * Z0s * m_Fs * std::tan(arg);

        // Avoid division by zero when converting to admittance
        if (std::abs(Z_ap) < 1e-12) {
            Z_ap = Complex(1e-12, 0.0);
        }

        // Convert to admittance and apply geometric factor (2a/l)
        Complex Y_ap = (2.0 * a_ / l_) / Z_ap;

        // Shunt admittance Y-matrix
        return {
            { Y_ap,  -Y_ap },
            {-Y_ap,   Y_ap }
        };
    }

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {
        // Check parent validity first
        if (!AP_SlotAperture::isValid(error_msg)) {
            return false;
        }

        if (m_tau <= 0.0) {
            error_msg = "Cover gap (tau) must be positive";
            return false;
        }

        // Physical constraint check
        double tan_length = l_/2.0 + w_/2.0 - 4.0*m_tau;
        if (tan_length <= 0.0) {
            error_msg = "Cover gap too large: (l/2 + w/2 - 4*tau) must be positive";
            return false;
        }

        if (std::isnan(m_Fs) || m_Fs <= 0.0) {
            error_msg = "Correction coefficient Fs calculation failed";
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
                      "Slot with Cover: l=%.1fmm, w=%.1fmm, tau=%.2fmm, Fs=%.3f, Z0s=%.1f Ohm",
                      l_ * 1000.0,
                      w_ * 1000.0,
                      m_tau * 1000.0,
                      m_Fs,
                      Z0s_);
        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    double getCoverGap() const { return m_tau; }
    double getCorrectionCoefficient() const { return m_Fs; }

private:
    double m_tau;  // Cover gap thickness [m]
    double m_Fs;   // Correction coefficient Fₛ (Eq 3.23)

    // ========================================================================
    // CORRECTION COEFFICIENT (Equation 3.23)
    // ========================================================================
    //
    // Fₛ = √[2τ(l + w + 2τ) / (l·w)]
    //
    // [FIX #2 - CRITICAL] Two corrections from original:
    //   - Numerator: (l + w + 2τ) not (l - w + 2τ)  [plus, not minus]
    //   - Denominator: l·w not h·w  [aperture width, not enclosure height]
    //
    // Boundary behavior:
    //   τ → 0: Fₛ → 0 (no cover effect)
    //   τ large: Fₛ grows (stronger resonator coupling)

    void calculateCorrectionCoefficient() {
        // Equation 3.23
        double numerator = 2.0 * m_tau * (l_ + w_ + 2.0 * m_tau);
        double denominator = l_ * w_;

        if (denominator <= 0.0) {
            m_Fs = 1.0;  // Fallback for invalid geometry
            return;
        }

        double ratio = numerator / denominator;

        if (ratio < 0.0) {
            m_Fs = 1.0;  // Fallback for invalid geometry
            return;
        }

        m_Fs = std::sqrt(ratio);

        // Sanity clamp (Fₛ should be a reasonable correction factor)
        m_Fs = std::max(0.01, std::min(100.0, m_Fs));
    }
};

} // namespace EMCore

#endif // AP_SLOTWITHCOVER_H
