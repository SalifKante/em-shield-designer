#ifndef AP_SLOTWITHCOVER_H
#define AP_SlotWithCover_H

#include "AP_SlotAperture.h"  // ← MUST INHERIT FROM THIS
#include <cmath>
#include <stdexcept>

namespace EMCore {

// ============================================================================
// SLOT APERTURE WITH REMOVABLE COVER
// ============================================================================
// Models a slot aperture at the junction between enclosure wall and removable
// metal cover, forming a quarter-wave resonator.
//
// Physical Configuration (Figure 3.9):
//   - Main aperture: width l, height w
//   - Cover slot: thickness τ (gap between cover and enclosure)
//   - Forms quarter-wave resonator affecting SE
//
// Theory: Russian documentation Section 3.1.2.3
//
// Key Equations:
//   Z_ap = j·Z₀ₛ·Fᵤ·tan[k₀(l/2 - w/2 - 4τ)]  (Equation 3.22)
//   Fᵤ = √[2τ(l - w + 2τ) / (hw)]           (Equation 3.23)
//
// where:
//   τ = slot thickness (gap between cover and enclosure wall)
//   Z₀ₛ = characteristic impedance of coplanar strip line
//   Fᵤ = correction coefficient
//   h = enclosure height (or waveguide height) - from denominator h*w
//
// Note: Equation 3.22 assumes air-filled gap. For dielectric-filled gaps,
//       surface capacitance must be calculated separately (see text).
// ============================================================================

class AP_SlotWithCover : public AP_SlotAperture {
public:
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
            throw std::invalid_argument("Cover gap (τ) must be positive");
        }

        // Calculate correction coefficient Fᵤ
        calculateCorrectionCoefficient();
    }

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        using namespace std::complex_literals;

        // Get slot-line impedance from parent
        double Z0s = getSlotLineImpedance();

        // Calculate aperture impedance with cover (Eq 3.22)
        double k0 = (2.0 * M_PI * f_Hz) / C_LIGHT;

        // Use protected getters to access parent's geometry
        double l = getApertureWidthImpl();
        double w = getApertureHeightImpl();

        // Argument for tangent function
        double arg = k0 * (l/2.0 - w/2.0 - 4.0*m_tau);

        // Handle numerical issues
        if (std::abs(arg) > 100.0) arg = std::copysign(100.0, arg);

        Complex Z_ap = 1.0i * Z0s * m_Fu * std::tan(arg);

        // Avoid division by zero
        if (std::abs(Z_ap) < 1e-9) Z_ap = Complex(1e-9, 0.0);

        // Convert to admittance
        Complex Y_ap = 1.0 / Z_ap;

        // Apply geometric factor (2a/l)
        double a = getEnclosureWidthImpl();
        Y_ap = Y_ap * (2.0 * a / l);

        return {{ Y_ap, -Y_ap }, {-Y_ap, Y_ap }};
    }

    std::string getDescription() const override {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "Slot with Cover: l=%.1fmm, w=%.1fmm, τ=%.2fmm, Fu=%.3f",
                      getApertureWidthImpl() * 1000.0,
                      getApertureHeightImpl() * 1000.0,
                      m_tau * 1000.0, m_Fu);
        return std::string(buf);
    }

    // Accessors
    double getCoverGap() const { return m_tau; }
    double getCorrectionCoefficient() const { return m_Fu; }

private:
    double m_tau;
    double m_Fu;

    void calculateCorrectionCoefficient() {
        double l = getApertureWidthImpl();
        double w = getApertureHeightImpl();
        double h = getEnclosureHeightImpl();

        // Equation 3.23: Fᵤ = √[2τ(l - w + 2τ) / (h·w)]
        double numerator = 2.0 * m_tau * (l - w + 2.0 * m_tau);
        double denominator = h * w;

        m_Fu = (denominator > 0) ? std::sqrt(numerator/denominator) : 1.0;
        m_Fu = std::max(0.1, std::min(10.0, m_Fu));
    }
};


} // namespace EMCore

#endif // AP_SLOTWITHCOVER_H
