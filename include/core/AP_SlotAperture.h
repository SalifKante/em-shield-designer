#ifndef AP_SLOTAPERTURE_H
#define AP_SLOTAPERTURE_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"
#include <cmath>
#include <complex>
#include <iostream>

// Uncomment to enable detailed debug output:
// #define DEBUG_AP_SLOT
#ifdef DEBUG_AP_SLOT
#include <iomanip>
#endif

namespace EMCore {

// ============================================================================
// SLOT APERTURE IN A METALLIC WALL
// ============================================================================
// Models an aperture (slot) in the front wall of a rectangular enclosure as
// a shunt admittance element in the MNA equivalent circuit.
//
// Physical Model:
//   - Represents a rectangular slot (l × w) in the enclosure front wall
//   - Modelled as a shunt branch between two circuit nodes
//   - Admittance computed via Cohn's coplanar strip-line formula [Ref 23]
//
// Key Equation (3.13):
//   Y_ap = (2a/l) · (1 / j·Z₀ₛ) · cot(πl/λ)
//
//   where:
//     a   = enclosure broad dimension [m]
//     l   = aperture width [m]
//     λ   = free-space wavelength = c/f [m]
//     Z₀ₛ = characteristic impedance of the equivalent coplanar strip
//           transmission line with conductor separation w [Ω]
//
// Slot-Line Impedance Z₀ₛ — Cohn Formula Convention:
//   The formula implemented here is:
//     Z₀ₛ = Z₀ · K(kₑ) / K'(kₑ)
//
//   where K/K' is the ratio of complete elliptic integrals, evaluated via
//   the standard approximation:
//     K(kₑ)/K'(kₑ) ≈ π / ln[2·(1 + (1−kₑ²)^(1/4)) / (1 − (1−kₑ²)^(1/4))]
//
//   NOTE ON CONVENTION: This differs from the classical Cohn (1954) textbook
//   form Z₀ₛ = (Z₀/2)·K'(k)/K(k) by the reciprocal and factor of 2.
//   The formula used here matches the implementation in the validated MATLAB
//   baseline (Reference [23], Eq. used in Ivanov et al. [1]).
//   DO NOT change this formula without re-validating against the MATLAB
//   reference data.
//
// Modular Parameter:
//   kₑ = wₑ / b
//   where wₑ is the Schneider-corrected effective aperture height accounting
//   for finite wall thickness t.
//
// Physical Validity Domain:
//   kₑ ∈ (0, 1): Normal operating range. kₑ > 1 is outside the validity
//   domain of the Cohn slot-line model (slot taller than enclosure).
//   A warning is emitted if this condition is detected.
// ============================================================================

class AP_SlotAperture : public BranchTemplate {
public:

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * @brief Create a slot aperture branch
     * @param node_from  Starting node index
     * @param node_to    Ending node index
     * @param branch_id  Unique branch identifier
     * @param enclosure_width_a   Broad dimension of enclosure cross-section [m]
     * @param enclosure_height_b  Narrow dimension of enclosure cross-section [m]
     * @param aperture_width_l    Aperture width (dimension along a) [m]
     * @param aperture_height_w   Aperture height (dimension along b) [m]
     * @param wall_thickness_t    Front-wall thickness [m]
     */
    AP_SlotAperture(int node_from, int node_to, int branch_id,
                    double enclosure_width_a,
                    double enclosure_height_b,
                    double aperture_width_l,
                    double aperture_height_w,
                    double wall_thickness_t)
        : BranchTemplate(BranchType::AP_SLOT_APERTURE, node_from, node_to, branch_id),
        a_(enclosure_width_a),
        b_(enclosure_height_b),
        l_(aperture_width_l),
        w_(aperture_height_w),
        t_(wall_thickness_t),
        Z0s_(0.0)
    {
        calculateSlotLineImpedance();
    }

    // ========================================================================
    // INTERFACE IMPLEMENTATIONS
    // ========================================================================

    std::string getDescription() const override {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "Slot Aperture: l=%.1fmm, w=%.1fmm, Z0s=%.2f Ohm",
                      l_ * 1000.0, w_ * 1000.0, Z0s_);
        return std::string(buf);
    }

    bool isValid(std::string& errorMsg) const override {
        if (a_ <= 0 || b_ <= 0 || l_ <= 0 || w_ <= 0) {
            errorMsg = "Aperture: all dimensions must be positive";
            return false;
        }
        if (t_ <= 0) {
            errorMsg = "Aperture: wall thickness must be positive";
            return false;
        }
        if (std::isnan(Z0s_) || std::isinf(Z0s_) || Z0s_ <= 0.0) {
            errorMsg = "Aperture: slot-line impedance calculation failed (NaN, Inf, or non-positive)";
            return false;
        }
        return true;
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION — Equation (3.13)
    // ========================================================================
    //
    //   Y_ap = (2a/l) · (1/j·Z₀ₛ) · cot(πl/λ)
    //
    //   Decomposed as:
    //     geometric_factor = 2a/l           [dimensionless coupling coefficient]
    //     slot_admittance  = 1/(j·Z₀ₛ)     [1/Ω = S]
    //     cot_term         = cot(πl/λ)      [dimensionless]
    //
    //   Resonance singularity: cot(πl/λ) → ∞ when πl/λ = nπ (i.e. l = nλ).
    //   At these frequencies the slot is at resonance; tan → 0 is clamped to
    //   a small nonzero value preserving sign to avoid NaN propagation.

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {

        const double lambda          = C_LIGHT / f_Hz;
        const Complex jUnit(0.0, 1.0);

        // Step 1: Geometric coupling factor 2a/l
        const double geometric_factor = (2.0 * a_) / l_;

        // Step 2: Slot admittance 1/(j·Z₀ₛ)
        const Complex slot_admittance = 1.0 / (jUnit * Z0s_);

        // Step 3: cot(πl/λ) — guard against slot resonance (tan → 0)
        const double arg     = M_PI * l_ / lambda;
        double       tan_val = std::tan(arg);

        constexpr double EPS = 1e-12;
        if (std::abs(tan_val) < EPS) {
            tan_val = (tan_val >= 0.0) ? EPS : -EPS;   // Preserve sign
        }
        const double cot_term = 1.0 / tan_val;

        // Step 4: Assemble Y_ap
        const Complex Y_ap = geometric_factor * slot_admittance * cot_term;

#ifdef DEBUG_AP_SLOT
        std::cout << "\n[AP_SlotAperture] f=" << f_Hz/1e9 << " GHz"
                  << "  lambda=" << lambda << " m"
                  << "  arg=" << arg << " rad"
                  << "  cot=" << cot_term
                  << "  Y_ap=(" << Y_ap.real() << ", " << Y_ap.imag() << ") S\n";
#endif

        // Standard 2-port passive admittance stamp
        return {
            { Y_ap, -Y_ap },
            {-Y_ap,  Y_ap }
        };
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    double getSlotLineImpedance()  const { return Z0s_; }
    double getEnclosureWidth()     const { return a_;   }
    double getEnclosureHeight()    const { return b_;   }
    double getApertureWidth()      const { return l_;   }
    double getApertureHeight()     const { return w_;   }
    double getWallThickness()      const { return t_;   }

protected:
    // ------------------------------------------------------------------
    // Protected members — accessible to derived classes (AP_SlotWithCover)
    // ------------------------------------------------------------------
    double a_;    // Enclosure broad dimension [m]
    double b_;    // Enclosure narrow dimension [m]
    double l_;    // Aperture width [m]
    double w_;    // Aperture height [m]
    double t_;    // Wall thickness [m]
    double Z0s_;  // Coplanar strip-line characteristic impedance [Ω]

private:
    // ========================================================================
    // SLOT-LINE IMPEDANCE — Cohn's Elliptic Integral Approximation
    // ========================================================================
    //
    // Step 1: Schneider effective-width correction for finite wall thickness
    //   wₑ = w − (5t/4π)·[1 + ln(4πw/t)]
    //
    // Step 2: Modular parameter
    //   kₑ = wₑ / b
    //   Normal domain: kₑ ∈ (0,1). kₑ > 1 triggers a validity warning.
    //
    // Step 3: Elliptic integral ratio approximation
    //   K(kₑ)/K'(kₑ) ≈ π / ln{2·[1 + (1−kₑ²)^(1/4)] / [1 − (1−kₑ²)^(1/4)]}
    //   Complex arithmetic used to handle kₑ > 1 without raising an exception,
    //   matching MATLAB reference behavior. Only the real part is retained.
    //
    // Step 4:
    //   Z₀ₛ = Z₀ · Re{ K(kₑ)/K'(kₑ) }
    //
    // See: Ivanov et al. [1], Reference [23] cited in Eq. (3.13).

    void calculateSlotLineImpedance() {

        // --- Step 1: Schneider effective width ---
        const double we = w_ - ((5.0 * t_) / (4.0 * M_PI))
                                   * (1.0 + std::log((4.0 * M_PI * w_) / t_));

        // --- Step 2: Modular parameter ---
        const double ke = we / b_;

        // Validity-domain warning for ke > 1
        if (ke > 1.0) {
            std::cerr << "[AP_SlotAperture] WARNING: modular parameter ke = "
                      << ke << " > 1 (we=" << we*1e3 << " mm > b=" << b_*1e3
                      << " mm). Geometry is outside the validity domain of the "
                      << "Cohn slot-line model. Z0s will be computed using "
                      << "complex arithmetic; result may not be physically meaningful.\n";
        }

        // --- Step 3: Elliptic integral ratio via complex arithmetic ---
        const std::complex<double> ke_c(ke, 0.0);
        const std::complex<double> one_minus_ke2  = 1.0 - ke_c * ke_c;
        const std::complex<double> fourth_root    = std::pow(one_minus_ke2, 0.25);

        const std::complex<double> numer = 1.0 + fourth_root;
        std::complex<double> denom = 1.0 - fourth_root;

        if (std::abs(denom) < 1e-12) {
            denom = std::complex<double>(1e-12, 0.0);
        }

        const std::complex<double> log_arg  = 2.0 * numer / denom;
        const std::complex<double> K_ratio  = M_PI / std::log(log_arg);

        // Warn if significant imaginary part remains (indicates ke well outside domain)
        if (std::abs(K_ratio.imag()) > 1e-4 * std::abs(K_ratio.real())) {
            std::cerr << "[AP_SlotAperture] WARNING: K(ke)/K'(ke) has non-negligible "
                      << "imaginary part = " << K_ratio.imag()
                      << ". Imaginary component will be discarded.\n";
        }

        // --- Step 4: Z₀ₛ = Z₀ · Re{K/K'} ---
        Z0s_ = Z_0 * K_ratio.real();

#ifdef DEBUG_AP_SLOT
        std::cout << "[AP_SlotAperture] Cohn: we=" << we*1e3 << "mm  ke=" << ke
                  << "  K/K'=" << K_ratio << "  Z0s=" << Z0s_ << " Ohm\n";
#endif
    }
};

} // namespace EMCore

#endif // AP_SLOTAPERTURE_H
