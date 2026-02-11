#ifndef AP_SLOTAPERTURE_H
#define AP_SLOTAPERTURE_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"
#include <cmath>
#include <complex>

// Uncomment to enable detailed debug output:
// #define DEBUG_AP_SLOT
#ifdef DEBUG_AP_SLOT
#include <iostream>
#include <iomanip>
#endif

namespace EMCore {

class AP_SlotAperture : public BranchTemplate {
public:
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

    std::string getDescription() const override {
        return "Slot Aperture (l=" + std::to_string(l_*1000) +
               "mm, w=" + std::to_string(w_*1000) +
               "mm, Z0s=" + std::to_string(Z0s_) + " Ohm)";
    }

    bool isValid(std::string& errorMsg) const override {
        if (a_ <= 0 || b_ <= 0 || l_ <= 0 || w_ <= 0) {
            errorMsg = "Aperture: dimensions must be positive";
            return false;
        }

        if (t_ <= 0) {
            errorMsg = "Aperture: wall thickness must be positive";
            return false;
        }
        if (std::isnan(Z0s_) || std::isinf(Z0s_)) {
            errorMsg = "Aperture: slot-line impedance calculation failed (NaN or Inf)";
            return false;
        }
        return true;
    }

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        // Aperture admittance (Eq. 3.13)
        // Y_ap = (2a/l) · (1/jZ₀ₛ) · cot(πl/λ)

        double lambda = C_LIGHT / f_Hz;  // Wavelength [m]
        Complex jUnit(0.0, 1.0);

        // Step 1: Geometric factor
        double geometric_factor = (2.0 * a_) / l_;

        // Step 2: Slot admittance
        Complex slot_admittance = 1.0 / (jUnit * Z0s_);

        // Step 3: Cotangent term with singularity protection
        // [FIX #1] Guard against tan→0 at slot resonance (πl/λ = nπ)
        double arg = M_PI * l_ / lambda;
        double tan_val = std::tan(arg);
        const double EPS = 1e-12;
        if (std::abs(tan_val) < EPS) {
            tan_val = (tan_val >= 0.0) ? EPS : -EPS;  // Preserve sign
        }
        double cot_term = 1.0 / tan_val;

        // Step 4: Final Y_ap
        Complex Y_ap = geometric_factor * slot_admittance * cot_term;

        // [FIX #2] Debug output only when DEBUG_AP_SLOT is defined
#ifdef DEBUG_AP_SLOT
        std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║        AP_SlotAperture::computeYParameters DEBUG          ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

        std::cout << std::fixed << std::setprecision(9);
        std::cout << "Input Parameters:\n";
        std::cout << "  Frequency:        f = " << f_Hz << " Hz (" << f_Hz/1e9 << " GHz)\n";
        std::cout << "  Wavelength:       lambda = " << lambda << " m\n";
        std::cout << "  Enclosure width:  a = " << a_ << " m (" << a_*1000 << " mm)\n";
        std::cout << "  Aperture width:   l = " << l_ << " m (" << l_*1000 << " mm)\n";
        std::cout << "  Slot impedance:  Z0s = " << Z0s_ << " Ohm\n\n";

        std::cout << "Step 1: Geometric Factor\n";
        std::cout << "  geometric_factor = 2*a/l\n";
        std::cout << "                   = 2 * " << a_ << " / " << l_ << "\n";
        std::cout << "                   = " << geometric_factor << "\n\n";

        std::cout << "Step 2: Slot Admittance\n";
        std::cout << "  slot_admittance = 1 / (j * Z0s)\n";
        std::cout << "                  = 1 / (j * " << Z0s_ << ")\n";
        std::cout << "                  = 1 / " << (jUnit * Z0s_) << "\n";
        std::cout << "                  = " << slot_admittance << " S\n";
        std::cout << "                  = (" << slot_admittance.real() << ", "
                  << slot_admittance.imag() << ") S\n\n";

        std::cout << "Step 3: Cotangent Calculation\n";
        std::cout << "  arg = pi * l / lambda\n";
        std::cout << "      = pi * " << l_ << " / " << lambda << "\n";
        std::cout << "      = " << arg << " rad\n";
        std::cout << "      = " << arg * 180.0 / M_PI << " degrees\n";
        std::cout << "  tan(arg) = " << tan_val << "\n";
        std::cout << "  cot(arg) = 1 / tan(arg)\n";
        std::cout << "           = " << cot_term << "\n\n";

        std::cout << "Step 4: Final Aperture Admittance\n";
        std::cout << "  Y_ap = geometric_factor * slot_admittance * cot_term\n";
        std::cout << "       = " << geometric_factor << " * " << slot_admittance << " * " << cot_term << "\n";
        std::cout << "       = " << Y_ap << " S\n";
        std::cout << "       = (" << Y_ap.real() << ", " << Y_ap.imag() << ") S\n";
        std::cout << "       = " << std::scientific << std::setprecision(6)
                  << Y_ap.real() << " " << (Y_ap.imag() >= 0 ? "+" : "") << Y_ap.imag() << "j S\n";

        std::cout << "\n2x2 Y-Matrix (shunt form):\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  [ " << Y_ap << "   " << -Y_ap << " ]\n";
        std::cout << "  [ " << -Y_ap << "   " << Y_ap << " ]\n";
        std::cout << "════════════════════════════════════════════════════════════\n\n";
#endif

        // For shunt admittance between two nodes:
        return {
            { Y_ap,  -Y_ap },
            {-Y_ap,   Y_ap }
        };
    }

    // [FIX #4] Removed redundant getVoltageSourceVector override.
    // Base class BranchTemplate already returns {0, 0} for passive elements.

    // ========================================================================
    // PUBLIC ACCESSORS (for external use)
    // ========================================================================

    double getSlotLineImpedance() const { return Z0s_; }

    double getEnclosureWidth() const { return a_; }
    double getEnclosureHeight() const { return b_; }
    double getApertureWidth() const { return l_; }
    double getApertureHeight() const { return w_; }
    double getWallThickness() const { return t_; }

protected:
    // ========================================================================
    // [FIX #5] Members moved from private to protected so derived classes
    // (e.g. AP_SlotWithCover) can access them directly.
    // Removed redundant protected Impl accessor functions.
    // ========================================================================
    double a_;   // Enclosure width [m]
    double b_;   // Enclosure height [m]
    double l_;   // Aperture width [m]
    double w_;   // Aperture height [m]
    double t_;   // Wall thickness [m]
    double Z0s_; // Slot-line impedance [Ω]

private:
    void calculateSlotLineImpedance() {
        // Cohn's formula for slot-line impedance
        // Matching MATLAB exactly, including complex arithmetic when ke > 1

        // Step 1: Effective width
        double we = w_ - ((5.0*t_)/(4.0*M_PI)) * (1.0 + std::log((4.0*M_PI*w_)/t_));

        // Step 2: Aspect ratio (can be > 1!)
        double ke = we / b_;

        // Step 3: Elliptic integral ratio K(ke)/K'(ke)
        // Use complex arithmetic to match MATLAB behavior
        std::complex<double> ke_complex(ke, 0.0);
        std::complex<double> ke_squared = ke_complex * ke_complex;

        // (1 - ke²)^(1/4) - allows negative under square root
        std::complex<double> one_minus_ke2 = 1.0 - ke_squared;
        std::complex<double> fourth_root = std::pow(one_minus_ke2, 0.25);

        // Calculate K = π / log(2*(1+(1-ke²)^(1/4)) / (1-(1-ke²)^(1/4)))
        std::complex<double> numerator = 1.0 + fourth_root;
        std::complex<double> denominator = 1.0 - fourth_root;

        // Guard against division by zero
        if (std::abs(denominator) < 1e-12) {
            denominator = std::complex<double>(1e-12, 0.0);
        }

        std::complex<double> log_arg = 2.0 * numerator / denominator;
        std::complex<double> K_complex = M_PI / std::log(log_arg);

        // [FIX #3] Use Z_0 from PhysicsConstants.h instead of hardcoded 120.0*M_PI
        // Take real part for Z0s (imaginary part should be small or zero)
        Z0s_ = Z_0 * K_complex.real();

        // Debug output for slot-line impedance calculation
#ifdef DEBUG_AP_SLOT
        std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║     Slot-Line Impedance Calculation (Cohn's Formula)      ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Input Parameters:\n";
        std::cout << "  Aperture height:    w = " << w_*1000 << " mm\n";
        std::cout << "  Enclosure height:   b = " << b_*1000 << " mm\n";
        std::cout << "  Wall thickness:     t = " << t_*1000 << " mm\n";
        std::cout << "  Aperture width:     l = " << l_*1000 << " mm\n";
        std::cout << "  Enclosure width:    a = " << a_*1000 << " mm\n\n";

        std::cout << "Intermediate Calculations:\n";
        std::cout << "  Effective width:  we = " << we*1000 << " mm\n";
        std::cout << "  Aspect ratio:     ke = we/b = " << ke << "\n";

        if (ke > 1.0) {
            std::cout << "  Note: ke > 1 requires complex arithmetic!\n";
        }

        std::cout << "\n  (1 - ke^2) = " << one_minus_ke2 << "\n";
        std::cout << "  (1 - ke^2)^(1/4) = " << fourth_root << "\n";
        std::cout << "  K (complex) = " << K_complex << "\n";
        std::cout << "  K (real part) = " << K_complex.real() << "\n";

        if (std::abs(K_complex.imag()) > 1e-6) {
            std::cout << "  Warning: K has significant imaginary part: " << K_complex.imag() << "\n";
        }

        std::cout << "\nFinal Result:\n";
        std::cout << "  Z0s = Z_0 * K_real\n";
        std::cout << "      = " << Z_0 << " * " << K_complex.real() << "\n";
        std::cout << "      = " << Z0s_ << " Ohm\n";
        std::cout << "════════════════════════════════════════════════════════════\n\n";
#endif
    }
};

} // namespace EMCore

#endif // AP_SLOTAPERTURE_H
