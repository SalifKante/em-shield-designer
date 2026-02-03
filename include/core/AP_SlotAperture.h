#ifndef AP_SLOTAPERTURE_H
#define AP_SLOTAPERTURE_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"
#include <cmath>
#include <complex>

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
        // REMOVED the check: if (l_ > a_ || w_ > b_)
        // Because MATLAB allows ke > 1 with complex arithmetic

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

        double geometric_factor = (2.0 * a_) / l_;
        Complex slot_admittance = 1.0 / (jUnit * Z0s_);

        // Cotangent calculation
        double arg = M_PI * l_ / lambda;
        double cot_term = 1.0 / std::tan(arg);  // cot(x) = 1/tan(x)

        Complex Y_ap = geometric_factor * slot_admittance * cot_term;

        // For shunt admittance between two nodes:
        return {
            { Y_ap,  -Y_ap },
            {-Y_ap,   Y_ap }
        };
    }

    std::vector<Complex> getVoltageSourceVector(double f_Hz) const override {
        return {Complex(0.0, 0.0), Complex(0.0, 0.0)};
    }

private:
    double a_;   // Enclosure width [m]
    double b_;   // Enclosure height [m]
    double l_;   // Aperture width [m]
    double w_;   // Aperture height [m]
    double t_;   // Wall thickness [m]
    double Z0s_; // Slot-line impedance [Ω]

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

        // Calculate K = π / log(2*(1+√(1-ke²)^(1/4)) / (1-√(1-ke²)^(1/4)))
        std::complex<double> numerator = 1.0 + fourth_root;
        std::complex<double> denominator = 1.0 - fourth_root;

        // Guard against division by zero
        if (std::abs(denominator) < 1e-12) {
            denominator = std::complex<double>(1e-12, 0.0);
        }

        std::complex<double> log_arg = 2.0 * numerator / denominator;
        std::complex<double> K_complex = M_PI / std::log(log_arg);

        // Take real part for Z0s (imaginary part should be small or zero)
        Z0s_ = 120.0 * M_PI * K_complex.real();

// Debug output
#ifdef DEBUG_APERTURE
        std::cout << "\n=== Aperture Slot-Line Impedance Calculation ===\n";
        std::cout << "Input parameters:\n";
        std::cout << "  w = " << w_*1000 << " mm (aperture height)\n";
        std::cout << "  b = " << b_*1000 << " mm (enclosure height)\n";
        std::cout << "  t = " << t_*1000 << " mm (wall thickness)\n";
        std::cout << "  l = " << l_*1000 << " mm (aperture width)\n";
        std::cout << "  a = " << a_*1000 << " mm (enclosure width)\n\n";

        std::cout << "Intermediate values:\n";
        std::cout << "  we = " << we*1000 << " mm (effective width)\n";
        std::cout << "  ke = " << ke << " (aspect ratio)\n";
        std::cout << "  Note: ke > 1 causes complex arithmetic!\n\n";

        std::cout << "  (1-ke²) = " << one_minus_ke2 << "\n";
        std::cout << "  (1-ke²)^(1/4) = " << fourth_root << "\n";
        std::cout << "  K (complex) = " << K_complex << "\n";
        std::cout << "  K (real part) = " << K_complex.real() << "\n\n";

        std::cout << "Final result:\n";
        std::cout << "  Z0s = " << Z0s_ << " Ω\n";
        std::cout << "===============================================\n\n";
#endif
    }
};

} // namespace EMCore

#endif // AP_SLOTAPERTURE_H
