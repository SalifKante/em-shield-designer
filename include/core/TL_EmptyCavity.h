#ifndef TL_EMPTYCAVITY_H
#define TL_EMPTYCAVITY_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"
#include <cmath>

namespace EMCore {
// ============================================================================
// EMPTY CAVITY TRANSMISSION LINE (TE₁₀ Mode)
// ============================================================================
// Models a rectangular cavity (width a, height b, length L) with no
// dielectric filling, operating in the fundamental TE₁₀ waveguide mode.
//
// Physical Parameters:
//   - a: cavity width (broad dimension) [m]
//   - b: cavity height (narrow dimension) [m]
//   - L: cavity length (propagation direction) [m]
//
// Electromagnetic Behavior:
//   - Cutoff frequency: fc₁₀ = c/(2a)
//   - Below fc₁₀: evanescent (no propagation)
//   - Above fc₁₀: propagating TE₁₀ mode
//
// Y-Parameters (Russian Theory - Equations 3.14, 3.15, 3.17):
//   Y₁₁ = Y₂₂ = 1/(j·Zg·tan(kg·L))     [Eq 3.14]
//   Y₁₂ = 1/(Zg·sin(kg·L))              [Eq 3.15]
//   Y₂₁ = -Y₁₂                          [Eq 3.15, explicit negative]
//
// Where:
//   Zg = (2πf·μ₀)/kg = ωμ₀/kg          [Eq 3.17] Waveguide impedance [Ω]
//   kg = 2π/λg                          [rad/m]  Propagation constant
//   λg = λ₀/√(1-(fc/f)²)               [m]      Guided wavelength
// ============================================================================

class TL_EmptyCavity : public BranchTemplate {
public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    TL_EmptyCavity(int node_from, int node_to, int branch_id,
                   double a_m, double b_m, double L_m)
        : BranchTemplate(BranchType::TL_EMPTY_CAVITY, node_from, node_to, branch_id)
        , m_a(a_m)
        , m_b(b_m)
        , m_L(L_m)
    {
        // Precompute cutoff frequency (doesn't change with frequency)
        m_f_c10 = C_LIGHT / (2.0 * m_a);  // fc = c/(2a) [Hz]
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION (implements pure virtual from base class)
    // ========================================================================
    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        using namespace std::complex_literals;  // Enable 1i notation

        // --------------------------------------------------------------------
        // STEP 1: Check if below cutoff
        // --------------------------------------------------------------------
        if (f_Hz < m_f_c10) {
            // Evanescent mode: return very small admittance
            // (Physically, waves decay exponentially and don't propagate)
            const double Y_small = 1e-12;
            return {{Complex(Y_small, 0.0), Complex(0.0, 0.0)},
                    {Complex(0.0, 0.0), Complex(Y_small, 0.0)}};
        }

        // --------------------------------------------------------------------
        // STEP 2: Propagation parameters
        // --------------------------------------------------------------------
        double ratio = m_f_c10 / f_Hz;                      // fc/f
        double sqrt_term = std::sqrt(1.0 - ratio * ratio);  // √(1 - (fc/f)²)

        // --------------------------------------------------------------------
        // STEP 3: Guided wavelength and propagation constant
        // --------------------------------------------------------------------
        double lambda_0 = C_LIGHT / f_Hz;           // Free-space wavelength [m]
        double lambda_g = lambda_0 / sqrt_term;     // Guided wavelength [m]
        double kg = (2.0 * M_PI) / lambda_g;        // Propagation constant [rad/m]
        double beta = kg * m_L;                     // Electrical length [rad]

        // --------------------------------------------------------------------
        // STEP 4: WAVEGUIDE IMPEDANCE (Russian Theory - Equation 3.17)
        // --------------------------------------------------------------------
        // Zg = (2πf·μ₀) / kg = ωμ₀ / kg
        double omega = 2.0 * M_PI * f_Hz;
        double Zg = (omega * MU_0) / kg;

        // --------------------------------------------------------------------
        // STEP 5: Y-PARAMETERS (Russian Theory - Equations 3.14, 3.15)
        // --------------------------------------------------------------------
        // Y11 = Y22 = 1 / (j·Zg·tan(kg·L))     [Eq 3.14]
        // Y12 = 1 / (Zg·sin(kg·L))             [Eq 3.15]
        // Y21 = -Y12                            [Eq 3.15, explicit negative!]

        Complex Y11 = 1.0 / (1.0i * Zg * std::tan(beta));
        Complex Y12 = 1.0 / (Zg * std::sin(beta));
        Complex Y21 = -Y12;  // Russian convention: Y21 = -Y12
        Complex Y22 = Y11;

        return {{Y11, Y12},
                {Y21, Y22}};
    }

    // ========================================================================
    // VALIDATION (implements pure virtual from base class)
    // ========================================================================
    bool isValid(std::string& error_msg) const override {
        // Check for positive dimensions
        if (m_a <= 0.0 || m_b <= 0.0 || m_L <= 0.0) {
            error_msg = "Cavity dimensions must be positive";
            return false;
        }

        // Typical constraint for TE₁₀ mode dominance
        if (m_a < m_b) {
            error_msg = "Warning: For TE10 mode, typically a >= b (width >= height)";
            // Not a hard error, but worth warning the user
        }

        // Check for reasonable physical dimensions (1 mm to 1 meter)
        if (m_a < 0.001 || m_a > 1.0) {
            error_msg = "Cavity width out of reasonable range (1mm - 1m)";
            return false;
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION (implements pure virtual from base class)
    // ========================================================================
    std::string getDescription() const override {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "Empty Cavity: a=%.1fmm, b=%.1fmm, L=%.1fmm (fc10=%.2fGHz)",
                      m_a * 1000.0,        // Convert m to mm
                      m_b * 1000.0,
                      m_L * 1000.0,
                      m_f_c10 / 1e9);      // Convert Hz to GHz
        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS (cavity-specific getters)
    // ========================================================================
    double getWidth() const { return m_a; }
    double getHeight() const { return m_b; }
    double getLength() const { return m_L; }
    double getCutoffFrequency() const { return m_f_c10; }

private:
    // Cavity physical dimensions [m]
    double m_a;       // Width (broad dimension)
    double m_b;       // Height (narrow dimension)
    double m_L;       // Length (propagation direction)

    // Precomputed electromagnetic parameter
    double m_f_c10;   // TE₁₀ cutoff frequency [Hz]
};

} // namespace EMCore

#endif // TL_EMPTYCAVITY_H
