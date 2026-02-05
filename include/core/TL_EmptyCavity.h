#ifndef TL_EMPTYCAVITY_H
#define TL_EMPTYCAVITY_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"
#include <cmath>
#include <complex>

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
//   kg = k₀·√(1-(fc/f)²)               [rad/m]  Propagation constant
//   k₀ = 2π/λ₀ = 2πf/c                  [rad/m]  Free-space wavenumber
//
// Note on Formulation:
//   This differs from standard transmission line theory (Y₀ = 1/Z₀).
//   The Russian theory uses waveguide impedance Zg = ωμ₀/kg, resulting in
//   Y-parameters that are factor of 2 larger with different phase.
// ============================================================================

class TL_EmptyCavity : public BranchTemplate {
public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * @brief Create an empty rectangular cavity transmission line
     * @param node_from Starting node (input port)
     * @param node_to Ending node (output port)
     * @param branch_id Unique branch identifier
     * @param a_m Cavity width (broad dimension) [m]
     * @param b_m Cavity height (narrow dimension) [m]
     * @param L_m Cavity length (propagation direction) [m]
     */
    TL_EmptyCavity(int node_from, int node_to, int branch_id,
                   double a_m, double b_m, double L_m)
        : BranchTemplate(BranchType::TL_EMPTY_CAVITY, node_from, node_to, branch_id)
        , m_a(a_m)
        , m_b(b_m)
        , m_L(L_m)
    {
        // Precompute cutoff frequency (doesn't change with frequency)
        m_f_c10 = C_LIGHT / (2.0 * m_a);  // fc = c/(2a) [Hz]

        // Precompute cutoff wavelength
        m_lambda_c = 2.0 * m_a;  // λc = 2a [m]
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION (MATLAB-COMPATIBLE)
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        using namespace std::complex_literals;  // Enable 1i notation

        // --------------------------------------------------------------------
        // STEP 1: Free-space wavenumber and wavelength
        // --------------------------------------------------------------------
        double lambda = C_LIGHT / f_Hz;           // Free-space wavelength [m]
        double k0 = (2.0 * M_PI) / lambda;        // Free-space wavenumber [rad/m]

        // --------------------------------------------------------------------
        // STEP 2: Propagation constant (waveguide) - MATLAB COMPATIBLE
        // --------------------------------------------------------------------
        // kg = k₀·√(1 - (λ/λc)²)  [above cutoff, real]
        // kg = j·k₀·√((λ/λc)² - 1) [below cutoff, imaginary] - MATCHES MATLAB!
        //
        // Note: MATLAB continues calculations even below cutoff, using
        // imaginary propagation constant for evanescent mode.

        double ratio = lambda / m_lambda_c;       // λ/λc
        std::complex<double> kg;                  // Complex propagation constant

        if (f_Hz >= m_f_c10) {
            // Above cutoff: propagating mode (kg is real)
            double sqrt_term = std::sqrt(1.0 - ratio * ratio);
            kg = k0 * sqrt_term;                  // Real value
        } else {
            // Below cutoff: evanescent mode (kg is imaginary)
            // MATCHES MATLAB: kg = j * k0 * sqrt((λ/λc)² - 1)
            double sqrt_term = std::sqrt(ratio * ratio - 1.0);
            kg = 1.0i * k0 * sqrt_term;           // Purely imaginary
        }

        double beta_real = kg.real() * m_L;       // Real part of electrical length
        double beta_imag = kg.imag() * m_L;       // Imag part of electrical length
        std::complex<double> beta(beta_real, beta_imag);

        // --------------------------------------------------------------------
        // STEP 3: WAVEGUIDE IMPEDANCE (Russian Theory - Equation 3.17)
        // --------------------------------------------------------------------
        // Zg = (2πf·μ₀) / kg = ωμ₀ / kg
        // Note: kg can be complex (for evanescent mode), so Zg will be complex
        //
        // Physical interpretation:
        //   - For TE modes, Zg increases as frequency approaches cutoff
        //   - At cutoff (kg→0), Zg→∞ (no propagation)
        //   - Below cutoff, Zg is complex (reactive impedance)

        double omega = 2.0 * M_PI * f_Hz;
        std::complex<double> Zg = (omega * MU_0) / kg;  // Complex for evanescent

        // --------------------------------------------------------------------
        // STEP 4: Y-PARAMETERS (Russian Theory - Equations 3.14, 3.15)
        // --------------------------------------------------------------------
        // Y11 = Y22 = 1 / (j·Zg·tan(kg·L))     [Eq 3.14]
        // Y12 = 1 / (Zg·sin(kg·L))             [Eq 3.15]
        // Y21 = -Y12                            [Eq 3.15, explicit negative!]
        //
        // Important: For evanescent mode, tan(kg·L) and sin(kg·L) become
        // hyperbolic functions due to imaginary argument.
        // MATLAB handles this automatically with complex arithmetic.

        std::complex<double> tan_beta = std::tan(beta);
        std::complex<double> sin_beta = std::sin(beta);

        // Handle numerical issues near singularities
        const double EPS = 1e-12;
        if (std::abs(tan_beta) < EPS) {
            tan_beta = std::complex<double>(EPS, 0.0);
        }
        if (std::abs(sin_beta) < EPS) {
            sin_beta = std::complex<double>(EPS, 0.0);
        }

        std::complex<double> Y11 = 1.0 / (1.0i * Zg * tan_beta);
        std::complex<double> Y12 = 1.0 / (Zg * sin_beta);
        std::complex<double> Y21 = -Y12;  // Russian convention: Y21 = -Y12
        std::complex<double> Y22 = Y11;

        // --------------------------------------------------------------------
        // STEP 5: DEBUG OUTPUT (for validation)
        // --------------------------------------------------------------------
        // Uncomment for debugging evanescent mode calculations
        /*
        if (f_Hz < m_f_c10 && f_Hz > 1e5) {  // Below cutoff, but not too low
            std::cout << "\n[DEBUG TL_EmptyCavity] f = " << f_Hz/1e6 << " MHz < fc = "
                      << m_f_c10/1e6 << " MHz (evanescent)\n";
            std::cout << "  kg = " << kg << " rad/m\n";
            std::cout << "  Zg = " << Zg << " Ω\n";
            std::cout << "  tan(kg*L) = " << tan_beta << "\n";
            std::cout << "  sin(kg*L) = " << sin_beta << "\n";
            std::cout << "  Y11 = " << Y11 << " S\n";
            std::cout << "  Y12 = " << Y12 << " S\n";
        }
        */

        return {
            {Complex(Y11.real(), Y11.imag()), Complex(Y12.real(), Y12.imag())},
            {Complex(Y21.real(), Y21.imag()), Complex(Y22.real(), Y22.imag())}
        };
    }

    // ========================================================================
    // VOLTAGE SOURCE VECTOR (PASSIVE ELEMENT)
    // ========================================================================

    std::vector<Complex> getVoltageSourceVector(double f_Hz) const override {
        // Transmission line is passive - no voltage source
        return {Complex(0.0, 0.0), Complex(0.0, 0.0)};
    }

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {
        // Check for positive dimensions
        if (m_a <= 0.0 || m_b <= 0.0 || m_L <= 0.0) {
            error_msg = "Cavity dimensions must be positive";
            return false;
        }

        // Typical constraint for TE₁₀ mode dominance
        if (m_a < m_b) {
            error_msg = "Warning: For TE₁₀ mode, typically a ≥ b (width ≥ height)";
            // Not a hard error, but worth warning the user
        }

        // Check for reasonable physical dimensions (0.1 mm to 10 m)
        if (m_a < 0.0001 || m_a > 10.0) {
            error_msg = "Cavity width out of reasonable range (0.1mm - 10m)";
            return false;
        }

        if (m_b < 0.0001 || m_b > 10.0) {
            error_msg = "Cavity height out of reasonable range (0.1mm - 10m)";
            return false;
        }

        if (m_L < 0.0001 || m_L > 100.0) {
            error_msg = "Cavity length out of reasonable range (0.1mm - 100m)";
            return false;
        }

        // Check for NaN
        if (std::isnan(m_a) || std::isnan(m_b) || std::isnan(m_L)) {
            error_msg = "Cavity dimensions contain NaN";
            return false;
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
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
    // ACCESSORS
    // ========================================================================

    // Geometry
    double getWidth() const { return m_a; }
    double getHeight() const { return m_b; }
    double getLength() const { return m_L; }

    // Electromagnetic parameters
    double getCutoffFrequency() const { return m_f_c10; }
    double getCutoffWavelength() const { return m_lambda_c; }

    // Convenience methods
    bool isPropagating(double f_Hz) const { return f_Hz >= m_f_c10; }

    double getGuidedWavelength(double f_Hz) const {
        if (f_Hz < m_f_c10) return 0.0;  // Evanescent (infinite decay length)
        double lambda_0 = C_LIGHT / f_Hz;
        double ratio = m_f_c10 / f_Hz;
        return lambda_0 / std::sqrt(1.0 - ratio * ratio);
    }

    // Get propagation constant (complex, matches MATLAB)
    std::complex<double> getPropagationConstant(double f_Hz) const {
        double lambda = C_LIGHT / f_Hz;
        double k0 = (2.0 * M_PI) / lambda;
        double ratio = lambda / m_lambda_c;

        if (f_Hz >= m_f_c10) {
            double sqrt_term = std::sqrt(1.0 - ratio * ratio);
            return k0 * sqrt_term;  // Real
        } else {
            double sqrt_term = std::sqrt(ratio * ratio - 1.0);
            return std::complex<double>(0.0, k0 * sqrt_term);  // Imaginary
        }
    }

private:
    // Cavity physical dimensions [m]
    double m_a;         // Width (broad dimension)
    double m_b;         // Height (narrow dimension)
    double m_L;         // Length (propagation direction)

    // Precomputed electromagnetic parameters
    double m_f_c10;     // TE₁₀ cutoff frequency [Hz]
    double m_lambda_c;  // Cutoff wavelength [m]
};

} // namespace EMCore

#endif // TL_EMPTYCAVITY_H
