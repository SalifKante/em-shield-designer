#ifndef TL_DIELECTRICCAVITY_H
#define TL_DIELECTRICCAVITY_H

#include "TL_EmptyCavity.h"  // Inherits from TL_EmptyCavity
#include <cmath>
#include <stdexcept>

namespace EMCore {

// ============================================================================
// DIELECTRIC-FILLED CAVITY TRANSMISSION LINE (TE₁₀ Mode)
// ============================================================================
// Models a rectangular cavity partially filled with dielectric material.
// Implements Russian Theory: Section 3.1.2.2 (Equations 3.19-3.21)
//
// Physical Configuration (Figure 3.8):
//   - Cavity dimensions: width a, height b, length L
//   - Dielectric layer: thickness h, relative permittivity εᵣ
//   - Air gap: thickness (b - h)
//
// Key Modification from Empty Cavity:
//   - Effective permittivity εₑff accounts for partial dielectric filling
//   - Modified propagation constant kₐ uses εₑff (Equation 3.19)
//   - Y-parameters use same form as empty cavity but with modified kₐ
//
// Effective Permittivity Calculation:
//   Method 1 (Maxwell-Garnett): Equation 3.20
//   Method 2 (Lichtenecker):    Equation 3.21 (more accurate for thin layers)
// ============================================================================

class TL_DielectricCavity : public TL_EmptyCavity {
public:
    // Effective permittivity calculation method
    enum class EffPermMethod {
        MAXWELL_GARNETT,    // Equation 3.20
        LICHTENECKER        // Equation 3.21
    };

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * @brief Create a dielectric-filled rectangular cavity
     * @param node_from Starting node (input port)
     * @param node_to Ending node (output port)
     * @param branch_id Unique branch identifier
     * @param a_m Cavity width (broad dimension) [m]
     * @param b_m Cavity height (narrow dimension) [m]
     * @param L_m Cavity length (propagation direction) [m]
     * @param h_m Dielectric layer thickness [m] (h ≤ b)
     * @param epsilon_r Relative permittivity of dielectric
     * @param method Effective permittivity calculation method
     */
    TL_DielectricCavity(int node_from, int node_to, int branch_id,
                        double a_m, double b_m, double L_m,
                        double h_m, double epsilon_r,
                        EffPermMethod method = EffPermMethod::LICHTENECKER)
        : TL_EmptyCavity(node_from, node_to, branch_id, a_m, b_m, L_m)
        , m_h(h_m)
        , m_epsilon_r(epsilon_r)
        , m_method(method)
    {
        // Validate dielectric parameters
        if (m_h <= 0.0) {
            throw std::invalid_argument(
                "Dielectric thickness must be positive"
                );
        }
        if (m_h > b_m) {
            throw std::invalid_argument(
                "Dielectric thickness cannot exceed cavity height"
                );
        }
        if (m_epsilon_r < 1.0) {
            throw std::invalid_argument(
                "Relative permittivity must be >= 1.0"
                );
        }

        // Calculate effective permittivity
        m_epsilon_eff = calculateEffectivePermittivity();

        // Recalculate cutoff frequency with dielectric
        m_fc10_dielectric = C_LIGHT / (2.0 * a_m * std::sqrt(m_epsilon_eff));
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION (OVERRIDE)
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        using namespace std::complex_literals;

        // --------------------------------------------------------------------
        // STEP 1: Free-space parameters
        // --------------------------------------------------------------------
        double lambda = C_LIGHT / f_Hz;
        double k0 = (2.0 * M_PI) / lambda;

        double sqrt_eff = std::sqrt(m_epsilon_eff);
        double lambda_c_eff = 2.0 * getWidth() * sqrt_eff;  // Modified cutoff wavelength

        // --------------------------------------------------------------------
        // STEP 2: Modified propagation constant (Equation 3.19)
        // --------------------------------------------------------------------
        // kₐ = (2π√εₑff/λ) · √[1 - (λ/(2a√εₑff))²]
        //    = k₀ · √εₑff · √[1 - (λ/λc_eff)²]
        //
        // where λc_eff = 2a√εₑff

        double ratio = lambda / lambda_c_eff;

        std::complex<double> sqrt_term;
        if (ratio < 1.0) {
            // Above cutoff: propagating mode (real)
            sqrt_term = std::sqrt(1.0 - ratio * ratio);
        } else {
            // Below cutoff: evanescent mode (imaginary)
            sqrt_term = 1.0i * std::sqrt(ratio * ratio - 1.0);
        }

        std::complex<double> ka = k0 * sqrt_eff * sqrt_term;
        std::complex<double> beta = ka * getLength();

        // --------------------------------------------------------------------
        // STEP 3: Waveguide impedance (Equation 3.17, with modified kₐ)
        // --------------------------------------------------------------------
        // Zg = ωμ₀/kₐ  (same form, but kₐ replaces kg)
        double omega = 2.0 * M_PI * f_Hz;
        std::complex<double> Zg = (omega * MU_0) / ka;

        // --------------------------------------------------------------------
        // STEP 4: Y-parameters (Equations 3.14, 3.15 with modified parameters)
        // --------------------------------------------------------------------
        // Y11 = Y22 = 1/(j·Zg·tan(kₐ·L))     [Eq 3.14]
        // Y12 = 1/(Zg·sin(kₐ·L))             [Eq 3.15]
        // Y21 = -Y12                          [Russian convention]

        std::complex<double> tan_beta = std::tan(beta);
        std::complex<double> sin_beta = std::sin(beta);

        // Handle numerical singularities
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

        return {
            {Complex(Y11.real(), Y11.imag()), Complex(Y12.real(), Y12.imag())},
            {Complex(Y21.real(), Y21.imag()), Complex(Y22.real(), Y22.imag())}
        };
    }

    // Base class getVoltageSourceVector() returns {0, 0} — correct for passive element.

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {
        // First check base class validity
        if (!TL_EmptyCavity::isValid(error_msg)) {
            return false;
        }

        // Check dielectric parameters
        if (m_h <= 0.0) {
            error_msg = "Dielectric thickness must be positive";
            return false;
        }

        if (m_h > getHeight()) {
            error_msg = "Dielectric thickness cannot exceed cavity height";
            return false;
        }

        if (m_epsilon_r < 1.0) {
            error_msg = "Relative permittivity must be >= 1.0";
            return false;
        }

        if (std::isnan(m_epsilon_eff) || m_epsilon_eff < 1.0) {
            error_msg = "Effective permittivity calculation failed";
            return false;
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        char buf[512];
        const char* method_str = (m_method == EffPermMethod::MAXWELL_GARNETT)
                                     ? "Maxwell-Garnett" : "Lichtenecker";

        std::snprintf(buf, sizeof(buf),
                      "Dielectric Cavity: a=%.1fmm, b=%.1fmm, L=%.1fmm, "
                      "h=%.1fmm (h/b=%.2f), er=%.2f, e_eff=%.3f (%s)",
                      getWidth() * 1000.0,
                      getHeight() * 1000.0,
                      getLength() * 1000.0,
                      m_h * 1000.0,
                      m_h / getHeight(),
                      m_epsilon_r,
                      m_epsilon_eff,
                      method_str);
        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    // Dielectric-specific getters
    double getDielectricThickness() const { return m_h; }
    double getRelativePermittivity() const { return m_epsilon_r; }
    double getEffectivePermittivity() const { return m_epsilon_eff; }
    EffPermMethod getMethod() const { return m_method; }

    // Modified cutoff frequency with dielectric
    double getDielectricCutoffFrequency() const { return m_fc10_dielectric; }

    // Fill factor (h/b)
    double getFillFactor() const { return m_h / getHeight(); }

    // Check if partially filled
    bool isPartiallyFilled() const { return m_h < getHeight(); }

    // Check if completely filled
    bool isCompletelyFilled() const { return std::abs(m_h - getHeight()) < 1e-9; }

private:
    double m_h;                 // Dielectric layer thickness [m]
    double m_epsilon_r;         // Relative permittivity of dielectric
    double m_epsilon_eff;       // Effective permittivity
    EffPermMethod m_method;
    double m_fc10_dielectric;   // Modified cutoff frequency [Hz]

    // ========================================================================
    // EFFECTIVE PERMITTIVITY CALCULATION
    // ========================================================================

    double calculateEffectivePermittivity() const {
        double h_b = m_h / getHeight();  // Fill factor (h/b)

        switch (m_method) {
        case EffPermMethod::MAXWELL_GARNETT:
            return calculateMaxwellGarnett(h_b);

        case EffPermMethod::LICHTENECKER:
            return calculateLichtenecker(h_b);

        default:
            return calculateLichtenecker(h_b);  // Default fallback
        }
    }

    /**
     * @brief Maxwell-Garnett formula (Equation 3.20)
     *
     * From Russian theory (Image 5, p.102):
     *
     *   εₑff = [ 2·ha·(εᵣ - 1) + (εᵣ + 2) ] / [ (εᵣ + 2) - ha·(εᵣ - 1) ]
     *
     * where ha = h/b is the filling factor.
     *
     * Boundary cases:
     *   ha = 0 (no dielectric):  εₑff = (εᵣ+2)/(εᵣ+2) = 1.0      ✓
     *   ha = 1 (full dielectric): εₑff = (2εᵣ-2+εᵣ+2)/(εᵣ+2-εᵣ+1)
     *                                   = 3εᵣ/3 = εᵣ               ✓
     */
    double calculateMaxwellGarnett(double h_b) const {
        // [FIX #1 - CRITICAL] Equation 3.20 from Russian theory
        // Denominator: (εᵣ + 2) - ha·(εᵣ - 1)
        //              ^^^^^^^^
        //              Was incorrectly coded as (εᵣ - 2)
        double numerator   = 2.0 * h_b * (m_epsilon_r - 1.0) + (m_epsilon_r + 2.0);
        double denominator = (m_epsilon_r + 2.0) - h_b * (m_epsilon_r - 1.0);

        // Guard against division by zero
        // (occurs when h/b = (εᵣ+2)/(εᵣ-1), which is > 1 for εᵣ > 1, so
        //  should never happen with valid h ≤ b and εᵣ ≥ 1)
        if (std::abs(denominator) < 1e-10) {
            // Fallback to Lichtenecker if Maxwell-Garnett is singular
            return calculateLichtenecker(h_b);
        }

        double eps_eff = numerator / denominator;

        // Sanity check: εₑff should be between 1 and εᵣ
        eps_eff = std::max(1.0, std::min(m_epsilon_r, eps_eff));

        return eps_eff;
    }

    /**
     * @brief Lichtenecker formula (Equation 3.21)
     *
     * From Russian theory (Image 5, p.102):
     *
     *   lg(εₑff) = (h/b)·lg(εᵣ) + ((b-h)/b)·lg(1)
     *
     * Since lg(1) = 0, this simplifies to:
     *
     *   εₑff = εᵣ^(h/b)
     *
     * Boundary cases:
     *   h/b = 0: εₑff = εᵣ^0 = 1.0    ✓
     *   h/b = 1: εₑff = εᵣ^1 = εᵣ     ✓
     */
    double calculateLichtenecker(double h_b) const {
        return std::pow(m_epsilon_r, h_b);
    }
};

} // namespace EMCore

#endif // TL_DIELECTRICCAVITY_H
