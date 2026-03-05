#ifndef TL_EMPTYCAVITY_H
#define TL_EMPTYCAVITY_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"
#include <cmath>
#include <complex>
#include <iostream>

namespace EMCore {

// ============================================================================
// EMPTY CAVITY TRANSMISSION LINE (TE₁₀ Mode)
// ============================================================================
// Models a rectangular air-filled cavity (width a, height b, length L) as a
// two-port transmission line segment operating in the fundamental TE₁₀ mode.
// Used to represent sub-domains III and IV in the MNA equivalent circuit
// (Figure 3.7, Section 3.1.2.1).
//
// Physical Parameters:
//   a  — cavity broad dimension (width)  [m]
//   b  — cavity narrow dimension (height)[m]
//   L  — cavity length (propagation dir) [m]
//
// TE₁₀ Cutoff:
//   fc₁₀ = c / (2a)
//   Above fc: propagating mode  → kg real, positive
//   Below fc: evanescent mode   → kg purely imaginary (kg = jα, α > 0)
//
// ============================================================================
// KEY EQUATIONS (Russian Theory, Section 3.1.2.1)
// ============================================================================
//
// Propagation constant (Eq. 3.16):
//   kg = (2π/λ) · √(1 − (λ/2a)²)
//      = k₀ · √(1 − (λ/λc)²)          where λc = 2a
//
//   Evanescent continuation (below cutoff):
//   kg = j · k₀ · √((λ/λc)² − 1)      (purely imaginary)
//
// Waveguide characteristic impedance (Eq. 3.17):
//   Zg = ωμ₀ / kg = (2πf · μ₀) / kg
//   Above cutoff: Zg real, positive.
//   Below cutoff: kg = jα → Zg = −jωμ₀/α (purely imaginary, reactive).
//
// 2×2 Admittance matrix (Eqs. 3.14, 3.15):
//
//   Y₁₁ = Y₂₂ = 1 / (j·Zg·tan(kg·L))       (Eq. 3.14)
//   Y₁₂ = Y₂₁ = −1 / (Zg·sin(kg·L))         (Eq. 3.15)
//
//   Note on sign of Y₁₂:
//   The off-diagonal element carries a leading minus sign as given in
//   Eq. (3.15). This is consistent with the standard passive reciprocal
//   two-port TL Y-matrix and ensures Y₁₂ = Y₂₁ (symmetry). The matrix
//   written in full is:
//
//     Y = [ 1/(j·Zg·tan(β))    −1/(Zg·sin(β)) ]
//         [ −1/(Zg·sin(β))      1/(j·Zg·tan(β)) ]
//
//   where β = kg·L is the complex electrical length.
//
// ============================================================================

class TL_EmptyCavity : public BranchTemplate {
public:

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * @brief Create an air-filled rectangular cavity transmission line
     * @param node_from  Input port node index
     * @param node_to    Output port node index
     * @param branch_id  Unique branch identifier
     * @param a_m        Cavity broad dimension (width)  [m]
     * @param b_m        Cavity narrow dimension (height)[m]
     * @param L_m        Cavity length (propagation dir) [m]
     */
    TL_EmptyCavity(int node_from, int node_to, int branch_id,
                   double a_m, double b_m, double L_m)
        : BranchTemplate(BranchType::TL_EMPTY_CAVITY, node_from, node_to, branch_id)
        , m_a(a_m)
        , m_b(b_m)
        , m_L(L_m)
    {
        m_lambda_c = 2.0 * m_a;           // Cutoff wavelength λc = 2a [m]
        m_f_c10    = C_LIGHT / m_lambda_c; // Cutoff frequency fc = c/(2a) [Hz]
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION — Equations (3.14), (3.15), (3.16), (3.17)
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        using namespace std::complex_literals;

        // Step 1: Free-space propagation constant and waveguide impedance
        const double lambda = C_LIGHT / f_Hz;
        const double k0     = (2.0 * M_PI) / lambda;
        const double omega  = 2.0 * M_PI * f_Hz;
        const double ratio  = lambda / m_lambda_c;

        // Step 2: Propagation constant kg — Eq. (3.16)
        std::complex<double> kg;
        if (f_Hz >= m_f_c10) {
            kg = k0 * std::sqrt(1.0 - ratio * ratio);          // Propagating
        } else {
            kg = 1.0i * k0 * std::sqrt(ratio * ratio - 1.0);   // Evanescent
        }

        // Step 3: Waveguide impedance Zg — Eq. (3.17)
        const std::complex<double> Zg = (omega * MU_0) / kg;

        // Step 4: Delegate Y-matrix assembly to shared helper
        return assembleYMatrix(kg, Zg, m_L);
    }

protected:
    // ========================================================================
    // SHARED Y-MATRIX ASSEMBLY HELPER — Equations (3.14) and (3.15)
    // ========================================================================
    // Assembles the 2×2 transmission-line admittance matrix given a complex
    // propagation constant (kg_eff), a complex waveguide impedance (Zg), and
    // the physical line length L.
    //
    // This helper is declared protected so that derived classes (e.g.
    // TL_DielectricCavity) can call it directly after computing their own
    // modified kg and Zg, eliminating code duplication and ensuring that
    // any future fix to the Y-matrix stamp propagates to all subclasses.
    //
    // Matrix form (Eqs. 3.14, 3.15):
    //
    //   Y = [ Y₁₁   Y₁₂ ]   Y₁₁ = Y₂₂ =   1 / (j·Zg·tan(β))   Eq.(3.14)
    //       [ Y₂₁   Y₂₂ ]   Y₁₂ = Y₂₁ = − 1 / (Zg·sin(β))      Eq.(3.15)
    //
    //   β = kg_eff · L  (complex electrical length)
    //
    // Sign convention for Y₁₂:
    //   The leading minus in Eq. (3.15) is mandatory. It guarantees:
    //     (a) Symmetry:          Y₁₂ = Y₂₁
    //     (b) Passivity:         Re{Y₁₁} ≥ |Re{Y₁₂}|  (diagonal dominance)
    //     (c) Correct physics:   node voltages computed by MNA are consistent
    //                            with energy conservation in the waveguide.
    //
    // Singularity protection:
    //   tan(β) = 0 at β = nπ  → Y₁₁ → ∞  (short-circuit resonance)
    //   sin(β) = 0 at β = nπ  → Y₁₂ → ∞  (same resonances)
    //   Both are clamped to EPS (preserving sign) to prevent NaN propagation
    //   into the Eigen LU solver.

    static std::vector<std::vector<Complex>>
    assembleYMatrix(std::complex<double> kg_eff,
                    std::complex<double> Zg,
                    double L)
    {
        using namespace std::complex_literals;

        const std::complex<double> beta = kg_eff * L;

        std::complex<double> tan_beta = std::tan(beta);
        std::complex<double> sin_beta = std::sin(beta);

        constexpr double EPS = 1e-12;
        if (std::abs(tan_beta) < EPS) tan_beta = std::complex<double>(EPS, 0.0);
        if (std::abs(sin_beta) < EPS) sin_beta = std::complex<double>(EPS, 0.0);

        // Eqs. (3.14) and (3.15)
        const std::complex<double> Y11 =  1.0 / (1.0i * Zg * tan_beta);  // Eq.(3.14)
        const std::complex<double> Y12 = -1.0 / (Zg * sin_beta);          // Eq.(3.15) — minus sign
        const std::complex<double> Y21 =  Y12;                             // Symmetry
        const std::complex<double> Y22 =  Y11;                             // Eq.(3.14)

        return {
            { Complex(Y11.real(), Y11.imag()),  Complex(Y12.real(), Y12.imag()) },
            { Complex(Y21.real(), Y21.imag()),  Complex(Y22.real(), Y22.imag()) }
        };
    }

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {

        if (std::isnan(m_a) || std::isnan(m_b) || std::isnan(m_L)) {
            error_msg = "Cavity dimensions contain NaN";
            return false;
        }

        if (m_a <= 0.0 || m_b <= 0.0 || m_L <= 0.0) {
            error_msg = "Cavity dimensions must be strictly positive";
            return false;
        }

        // Physical range check: 0.1 mm – 10 m for lateral dimensions
        if (m_a < 1e-4 || m_a > 10.0) {
            error_msg = "Cavity width a out of range (0.1 mm – 10 m)";
            return false;
        }
        if (m_b < 1e-4 || m_b > 10.0) {
            error_msg = "Cavity height b out of range (0.1 mm – 10 m)";
            return false;
        }
        if (m_L < 1e-4 || m_L > 100.0) {
            error_msg = "Cavity length L out of range (0.1 mm – 100 m)";
            return false;
        }

        // Advisory: TE₁₀ mode assumes a ≥ b. Emit a warning to stderr
        // but do NOT set error_msg (caller checks non-empty error_msg as failure).
        if (m_a < m_b) {
            std::cerr << "[TL_EmptyCavity] WARNING: a (" << m_a * 1e3
                      << " mm) < b (" << m_b * 1e3
                      << " mm). For TE₁₀ dominance, the broad dimension a "
                      << "should be >= b.\n";
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "Empty Cavity TL: a=%.1fmm, b=%.1fmm, L=%.1fmm  "
                      "(fc10=%.3f GHz)",
                      m_a * 1e3, m_b * 1e3, m_L * 1e3,
                      m_f_c10 / 1e9);
        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    double getWidth()             const { return m_a;      }
    double getHeight()            const { return m_b;      }
    double getLength()            const { return m_L;      }
    double getCutoffFrequency()   const { return m_f_c10;  }
    double getCutoffWavelength()  const { return m_lambda_c; }

    bool isPropagating(double f_Hz) const { return f_Hz >= m_f_c10; }

    /** Guided wavelength above cutoff; returns 0.0 in evanescent regime. */
    double getGuidedWavelength(double f_Hz) const {
        if (f_Hz < m_f_c10) return 0.0;
        const double lambda_0 = C_LIGHT / f_Hz;
        const double ratio    = m_f_c10 / f_Hz;
        return lambda_0 / std::sqrt(1.0 - ratio * ratio);
    }

    /** Complex propagation constant kg at frequency f_Hz. */
    std::complex<double> getPropagationConstant(double f_Hz) const {
        using namespace std::complex_literals;
        const double lambda = C_LIGHT / f_Hz;
        const double k0     = (2.0 * M_PI) / lambda;
        const double ratio  = lambda / m_lambda_c;

        if (f_Hz >= m_f_c10) {
            return k0 * std::sqrt(1.0 - ratio * ratio);
        } else {
            return 1.0i * k0 * std::sqrt(ratio * ratio - 1.0);
        }
    }

    /** Waveguide characteristic impedance Zg at frequency f_Hz. */
    std::complex<double> getWaveguideImpedance(double f_Hz) const {
        const double omega = 2.0 * M_PI * f_Hz;
        return (omega * MU_0) / getPropagationConstant(f_Hz);
    }

private:
    double m_a;          // Broad dimension (width)   [m]
    double m_b;          // Narrow dimension (height)  [m]
    double m_L;          // Length (propagation dir)   [m]
    double m_f_c10;      // TE₁₀ cutoff frequency      [Hz]
    double m_lambda_c;   // Cutoff wavelength = 2a     [m]
};

} // namespace EMCore

#endif // TL_EMPTYCAVITY_H
