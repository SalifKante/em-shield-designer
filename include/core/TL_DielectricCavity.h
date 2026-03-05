#ifndef TL_DIELECTRICCAVITY_H
#define TL_DIELECTRICCAVITY_H

#include "TL_EmptyCavity.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace EMCore {

// ============================================================================
// DIELECTRIC-FILLED CAVITY TRANSMISSION LINE (TE₁₀ Mode)
// ============================================================================
// Models a rectangular cavity partially filled with a dielectric layer as a
// two-port transmission line. Implements Section 3.1.2.2 of the Russian theory
// (Equations 3.19 – 3.21).
//
// Physical Configuration (Figure 3.8):
//   - Cavity cross-section: width a [m], height b [m]
//   - Cavity length:        L [m]
//   - Dielectric layer:     thickness h [m] (h ≤ b), permittivity εᵣ
//   - Air gap:              thickness (b − h) [m]
//
// ============================================================================
// KEY EQUATIONS (Russian Theory, Section 3.1.2.2)
// ============================================================================
//
// Modified propagation constant (Eq. 3.19):
//
//   kg = (2π√εeff / λ) · √[ 1 − (λ / (2a√εeff))² ]
//
//   This is the empty-cavity formula (Eq. 3.16) with:
//     k₀  → k₀ · √εeff          (phase velocity reduced by √εeff)
//     λc  → λc · √εeff = 2a√εeff (cutoff wavelength increases)
//
//   Evanescent continuation (below modified cutoff):
//     kg = j · k₀√εeff · √[ (λ/λc_eff)² − 1 ]
//
// Effective permittivity — two models available:
//
//   Maxwell-Garnett (Eq. 3.20):
//     εeff = [ 2·(h/b)·(εᵣ−1) + (εᵣ+2) ] / [ (εᵣ+2) − (h/b)·(εᵣ−1) ]
//
//   Lichtenecker logarithmic mixture (Eq. 3.21):
//     lg(εeff) = (h/b)·lg(εᵣ) + (1−h/b)·lg(1)  →  εeff = εᵣ^(h/b)
//
// Waveguide impedance (Eq. 3.17 with modified kg):
//   Zg = ωμ₀ / kg
//
// Y-matrix (Eqs. 3.14, 3.15 — identical structure, kg and Zg modified):
//   Y₁₁ = Y₂₂ =   1 / (j·Zg·tan(kg·L))    Eq.(3.14)
//   Y₁₂ = Y₂₁ = − 1 / (Zg·sin(kg·L))      Eq.(3.15)
//
// Y-matrix assembly is delegated to TL_EmptyCavity::assembleYMatrix()
// to ensure a single, validated implementation of Eqs. (3.14)–(3.15).
// ============================================================================

class TL_DielectricCavity : public TL_EmptyCavity {
public:

    // ========================================================================
    // EFFECTIVE PERMITTIVITY METHOD SELECTOR
    // ========================================================================

    enum class EffPermMethod {
        MAXWELL_GARNETT,   ///< Eq. (3.20) — preferred for volume inclusion model
        LICHTENECKER       ///< Eq. (3.21) — preferred for thin layers (default)
    };

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * @brief Create a partially dielectric-filled rectangular cavity
     *
     * @param node_from   Input port node index
     * @param node_to     Output port node index
     * @param branch_id   Unique branch identifier
     * @param a_m         Cavity broad dimension (width)  [m]
     * @param b_m         Cavity narrow dimension (height)[m]
     * @param L_m         Cavity length (propagation dir) [m]
     * @param h_m         Dielectric layer thickness [m]  (must satisfy 0 < h ≤ b)
     * @param epsilon_r   Relative permittivity of dielectric (must be ≥ 1.0)
     * @param method      Effective permittivity formula selector
     *                    (default: Lichtenecker)
     */
    TL_DielectricCavity(int node_from, int node_to, int branch_id,
                        double a_m, double b_m, double L_m,
                        double h_m, double epsilon_r,
                        EffPermMethod method = EffPermMethod::LICHTENECKER)
        : TL_EmptyCavity(node_from, node_to, branch_id, a_m, b_m, L_m)
        , m_h(h_m)
        , m_epsilon_r(epsilon_r)
        , m_method(method)
        , m_epsilon_eff(1.0)
        , m_fc10_dielectric(0.0)
    {
        if (m_h <= 0.0) {
            throw std::invalid_argument(
                "TL_DielectricCavity: dielectric thickness h must be positive");
        }
        if (m_h > b_m) {
            throw std::invalid_argument(
                "TL_DielectricCavity: dielectric thickness h cannot exceed cavity height b");
        }
        if (m_epsilon_r < 1.0) {
            throw std::invalid_argument(
                "TL_DielectricCavity: relative permittivity must be >= 1.0");
        }

        // Compute effective permittivity and modified cutoff frequency
        m_epsilon_eff      = calculateEffectivePermittivity();
        m_fc10_dielectric  = C_LIGHT / (2.0 * a_m * std::sqrt(m_epsilon_eff));
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION — Equations (3.19), (3.17), (3.14), (3.15)
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        using namespace std::complex_literals;

        // ----------------------------------------------------------------
        // Step 1: Free-space quantities
        // ----------------------------------------------------------------
        const double lambda     = C_LIGHT / f_Hz;           // λ = c/f [m]
        const double k0         = (2.0 * M_PI) / lambda;    // k₀ = 2π/λ [rad/m]
        const double omega      = 2.0 * M_PI * f_Hz;        // ω = 2πf [rad/s]

        // ----------------------------------------------------------------
        // Step 2: Modified propagation constant — Eq. (3.19)
        //
        //   sqrt_eff  = √εeff
        //   λc_eff    = 2a · √εeff     (modified cutoff wavelength)
        //   ratio     = λ / λc_eff
        //
        //   Above modified cutoff: kg = k₀·√εeff · √(1 − ratio²)   [real]
        //   Below modified cutoff: kg = j·k₀·√εeff · √(ratio²−1)   [imaginary]
        // ----------------------------------------------------------------
        const double sqrt_eff   = std::sqrt(m_epsilon_eff);
        const double lambda_c_eff = 2.0 * getWidth() * sqrt_eff;
        const double ratio      = lambda / lambda_c_eff;

        std::complex<double> kg;
        if (ratio < 1.0) {
            // Propagating mode (above modified cutoff)
            kg = k0 * sqrt_eff * std::sqrt(1.0 - ratio * ratio);
        } else {
            // Evanescent mode (below modified cutoff)
            kg = 1.0i * k0 * sqrt_eff * std::sqrt(ratio * ratio - 1.0);
        }

        // ----------------------------------------------------------------
        // Step 3: Waveguide impedance — Eq. (3.17) with modified kg
        //
        //   Zg = ωμ₀ / kg
        // ----------------------------------------------------------------
        const std::complex<double> Zg = (omega * MU_0) / kg;

        // ----------------------------------------------------------------
        // Step 4: Assemble Y-matrix via shared parent helper
        //
        //   Delegates to TL_EmptyCavity::assembleYMatrix(kg, Zg, L) which
        //   implements Eqs. (3.14) and (3.15) with the correct Y₁₂ sign.
        //   This guarantees a single, validated implementation of the
        //   Y-matrix stamp for both air-filled and dielectric-filled cavities.
        // ----------------------------------------------------------------
        return assembleYMatrix(kg, Zg, getLength());
    }

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {

        // Check base class (geometry ranges, a ≥ b advisory, NaN checks)
        if (!TL_EmptyCavity::isValid(error_msg)) {
            return false;
        }

        if (m_h <= 0.0) {
            error_msg = "Dielectric thickness h must be positive";
            return false;
        }
        if (m_h > getHeight()) {
            error_msg = "Dielectric thickness h cannot exceed cavity height b";
            return false;
        }
        if (m_epsilon_r < 1.0) {
            error_msg = "Relative permittivity must be >= 1.0";
            return false;
        }
        if (std::isnan(m_epsilon_eff) || std::isinf(m_epsilon_eff)
            || m_epsilon_eff < 1.0) {
            error_msg = "Effective permittivity calculation produced an invalid result";
            return false;
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        const char* method_str =
            (m_method == EffPermMethod::MAXWELL_GARNETT) ? "Maxwell-Garnett"
                                                         : "Lichtenecker";
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "Dielectric Cavity TL: a=%.1fmm, b=%.1fmm, L=%.1fmm, "
                      "h=%.1fmm (h/b=%.3f), er=%.3f → eeff=%.4f  "
                      "[%s, fc_eff=%.3f GHz]",
                      getWidth()  * 1e3,
                      getHeight() * 1e3,
                      getLength() * 1e3,
                      m_h * 1e3,
                      getFillFactor(),
                      m_epsilon_r,
                      m_epsilon_eff,
                      method_str,
                      m_fc10_dielectric / 1e9);
        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    double       getDielectricThickness()      const { return m_h;               }
    double       getRelativePermittivity()     const { return m_epsilon_r;       }
    double       getEffectivePermittivity()    const { return m_epsilon_eff;     }
    EffPermMethod getMethod()                  const { return m_method;          }
    double       getDielectricCutoffFreq()     const { return m_fc10_dielectric; }

    /// Fill factor h/b
    double getFillFactor()     const { return m_h / getHeight(); }

    /// True when h < b (partial fill)
    bool isPartiallyFilled()   const { return m_h < getHeight() - 1e-12; }

    /// True when h ≈ b (complete fill)
    bool isCompletelyFilled()  const { return std::abs(m_h - getHeight()) < 1e-12; }

private:
    double        m_h;                 // Dielectric layer thickness [m]
    double        m_epsilon_r;         // Relative permittivity of dielectric
    EffPermMethod m_method;            // Formula selector
    double        m_epsilon_eff;       // Computed effective permittivity
    double        m_fc10_dielectric;   // Modified cutoff frequency [Hz]

    // ========================================================================
    // EFFECTIVE PERMITTIVITY DISPATCH
    // ========================================================================

    double calculateEffectivePermittivity() const {
        const double h_b = m_h / getHeight();   // Fill factor h/b ∈ (0, 1]
        switch (m_method) {
        case EffPermMethod::MAXWELL_GARNETT: return calculateMaxwellGarnett(h_b);
        case EffPermMethod::LICHTENECKER:    return calculateLichtenecker(h_b);
        default:                             return calculateLichtenecker(h_b);
        }
    }

    // ========================================================================
    // MAXWELL-GARNETT FORMULA — Equation (3.20)
    // ========================================================================
    //
    //   εeff = [ 2·(h/b)·(εᵣ−1) + (εᵣ+2) ]
    //          ─────────────────────────────
    //          [ (εᵣ+2) − (h/b)·(εᵣ−1)   ]
    //
    // Boundary verification:
    //   h/b = 0 → εeff = (εᵣ+2)/(εᵣ+2)              = 1.0  ✓
    //   h/b = 1 → εeff = (2εᵣ−2+εᵣ+2)/(εᵣ+2−εᵣ+1) = εᵣ   ✓
    //
    // Singularity: denominator = 0 iff h/b = (εᵣ+2)/(εᵣ−1).
    //   For εᵣ ≥ 1, this ratio is always > 1, so it is unreachable when
    //   h ≤ b. Guard retained for floating-point safety only.

    double calculateMaxwellGarnett(double h_b) const {
        const double num = 2.0 * h_b * (m_epsilon_r - 1.0) + (m_epsilon_r + 2.0);
        const double den = (m_epsilon_r + 2.0) - h_b * (m_epsilon_r - 1.0);

        if (std::abs(den) < 1e-10) {
            std::cerr << "[TL_DielectricCavity] WARNING: Maxwell-Garnett "
                      << "denominator near zero. Falling back to Lichtenecker.\n";
            return calculateLichtenecker(h_b);
        }

        const double eps = num / den;

        // Physical bounds: 1 ≤ εeff ≤ εᵣ
        return std::max(1.0, std::min(m_epsilon_r, eps));
    }

    // ========================================================================
    // LICHTENECKER LOGARITHMIC MIXTURE — Equation (3.21)
    // ========================================================================
    //
    //   lg(εeff) = (h/b)·lg(εᵣ) + (1−h/b)·lg(1)
    //
    //   Since lg(1) = 0:
    //   εeff = εᵣ^(h/b)
    //
    // Boundary verification:
    //   h/b = 0 → εeff = εᵣ⁰ = 1.0  ✓
    //   h/b = 1 → εeff = εᵣ¹ = εᵣ   ✓

    double calculateLichtenecker(double h_b) const {
        return std::pow(m_epsilon_r, h_b);
    }
};

} // namespace EMCore

#endif // TL_DIELECTRICCAVITY_H
