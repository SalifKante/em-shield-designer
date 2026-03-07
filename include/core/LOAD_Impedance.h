#ifndef LOAD_IMPEDANCE_H
#define LOAD_IMPEDANCE_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"
#include <cmath>
#include <iostream>

namespace EMCore {

// ============================================================================
// LOAD TERMINATION IMPEDANCE
// ============================================================================
// Represents an explicit termination impedance at an observation or
// measurement port in the MNA equivalent circuit.
//
// Physical Interpretation:
//   In the Russian theory (Figure 3.7, Eq. 3.18), the final TL segment
//   (branch IV) carries an implicit short-circuit termination: its far end
//   connects to ground, and the short-circuit boundary condition is absorbed
//   into the Y₂₂ diagonal of that segment. This class provides an explicit,
//   generalised load branch for arbitrary topologies where the termination
//   must be specified independently.
//
// Topology:
//   node_from ---[Z_L]--- node_to
//
//   Typical usage:
//     node_from = active observation node
//     node_to   = ground (node index 0)
//
// Y-Parameter Formulation:
//   Standard passive 2-port admittance stamp for a series element Z_L:
//
//     Y = [ +Y_L   −Y_L ]     Y_L = 1 / Z_L
//         [ −Y_L   +Y_L ]
//
//   The MNA framework projects this via A·Y·Aᵀ. When node_to = 0 (ground),
//   the projection correctly reduces to a shunt admittance Y_L at node_from.
//
// Supported Load Types:
//   - Resistive matched load:  Z_L = Z₀ = 120π Ω  (free-space impedance)
//   - RF measurement probe:    Z_L = 50 Ω
//   - Near open-circuit:       Z_L → very large  (Y_L → 0, numerically safe)
//   - Short circuit:           Z_L → 0  NOT supported via this class;
//                              use node removal / direct grounding instead,
//                              as Y_L → ∞ is numerically ill-conditioned.
//
// Frequency Dependence:
//   Z_L is treated as a frequency-independent constant. For reactive or
//   frequency-dependent terminations, derive from this class and override
//   computeYParameters(double f_Hz).
// ============================================================================

class LOAD_Impedance : public BranchTemplate {
public:

    // ========================================================================
    // CONSTRUCTORS
    // ========================================================================

    /**
     * @brief Construct a load with a complex impedance
     * @param node_from  Active observation node
     * @param node_to    Termination node (typically 0 = ground)
     * @param branch_id  Unique branch identifier
     * @param Z_load     Complex load impedance [Ω]
     */
    LOAD_Impedance(int node_from, int node_to, int branch_id,
                   Complex Z_load)
        : BranchTemplate(BranchType::LOAD_IMPEDANCE, node_from, node_to, branch_id)
        , m_Z_L(Z_load)
    {}

    /**
     * @brief Construct a load with a real (resistive) impedance
     * @param node_from     Active observation node
     * @param node_to       Termination node (typically 0 = ground)
     * @param branch_id     Unique branch identifier
     * @param Z_load_real   Real load impedance [Ω]
     */
    LOAD_Impedance(int node_from, int node_to, int branch_id,
                   double Z_load_real)
        : BranchTemplate(BranchType::LOAD_IMPEDANCE, node_from, node_to, branch_id)
        , m_Z_L(Z_load_real, 0.0)
    {}

    // ========================================================================
    // Y-PARAMETER COMPUTATION
    // ========================================================================
    //
    // Standard passive 2-port admittance stamp for a series element Z_L:
    //
    //   Terminal 1 (node_from) ──[Z_L]── Terminal 2 (node_to)
    //
    //   KCL at terminal 1:  I₁ = (V₁ − V₂) / Z_L = +Y_L·V₁ − Y_L·V₂
    //   KCL at terminal 2:  I₂ = (V₂ − V₁) / Z_L = −Y_L·V₁ + Y_L·V₂
    //
    //   Y-matrix:  [ +Y_L  −Y_L ]     Y_L = 1/Z_L
    //              [ −Y_L  +Y_L ]
    //
    // This stamp is identical in form to all other passive 2-port branches
    // (AP_SlotAperture, SRC_VoltageSource). The incidence matrix A in the
    // MNA assembly handles grounding automatically.
    //
    // Note: f_Hz is accepted but not used. Z_L is frequency-independent.

    std::vector<std::vector<Complex>> computeYParameters(double /*f_Hz*/) const override {
        const Complex Y_L = 1.0 / m_Z_L;
        return {
            { Y_L,  -Y_L },
            {-Y_L,   Y_L }
        };
    }

    // Base class getVoltageSourceVector() returns {0, 0} — correct for
    // passive element. No override needed.

    // ========================================================================
    // VALIDATION
    // ========================================================================
    //
    // Contract: error_msg is set ONLY when returning false.
    //           error_msg is left empty when returning true.
    //           Advisory conditions are routed to std::cerr only.

    bool isValid(std::string& error_msg) const override {

        // --- Hard failure: NaN or infinity ---
        if (std::isnan(m_Z_L.real()) || std::isnan(m_Z_L.imag()) ||
            std::isinf(m_Z_L.real()) || std::isinf(m_Z_L.imag())) {
            error_msg = "Load impedance is NaN or infinity";
            return false;
        }

        // --- Hard failure: near short-circuit ---
        // Z_L ≈ 0 → Y_L → ∞, which makes the MNA matrix singular.
        // Short-circuit terminations must be handled by direct node grounding,
        // not by passing an unbounded admittance to the Eigen solver.
        if (std::abs(m_Z_L) < 1e-6) {
            error_msg = "Load impedance too small (|Z_L| < 1 µΩ). "
                        "Short-circuit terminations must be implemented via "
                        "node removal, not via this branch.";
            return false;
        }

        // --- Advisory: near open-circuit ---
        // Z_L → ∞ → Y_L → 0. This is numerically safe (the branch contributes
        // negligibly to the nodal admittance matrix) and physically meaningful
        // as an open-circuit or high-impedance probe. No hard error is raised.
        if (std::abs(m_Z_L) > 1e6) {
            std::cerr << "[LOAD_Impedance] INFO: |Z_L| = "
                      << std::abs(m_Z_L)
                      << " Ω. Load is approaching open-circuit (Y_L ≈ 0). "
                      << "Contribution to nodal admittance matrix is negligible.\n";
        }

        return true;   // error_msg intentionally empty on success
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "Load Impedance: Z_L = (%.4f %+.4fj) Ω  "
                      "|Z_L| = %.4f Ω  ∠%.2f°",
                      m_Z_L.real(),
                      m_Z_L.imag(),
                      std::abs(m_Z_L),
                      std::arg(m_Z_L) * 180.0 / M_PI);
        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    Complex getImpedance()   const { return m_Z_L; }
    double  getMagnitude()   const { return std::abs(m_Z_L);              }
    double  getPhase()       const { return std::arg(m_Z_L);              }  // [rad]
    double  getPhaseDeg()    const { return std::arg(m_Z_L) * 180.0 / M_PI; }
    double  getReal()        const { return m_Z_L.real();                 }
    double  getImaginary()   const { return m_Z_L.imag();                 }

    void setImpedance(Complex Z)       { m_Z_L = Z;                  }
    void setImpedance(double Z_real)   { m_Z_L = Complex(Z_real, 0.0); }

private:
    Complex m_Z_L;   // Load impedance [Ω]
};

} // namespace EMCore

#endif // LOAD_IMPEDANCE_H
