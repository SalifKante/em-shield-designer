#ifndef LOAD_IMPEDANCE_H
#define LOAD_IMPEDANCE_H

#include "BranchTemplate.h"
#include "PhysicsConstants.h"

namespace EMCore {

// ============================================================================
// LOAD TERMINATION
// ============================================================================
// Represents a termination impedance at the end of the cavity chain.
//
// Physical Interpretation:
//   - Models the load impedance at the observation/measurement port
//   - Connected as a SHUNT element between a node and ground
//   - Typical usage: Connect to final observation node
//
// Y-Parameter Formulation:
//   Standard 2-port shunt element (same form as aperture):
//     Y-matrix = [[ Y_L, -Y_L ],
//                 [-Y_L,  Y_L ]]
//   where Y_L = 1/Z_L
//
//   This is the CORRECT form for MNA assembly via A·Y·A^T.
//   The incidence matrix A handles the grounding — each branch
//   must provide a complete 2×2 Y-matrix regardless of topology.
//
// Common Load Types:
//   - Matched load: Z_L = Zg (waveguide impedance, typically ~377 Ω)
//   - Open circuit: Z_L → ∞ (Y_L → 0)
//   - Short circuit: Z_L → 0 (Y_L → ∞)
//   - Measurement probe: Z_L = 50 Ω (standard RF impedance)
//
// Topology:
//   node_from ---[Z_L]--- node_to (typically ground = 0)
//
// Note: In Russian theory (Figure 3.7, Eq 3.18), the load is implicit
//       in the final TL segment's short-circuit termination. This class
//       provides explicit load modeling for generalized topologies.
// ============================================================================

class LOAD_Impedance : public BranchTemplate {
public:
    // ========================================================================
    // CONSTRUCTORS
    // ========================================================================

    // Complex impedance constructor
    LOAD_Impedance(int node_from, int node_to, int branch_id,
                   Complex Z_load)
        : BranchTemplate(BranchType::LOAD_IMPEDANCE, node_from, node_to, branch_id)
        , m_Z_L(Z_load)
    {}

    // Real impedance constructor (convenience)
    LOAD_Impedance(int node_from, int node_to, int branch_id,
                   double Z_load_real)
        : BranchTemplate(BranchType::LOAD_IMPEDANCE, node_from, node_to, branch_id)
        , m_Z_L(Z_load_real, 0.0)
    {}

    // ========================================================================
    // Y-PARAMETER COMPUTATION
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        // Standard 2-port shunt element Y-matrix:
        //
        // Circuit model:
        //   Terminal 1 (node_from) ---[Z_L]--- Terminal 2 (node_to)
        //
        // Current relationship:
        //   I1 = -I2 = (V1 - V2) / Z_L
        //
        // Y-parameter form:
        //   I1 = Y_L*V1 + (-Y_L)*V2
        //   I2 = (-Y_L)*V1 + Y_L*V2
        //
        // This is the SAME form as any 2-terminal shunt element (e.g., aperture).
        // The MNA framework requires this complete 2×2 matrix so that
        // A·Y·A^T assembles correctly for any topology.
        //
        // When node_to = ground (0), the incidence matrix A effectively
        // reduces this to Y11_effective = Y_L at node_from, which is the
        // physically expected result.

        Complex Y_L = 1.0 / m_Z_L;

        return {
            { Y_L,  -Y_L },   // [Y11, Y12]
            {-Y_L,   Y_L }    // [Y21, Y22]
        };
    }

    // Base class getVoltageSourceVector() returns {0, 0} — correct for passive load.

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {
        double Z_magnitude = std::abs(m_Z_L);

        // Check for short circuit (very small impedance)
        if (Z_magnitude < 1e-6) {
            error_msg = "Load impedance too small (short circuit, |Z| < 1 uOhm)";
            return false;
        }

        // Check for very large impedance (approaching open circuit)
        if (Z_magnitude > 1e9) {
            error_msg = "Load impedance too large (open circuit, |Z| > 1 GOhm)";
            return false;
        }

        // Check for NaN or infinity
        if (std::isnan(m_Z_L.real()) || std::isnan(m_Z_L.imag()) ||
            std::isinf(m_Z_L.real()) || std::isinf(m_Z_L.imag())) {
            error_msg = "Load impedance is NaN or infinity";
            return false;
        }

        // Warning for very large impedance (not an error)
        if (Z_magnitude > 1e6) {
            error_msg = "Warning: Very large load impedance (|Z| > 1 MOhm), approaching open circuit";
            // Return true — this is advisory only
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        char buf[256];

        double Z_mag = std::abs(m_Z_L);
        double Z_phase = std::arg(m_Z_L) * 180.0 / M_PI;

        std::snprintf(buf, sizeof(buf),
                      "Load: Z_L = (%.3f%+.3fj) Ohm = %.3f at %.1f deg Ohm",
                      m_Z_L.real(), m_Z_L.imag(),
                      Z_mag, Z_phase);

        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    Complex getImpedance() const { return m_Z_L; }
    void setImpedance(Complex Z) { m_Z_L = Z; }

    // Convenience methods
    double getMagnitude() const { return std::abs(m_Z_L); }
    double getPhase() const { return std::arg(m_Z_L); }
    double getReal() const { return m_Z_L.real(); }
    double getImaginary() const { return m_Z_L.imag(); }

private:
    Complex m_Z_L;  // Load impedance [Ω]
};

} // namespace EMCore

#endif // LOAD_IMPEDANCE_H
