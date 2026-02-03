#ifndef LOAD_IMPEDANCE_H
#define LOAD_IMPEDANCE_H

#include "BranchTemplate.h"

namespace EMCore {
    // ============================================================================
    // LOAD TERMINATION
    // ============================================================================
    // Represents a termination impedance at the end of the cavity chain.
    //
    // In your Russian theory:
    //   - The load is implicitly included in the Y-matrix
    //   - Y22 of last TL segment is modified: Y22_III - Y11_IV
    //   - This represents the load impedance effect
    //
    // For standalone load branch:
    //   Y-parameters for shunt element (connected to ground):
    //   Y11 = 1/Z_L  (shunt admittance to ground)
    //   Y12 = Y21 = Y22 = 0
    //
    // Special cases:
    //   - Matched load: Z_L = Zg (waveguide impedance)
    //   - Open circuit: Z_L → ∞ (Y11 → 0)
    //   - Short circuit: Z_L → 0 (Y11 → ∞)
    // ============================================================================

    class LOAD_Impedance : public BranchTemplate {
    public:
        // ========================================================================
        // CONSTRUCTOR
        // ========================================================================
        LOAD_Impedance(int node_from, int node_to, int branch_id,
                       Complex Z_load)
            : BranchTemplate(BranchType::LOAD_IMPEDANCE, node_from, node_to, branch_id)
            , m_Z_L(Z_load)
        {}

        // Convenience constructor for real impedance
        LOAD_Impedance(int node_from, int node_to, int branch_id,
                       double Z_load_real)
            : BranchTemplate(BranchType::LOAD_IMPEDANCE, node_from, node_to, branch_id)
            , m_Z_L(Z_load_real, 0.0)
        {}

        // ========================================================================
        // Y-PARAMETER COMPUTATION
        // ========================================================================
        std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
            // Shunt element to ground: Y = 1/Z_L
            Complex Y_L = 1.0 / m_Z_L;

            // Y-matrix for shunt element:
            //   Current flows from node_from to ground
            //   Y11 = Y_L, all others = 0
            return {{Y_L, Complex(0.0, 0.0)},
                    {Complex(0.0, 0.0), Complex(0.0, 0.0)}};
        }

        // ========================================================================
        // VALIDATION
        // ========================================================================
        bool isValid(std::string& error_msg) const override {
            if (std::abs(m_Z_L) < 1e-6) {
                error_msg = "Load impedance too small (short circuit)";
                return false;
            }

            if (std::abs(m_Z_L) > 1e6) {
                error_msg = "Warning: Very large load impedance (approaching open circuit)";
                // Not necessarily an error
            }

            return true;
        }

        // ========================================================================
        // DESCRIPTION
        // ========================================================================
        std::string getDescription() const override {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "Load: Z_L = %.1f%+.1fj Ohm",
                          m_Z_L.real(), m_Z_L.imag());
            return std::string(buf);
        }

        // ========================================================================
        // ACCESSORS
        // ========================================================================
        Complex getImpedance() const { return m_Z_L; }
        void setImpedance(Complex Z) { m_Z_L = Z; }

        private:
            Complex m_Z_L;  // Load impedance [Ohm]

    };
}


#endif // LOAD_IMPEDANCE_H
