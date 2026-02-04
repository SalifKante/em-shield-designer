#ifndef LOAD_IMPEDANCE_H
#define LOAD_IMPEDANCE_H

#include "BranchTemplate.h"

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
//   For a shunt element (one terminal to node, other to ground):
//     Y11 = 1/Z_L  (admittance to ground)
//     Y12 = Y21 = Y22 = 0  (no coupling, other terminal is ground)
//
// Common Load Types:
//   - Matched load: Z_L = Zg (waveguide impedance, typically ~377 Ω)
//   - Open circuit: Z_L → ∞ (Y11 → 0)
//   - Short circuit: Z_L → 0 (Y11 → ∞)
//   - Measurement probe: Z_L = 50 Ω (standard RF impedance)
//
// Topology:
//   node_from ---[Z_L]--- ground (node_to should be 0)
//
// Note: In Russian theory, load is sometimes implicit in final TL segment.
//       This class provides explicit load modeling for clarity.
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
    {
        // Validate that node_to is ground (0)
        if (node_to != 0) {
            std::cerr << "WARNING: Load impedance typically connects to ground (node 0), "
                      << "but node_to = " << node_to << "\n";
        }
    }

    // Real impedance constructor (convenience)
    LOAD_Impedance(int node_from, int node_to, int branch_id,
                   double Z_load_real)
        : BranchTemplate(BranchType::LOAD_IMPEDANCE, node_from, node_to, branch_id)
        , m_Z_L(Z_load_real, 0.0)
    {
        if (node_to != 0) {
            std::cerr << "WARNING: Load impedance typically connects to ground (node 0), "
                      << "but node_to = " << node_to << "\n";
        }
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        // Shunt element to ground: Y = 1/Z_L
        //
        // Circuit model:
        //   Terminal 1 (node_from) ---[Z_L]--- Terminal 2 (ground)
        //
        // Y-parameters for this configuration:
        //   I1 = Y11*V1 + Y12*V2
        //   I2 = Y21*V1 + Y22*V2
        //
        // Since V2 = 0 (ground), and I1 = V1/Z_L:
        //   Y11 = 1/Z_L
        //   Y12 = 0 (no current flows to ground terminal from V2)
        //   Y21 = 0 (symmetry)
        //   Y22 = 0 (ground terminal)

        Complex Y_L = 1.0 / m_Z_L;

        return {
            { Y_L,                  Complex(0.0, 0.0) },  // [Y11, Y12]
            { Complex(0.0, 0.0),    Complex(0.0, 0.0) }   // [Y21, Y22]
        };
    }

    // ========================================================================
    // VOLTAGE SOURCE VECTOR (PASSIVE ELEMENT)
    // ========================================================================

    std::vector<Complex> getVoltageSourceVector(double f_Hz) const override {
        // Load is passive - no voltage source
        return {Complex(0.0, 0.0), Complex(0.0, 0.0)};
    }

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {
        double Z_magnitude = std::abs(m_Z_L);

        // Check for short circuit (very small impedance)
        if (Z_magnitude < 1e-6) {
            error_msg = "Load impedance too small (≈ short circuit, |Z| < 1 µΩ)";
            return false;
        }

        // Check for very large impedance (approaching open circuit)
        if (Z_magnitude > 1e9) {
            error_msg = "Load impedance too large (≈ open circuit, |Z| > 1 GΩ)";
            return false;
        }

        // Warn about unusual impedance values
        if (Z_magnitude > 1e6) {
            error_msg = "WARNING: Very large load impedance (|Z| > 1 MΩ), approaching open circuit";
            // Not an error, just a warning
        }

        // Check for NaN or infinity
        if (std::isnan(m_Z_L.real()) || std::isnan(m_Z_L.imag()) ||
            std::isinf(m_Z_L.real()) || std::isinf(m_Z_L.imag())) {
            error_msg = "Load impedance is NaN or infinity";
            return false;
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        char buf[256];

        // Format impedance
        double Z_mag = std::abs(m_Z_L);
        double Z_phase = std::arg(m_Z_L) * 180.0 / M_PI;

        std::snprintf(buf, sizeof(buf),
                      "Load: Z_L = (%.3f%+.3fj) Ω = %.3f∠%.1f° Ω",
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
