#ifndef SRC_VOLTAGESOURCE_H
#define SRC_VOLTAGESOURCE_H

#include "BranchTemplate.h"
#include <iostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace EMCore {

// ============================================================================
// VOLTAGE SOURCE WITH SERIES IMPEDANCE
// ============================================================================
// Models an incident electromagnetic field source with internal impedance.
//
// Physical Model:
//   - Represents incident EM field as an equivalent voltage source
//   - Series impedance Zs models source internal resistance
//   - For plane wave: Zs = Z0 = 120π Ω (free-space impedance)
//
// Circuit Topology:
//   node_from (GND) ---[Zs]---(+V_inc)--- node_to (active)
//
// Y-Parameter Formulation:
//   Series impedance between two terminals:
//     Y11 = Y22 = 1/Zs
//     Y12 = Y21 = -1/Zs
//
// Voltage Source Convention (Standard MNA):
//   V_vec = [0, V_inc] means voltage V_inc appears at terminal 2
//   Terminal 1 is reference (ground), terminal 2 is active
//
// Usage:
//   Typically connects ground (node 0) to first active node (node 1)
//   Example: SRC_VoltageSource(0, 1, 0, Complex(1.0, 0.0), 377.0)
// ============================================================================

class SRC_VoltageSource : public BranchTemplate {
public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * @brief Create a voltage source with series impedance
     * @param node_from Starting node (typically 0 = ground)
     * @param node_to Ending node (first active node)
     * @param branch_id Unique branch identifier
     * @param V_incident Incident voltage phasor [V] (default: 1+0j)
     * @param Z_source Source impedance [Ω] (default: 120π ≈ 377 Ω)
     */
    SRC_VoltageSource(int node_from, int node_to, int branch_id,
                      Complex V_incident = Complex(1.0, 0.0),
                      double Z_source = 120.0 * M_PI)
        : BranchTemplate(BranchType::SRC_VOLTAGE, node_from, node_to, branch_id)
        , m_V_inc(V_incident)
        , m_Zs(Z_source)
    {
        // Validate construction parameters
        if (node_from != 0) {
            std::cerr << "WARNING: Voltage source typically starts from ground (node 0), "
                      << "but node_from = " << node_from << "\n";
        }
    }

    // ========================================================================
    // Y-PARAMETER COMPUTATION
    // ========================================================================

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        // Y-parameters for series impedance Zs:
        //
        // Circuit model:
        //   Terminal 1 ---[Zs]--- Terminal 2
        //
        // Admittance relationships:
        //   I1 = -I2 = (V1 - V2) / Zs
        //
        // Y-parameter form:
        //   I1 = Y11*V1 + Y12*V2
        //   I2 = Y21*V1 + Y22*V2
        //
        // Solving:
        //   Y11 = Y22 = 1/Zs
        //   Y12 = Y21 = -1/Zs
        //
        // Physical interpretation:
        //   - Y11, Y22: Self-admittance at each terminal
        //   - Y12, Y21: Negative mutual admittance (series connection)

        Complex Y_s = 1.0 / m_Zs;

        return {
            { Y_s,  -Y_s },   // [Y11, Y12]
            {-Y_s,   Y_s }    // [Y21, Y22]
        };
    }

    // ========================================================================
    // VOLTAGE SOURCE VECTOR
    // ========================================================================

    std::vector<Complex> getVoltageSourceVector(double f_Hz) const override {
        // STANDARD MNA CONVENTION:
        //
        // The voltage source V_inc appears between terminals such that:
        //   V_terminal2 - V_terminal1 = V_inc
        //
        // Branch topology: Node 0 (terminal 1) → Node 1 (terminal 2)
        //   Terminal 1: Connected to ground (node 0), so V_terminal1 = 0
        //   Terminal 2: Connected to active node, receives voltage V_inc
        //
        // Therefore: V_vec = [0, V_inc]
        //            where V_vec[0] = voltage at terminal 1 (ground)
        //                  V_vec[1] = voltage at terminal 2 (active)
        //
        // This ensures: V_node1 - V_node0 = V_inc

        return {
            Complex(0.0, 0.0),   // Terminal 1 (ground reference)
            m_V_inc              // Terminal 2 (incident voltage)
        };
    }

    // ========================================================================
    // VALIDATION
    // ========================================================================

    bool isValid(std::string& error_msg) const override {
        // Check source impedance
        if (std::abs(m_Zs) < 1e-6) {
            error_msg = "Source impedance too small (|Zs| < 1 µΩ)";
            return false;
        }

        if (std::abs(m_Zs) > 1e6) {
            error_msg = "Source impedance too large (|Zs| > 1 MΩ)";
            return false;
        }

        // Check for NaN or infinity
        if (std::isnan(m_Zs) || std::isinf(m_Zs)) {
            error_msg = "Source impedance is NaN or infinity";
            return false;
        }

        if (std::isnan(m_V_inc.real()) || std::isnan(m_V_inc.imag()) ||
            std::isinf(m_V_inc.real()) || std::isinf(m_V_inc.imag())) {
            error_msg = "Source voltage is NaN or infinity";
            return false;
        }

        return true;
    }

    // ========================================================================
    // DESCRIPTION
    // ========================================================================

    std::string getDescription() const override {
        char buf[256];

        double V_mag = std::abs(m_V_inc);
        double V_phase = std::arg(m_V_inc) * 180.0 / M_PI;

        std::snprintf(buf, sizeof(buf),
                      "Voltage Source: V_inc=%.3f∠%.1f°V, Zs=%.1f Ω",
                      V_mag, V_phase, m_Zs);

        return std::string(buf);
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    // Getters
    Complex getVoltage() const { return m_V_inc; }
    double getSourceImpedance() const { return m_Zs; }

    // Voltage magnitude and phase
    double getVoltageMagnitude() const { return std::abs(m_V_inc); }
    double getVoltagePhase() const { return std::arg(m_V_inc); }  // radians
    double getVoltagePhaseDegrees() const { return std::arg(m_V_inc) * 180.0 / M_PI; }

    // Setters
    void setVoltage(Complex V) { m_V_inc = V; }
    void setVoltage(double magnitude, double phase_rad) {
        m_V_inc = std::polar(magnitude, phase_rad);
    }
    void setSourceImpedance(double Zs) { m_Zs = Zs; }

private:
    Complex m_V_inc;  // Incident voltage phasor [V]
    double m_Zs;      // Source impedance [Ω]
};

} // namespace EMCore

#endif // SRC_VOLTAGESOURCE_H
