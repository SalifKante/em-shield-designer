#ifndef SRC_VOLTAGESOURCE_H
#define SRC_VOLTAGESOURCE_H

#include "BranchTemplate.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace EMCore {

class SRC_VoltageSource : public BranchTemplate {
public:
    SRC_VoltageSource(int node_from, int node_to, int branch_id,
                      Complex V_incident = Complex(1.0, 0.0),
                      double Z_source = 120.0 * M_PI)
        : BranchTemplate(BranchType::SRC_VOLTAGE, node_from, node_to, branch_id)
        , m_V_inc(V_incident)
        , m_Zs(Z_source)
    {}

    std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const override {
        // Series impedance Zs
        // Y-parameters for series impedance: Y11 = Y22 = 1/Zs, Y12 = Y21 = -1/Zs
        Complex Y_s = 1.0 / m_Zs;

        return {{Y_s, -Y_s},   // [Y11, Y12]
                {-Y_s, Y_s}};  // [Y21, Y22]
    }

    std::vector<Complex> getVoltageSourceVector(double f_Hz) const override {
        // FIXED: Voltage source convention for standard MNA
        //
        // Branch connects: node_from (terminal 1) → node_to (terminal 2)
        // For this source: Node 0 (GND, terminal 1) → Node 1 (terminal 2)
        //
        // The voltage V_inc appears between the terminals such that:
        // V_terminal2 - V_terminal1 = V_inc
        //
        // Since terminal 1 is at GND (V=0), we put V_inc on terminal 2

        return {Complex(0.0, 0.0), m_V_inc};
        //      ↑ terminal 1 (GND)  ↑ terminal 2 (active node, receives voltage)
    }

    bool isValid(std::string& error_msg) const override {
        if (std::abs(m_Zs) < 1e-6) {
            error_msg = "Source impedance too small";
            return false;
        }
        return true;
    }

    std::string getDescription() const override {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "Voltage Source: V_inc=%.2fV, Zs=%.1f Ohm",
                      std::abs(m_V_inc), m_Zs);
        return std::string(buf);
    }

    // Getters and setters
    Complex getVoltage() const { return m_V_inc; }
    double getSourceImpedance() const { return m_Zs; }
    void setVoltage(Complex V) { m_V_inc = V; }
    void setSourceImpedance(double Zs) { m_Zs = Zs; }

private:
    Complex m_V_inc;  // Incident voltage [V]
    double m_Zs;      // Source impedance [Ω]
};

} // namespace EMCore

#endif // SRC_VOLTAGESOURCE_H
