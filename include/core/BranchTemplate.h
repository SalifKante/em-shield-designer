#ifndef BRANCHTEMPLATE_H
#define BRANCHTEMPLATE_H

#include <complex>
#include <vector>
#include <string>

namespace EMCore {
// Type alias for complex numbers (shorter to write)
using Complex = std::complex<double>;

// ============================================================================
// BRANCH TEMPLATE BASE CLASS
// ============================================================================
// Abstract interface for all electromagnetic circuit elements.
//
// Each physical element (cavity, aperture, source, load) inherits from this
// and implements its own Y-parameter computation based on its physics.
//
// Topology: Each branch connects two nodes (node_from -> node_to)
// Example:
//   - Transmission line: node_1 --[TL]-- node_2
//   - Aperture: node_2 --[AP]-- node_3
//   - Voltage source: node_0 --[SRC]-- node_1
//   - Load: node_N --[LOAD]-- ground(0)
// ============================================================================

class BranchTemplate {
public:
    // Types of branches in our system
    enum class BranchType {
        TL_EMPTY_CAVITY,       // Empty rectangular cavity (TE10 mode)
        TL_DIELECTRIC_CAVITY,  // Dielectric-filled cavity
        AP_SLOT_APERTURE,      // Slot aperture coupling between sections
        AP_SLOT_WITH_COVER,    // Slot aperture with protective cover/shield
        SRC_VOLTAGE,           // Voltage source (incident electromagnetic field)
        LOAD_IMPEDANCE         // Termination load (matched or mismatched)
    };

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    BranchTemplate(BranchType type, int node_from, int node_to, int branch_id)
        : m_type(type)
        , m_node_from(node_from)
        , m_node_to(node_to)
        , m_branch_id(branch_id)
    {}

    // Virtual destructor (essential for polymorphism!)
    virtual ~BranchTemplate() = default;

    // ========================================================================
    // PURE VIRTUAL METHODS
    // ========================================================================
    // These MUST be implemented by every derived class (= 0 means "pure virtual")

    // Compute 2x2 Y-parameter matrix at given frequency
    // Returns: [ [Y11, Y12],
    //            [Y21, Y22] ]
    //
    // Y-parameters relate voltages and currents:
    //   I1 = Y11*V1 + Y12*V2
    //   I2 = Y21*V1 + Y22*V2
    virtual std::vector<std::vector<Complex>> computeYParameters(double f_Hz) const = 0;

    // Check if branch parameters are physically valid
    // Returns: true if valid, false otherwise
    // Sets error_msg if validation fails
    virtual bool isValid(std::string& error_msg) const = 0;

    // Get human-readable description (for GUI display and debugging)
    // Example: "Empty Cavity: 50x25x100 mm, fc=3.0 GHz"
    virtual std::string getDescription() const = 0;

    // ========================================================================
    // VIRTUAL METHOD WITH DEFAULT IMPLEMENTATION
    // ========================================================================

    // Get voltage source vector [V1, V2]
    // Only voltage sources override this; passive elements use default (zeros)
    virtual std::vector<Complex> getVoltageSourceVector(double f_Hz) const {
        return {Complex(0.0, 0.0), Complex(0.0, 0.0)};
    }

    // ========================================================================
    // ACCESSORS (non-virtual, common to all branches)
    // ========================================================================

    BranchType getType() const { return m_type; }
    int getNodeFrom() const { return m_node_from; }
    int getNodeTo() const { return m_node_to; }
    int getBranchID() const { return m_branch_id; }

    // Helper to check if this branch is a voltage source
    bool isVoltageSource() const {
        return m_type == BranchType::SRC_VOLTAGE;
    }

protected:
    // Member variables (accessible by derived classes)
    BranchType m_type;      // What kind of branch is this?
    int m_node_from;        // Starting node (0-based indexing)
    int m_node_to;          // Ending node (0-based indexing)
    int m_branch_id;        // Unique identifier for this branch
};

} // namespace EMCore

#endif // BRANCHTEMPLATE_H
