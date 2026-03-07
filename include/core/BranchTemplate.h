#ifndef BRANCHTEMPLATE_H
#define BRANCHTEMPLATE_H

#include <complex>
#include <vector>
#include <string>

namespace EMCore {

// ============================================================================
// COMPLEX NUMBER TYPE ALIAS
// ============================================================================
// std::complex<double> is used throughout the framework for admittances,
// impedances, propagation constants, and nodal voltages.
// Eigen's MatrixXcd is also parameterised on std::complex<double>, so this
// alias ensures full compatibility with the MNA linear algebra layer.

using Complex = std::complex<double>;

// ============================================================================
// BRANCHTEMPLATE — Abstract Base Class for All Circuit Elements
// ============================================================================
// Defines the interface that every electromagnetic circuit branch must satisfy
// for correct assembly into the MNA system (Eq. 3.9):
//
//   A · Y · Aᵀ · U = −A · Y · V
//
// Derived classes:
//   TL_EmptyCavity        — rectangular waveguide segment, air-filled
//   TL_DielectricCavity   — rectangular waveguide segment, partially filled
//   AP_SlotAperture        — open slot aperture (Eq. 3.13)
//   AP_SlotWithCover       — slot with removable cover (Eqs. 3.22 / 3.24)
//   SRC_VoltageSource      — incident-field source with series impedance
//   LOAD_Impedance         — termination impedance
//
// ============================================================================
// NODE INDEXING CONVENTION
// ============================================================================
// Nodes use 1-BASED indexing throughout the framework:
//
//   Node 0          — implicit ground reference (never assigned to a column
//                     in the incidence matrix; handled by the > 0 guard in
//                     MNASolver::buildIncidenceMatrix).
//   Nodes 1 … M    — active (non-ground) nodes.
//
// MNASolver converts to 0-based Eigen column indices internally:
//   Eigen column = node_index − 1
//
// CircuitGenerator assigns node indices starting at 1 for the first active
// node. A branch connected to ground uses node_to = 0 (or node_from = 0).
//
// Example — single-section circuit (Figure 3.7):
//   Source:      node_from=0, node_to=1   (0 = ground, 1 = first active node)
//   Aperture:    node_from=1, node_to=0   (shunt to ground)
//   TL_p:        node_from=1, node_to=2   (observation node P₁ = 2)
//   TL_{d-p}:    node_from=2, node_to=0   (short-circuit back wall)
//
// ============================================================================
// Y-PARAMETER CONTRACT
// ============================================================================
// computeYParameters(f_Hz) must return a 2×2 matrix in row-major order:
//
//   return { { Y11, Y12 },
//            { Y21, Y22 } };
//
// MNASolver::buildBranchYMatrix() accesses:
//   Y_branch[0][0] = Y11,  Y_branch[0][1] = Y12
//   Y_branch[1][0] = Y21,  Y_branch[1][1] = Y22
//
// All passive reciprocal branches must satisfy:
//   Y12 = Y21        (symmetry)
//   Y11 = Y22        (for symmetric two-ports)
//   Y12 = −Y11       (passivity / KCL at each port)
//
// ============================================================================
// VOLTAGE SOURCE VECTOR CONTRACT
// ============================================================================
// getVoltageSourceVector(f_Hz) must return a size-2 vector:
//   { V_terminal1, V_terminal2 }
//
// MNASolver::buildSourceVector() places:
//   V_vec(2k)   = V_terminal1
//   V_vec(2k+1) = V_terminal2
//
// Passive elements use the default implementation which returns {0, 0}.
// SRC_VoltageSource overrides this to return {0, V₀}.
// ============================================================================

class BranchTemplate {
public:

    // ========================================================================
    // BRANCH TYPE IDENTIFIER
    // ========================================================================

    enum class BranchType {
        TL_EMPTY_CAVITY,       ///< Air-filled rectangular cavity (TE₁₀ mode)
        TL_DIELECTRIC_CAVITY,  ///< Partially dielectric-filled cavity
        AP_SLOT_APERTURE,      ///< Open slot aperture (Eq. 3.13)
        AP_SLOT_WITH_COVER,    ///< Slot with removable cover (Eqs. 3.22 / 3.24)
        SRC_VOLTAGE,           ///< Incident-field voltage source
        LOAD_IMPEDANCE         ///< Termination load impedance
    };

    // ========================================================================
    // CONSTRUCTOR / DESTRUCTOR
    // ========================================================================

    /**
     * @param type       Branch type identifier
     * @param node_from  Source node (1-based; 0 = ground)
     * @param node_to    Sink node   (1-based; 0 = ground)
     * @param branch_id  Unique branch index (assigned by CircuitGenerator)
     */
    BranchTemplate(BranchType type,
                   int        node_from,
                   int        node_to,
                   int        branch_id)
        : m_type(type)
        , m_node_from(node_from)
        , m_node_to(node_to)
        , m_branch_id(branch_id)
    {}

    /// Virtual destructor — mandatory for correct polymorphic deletion.
    virtual ~BranchTemplate() = default;

    // ========================================================================
    // PURE VIRTUAL INTERFACE
    // ========================================================================

    /**
     * @brief Compute the 2×2 admittance matrix at frequency f_Hz.
     *
     * Returns { {Y11, Y12}, {Y21, Y22} } as defined in the class header.
     * Must be implemented by every derived class.
     *
     * @param f_Hz  Frequency [Hz]. Must be strictly positive.
     * @returns     2×2 complex admittance matrix [S].
     */
    virtual std::vector<std::vector<Complex>>
    computeYParameters(double f_Hz) const = 0;

    /**
     * @brief Validate physical parameters of this branch.
     *
     * Contract: sets error_msg and returns false on failure.
     *           leaves error_msg empty and returns true on success.
     *           Advisory conditions must go to std::cerr only.
     *
     * @param error_msg  Output: human-readable failure description (empty on success).
     * @returns          true if all parameters are physically valid.
     */
    virtual bool isValid(std::string& error_msg) const = 0;

    /**
     * @brief Return a human-readable description for GUI display and debug logs.
     *
     * Example: "Empty Cavity TL: a=50mm, b=25mm, L=100mm (fc10=3.000 GHz)"
     */
    virtual std::string getDescription() const = 0;

    // ========================================================================
    // VIRTUAL METHOD WITH DEFAULT IMPLEMENTATION
    // ========================================================================

    /**
     * @brief Return the per-terminal source voltage vector {V₁, V₂}.
     *
     * Passive branches use this default (returns {0, 0}).
     * SRC_VoltageSource overrides to return {0, V₀}.
     *
     * MNASolver::buildSourceVector() requires exactly 2 elements.
     *
     * @param f_Hz  Frequency [Hz] (unused by passive elements).
     * @returns     { V_terminal1, V_terminal2 }
     */
    virtual std::vector<Complex> getVoltageSourceVector(double /*f_Hz*/) const {
        return { Complex(0.0, 0.0), Complex(0.0, 0.0) };
    }

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    BranchType getType()     const { return m_type;      }

    /// Source node index (1-based; 0 = ground reference)
    int getNodeFrom()        const { return m_node_from; }

    /// Sink node index (1-based; 0 = ground reference)
    int getNodeTo()          const { return m_node_to;   }

    int getBranchID()        const { return m_branch_id; }

    bool isVoltageSource()   const {
        return m_type == BranchType::SRC_VOLTAGE;
    }

protected:
    BranchType m_type;       ///< Branch type identifier
    int        m_node_from;  ///< Source node (1-based; 0 = ground)
    int        m_node_to;    ///< Sink node   (1-based; 0 = ground)
    int        m_branch_id;  ///< Unique branch index
};

} // namespace EMCore

#endif // BRANCHTEMPLATE_H
