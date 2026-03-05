#ifndef MNASOLVER_H
#define MNASOLVER_H

#include "BranchTemplate.h"
#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>

namespace EMCore {

// ============================================================================
// MODIFIED NODAL ANALYSIS SOLVER
// ============================================================================
// Assembles and solves the MNA system for the EMShieldDesigner equivalent
// circuit, implementing the core linear system of Section 3.1.2.1:
//
//   A · Y · Aᵀ · U = −A · Y · V          (Eq. 3.9)
//
// where:
//   Y   — block-diagonal branch admittance matrix [2N × 2N]
//   A   — incidence matrix                        [M  × 2N]
//   U   — unknown nodal voltage vector            [M  × 1 ]
//   V   — branch source voltage vector            [2N × 1 ]
//   N   — number of branches
//   M   — number of non-ground nodes
//
// ============================================================================
// INCIDENCE MATRIX CONVENTION
// ============================================================================
// This solver uses an UNSIGNED incidence matrix, differing from the signed
// convention shown in Eq. (3.10):
//
//   Theory (signed):    A = [ -1  +1  +1   0 ]
//                           [  0   0  -1  +1 ]
//
//   Implementation:     all non-zero entries are +1
//
// Why this is mathematically equivalent for SE computation:
//   All passive branches carry Y₁₂ = Y₂₁ = −Y₁₁ (from the 2-port stamp).
//   With unsigned A, the assembled nodal admittance matrix A·Y·Aᵀ produces
//   the same numerical values as the signed convention. The RHS vector
//   −A·Y·V also produces the same magnitude at every node.
//
//   Consequence: the solved voltage vector U satisfies
//     U_code = −U_theory  (overall sign inversion)
//
//   Impact on SE (Eq. 3.8):
//     SE = −20·log₁₀|2·U_obs/V₀|
//   Since |−U| = |U|, the SE result is identical to the theory.
//   The convention is therefore self-consistent for all SE computations.
//
// IMPORTANT: Raw nodal voltages from solve() carry this sign convention.
//   Do NOT use the sign of U for phase calculations or field reconstruction
//   without accounting for this inversion.
//
// Per-terminal source vector:
//   V_vec is a 2N-element vector where entry (2k, 2k+1) holds the terminal
//   voltages of branch k as returned by getVoltageSourceVector().
//   For SRC_VoltageSource: V_vec = [..., 0, V₀, ...]   (terminal 2 = V₀)
//   For passive branches:  V_vec = [..., 0,  0, ...]
//
// ============================================================================
// SHIELDING EFFECTIVENESS — Equation (3.8)
// ============================================================================
//
//   SE = −20·log₁₀|2·U_obs / V₀|   [dB]
//
//   U_obs — complex nodal voltage at the observation node
//   V₀    — incident source voltage (typically 1+0j for normalisation)
//
//   The factor of 2 arises from the voltage-divider relationship between the
//   source and the matched load in the equivalent circuit. It is mandatory
//   and must not be omitted.
//
//   computeSE() encapsulates this formula to prevent per-caller
//   re-implementation errors.
// ============================================================================

class MNASolver {
public:

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    MNASolver();

    // ========================================================================
    // CIRCUIT ASSEMBLY
    // ========================================================================

    /**
     * @brief Add a branch to the MNA system.
     *
     * Branches are indexed in the order they are added. The incidence matrix
     * rows and branch Y-matrix blocks are ordered accordingly.
     *
     * Node indices must be contiguous: if the highest node index added is M,
     * all indices 1 … M must appear. Gaps (e.g. nodes 1, 3 without 2) will
     * produce a singular nodal admittance matrix and are detected at solve().
     *
     * @param branch  Shared pointer to a BranchTemplate-derived object.
     */
    void addBranch(std::shared_ptr<BranchTemplate> branch);

    /** @brief Remove all branches and reset node tracking. */
    void clearBranches();

    // ========================================================================
    // SOLVE
    // ========================================================================

    /**
     * @brief Assemble and solve the MNA system at frequency f_Hz.
     *
     * Implements Eq. (3.9):  A·Y·Aᵀ·U = −A·Y·V
     *
     * Uses Eigen colPivHouseholderQr for numerical stability near resonance.
     * Solution quality is checked via relative residual; a warning is emitted
     * to std::cerr if the relative error exceeds 1e-3.
     *
     * @param f_Hz  Frequency [Hz]. Must be strictly positive.
     * @returns     Complex nodal voltage vector U [M × 1].
     *              Entry k corresponds to node (k+1); node 0 (ground) is
     *              implicit and excluded.
     * @throws std::runtime_error on empty branch list, non-positive frequency,
     *         non-contiguous node numbering, or singular system matrix.
     */
    Eigen::VectorXcd solve(double f_Hz);

    // ========================================================================
    // SHIELDING EFFECTIVENESS — Equation (3.8)
    // ========================================================================

    /**
     * @brief Compute Shielding Effectiveness from a solved voltage vector.
     *
     * Implements Eq. (3.8):
     *   SE = −20·log₁₀(|2·U_obs / V₀|)   [dB]
     *
     * @param U          Node voltage vector returned by solve().
     * @param obs_node   Node index of the observation point (1-based, > 0).
     * @param V0         Incident source voltage phasor [V].
     *                   Use Complex(1.0, 0.0) for the standard unit-source
     *                   normalisation.
     * @returns          SE in dB. Returns +∞ dB if U_obs == 0 (perfect shield).
     * @throws std::out_of_range if obs_node is outside [1, M].
     */
    double computeSE(const Eigen::VectorXcd& U,
                     int                     obs_node,
                     Complex                 V0) const;

    // ========================================================================
    // ACCESSORS
    // ========================================================================

    /** @brief Number of non-ground nodes M (excluding ground node 0). */
    int getNumNodes() const;

    /** @brief Number of branches N currently registered. */
    int getNumBranches() const { return static_cast<int>(branches_.size()); }

    /** @brief Enable or disable verbose matrix debug output to std::cout. */
    void setDebugMode(bool enable) { debug_mode_ = enable; }

    bool getDebugMode() const { return debug_mode_; }

private:

    std::vector<std::shared_ptr<BranchTemplate>> branches_;
    int  max_node_index_;
    bool debug_mode_;

    // ----------------------------------------------------------------
    // Matrix builders (called internally by solve())
    // ----------------------------------------------------------------

    /**
     * @brief Build unsigned incidence matrix A_temp [2N × M].
     *        solve() transposes this to A [M × 2N].
     *        See class header for the unsigned-convention rationale.
     */
    Eigen::MatrixXd buildIncidenceMatrix();

    /**
     * @brief Build block-diagonal branch admittance matrix Y [2N × 2N].
     *        Block k occupies rows/cols [2k, 2k+1].
     */
    Eigen::MatrixXcd buildBranchYMatrix(double f_Hz);

    /**
     * @brief Build per-terminal source voltage vector V_vec [2N × 1].
     *        Entry (2k, 2k+1) = getVoltageSourceVector() of branch k.
     */
    Eigen::VectorXcd buildSourceVector(double f_Hz);

    /**
     * @brief Verify that node indices are contiguous (1 … max_node_index_).
     *        Throws std::runtime_error if a gap is detected.
     */
    void validateNodeContiguity() const;
};

} // namespace EMCore

#endif // MNASOLVER_H
