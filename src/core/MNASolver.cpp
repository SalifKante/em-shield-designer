#include "../../include/core/MNASolver.h"
#include <complex>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <set>
#include <cmath>

namespace EMCore {

using Complex = std::complex<double>;

// ============================================================================
// CONSTRUCTOR
// ============================================================================

MNASolver::MNASolver()
    : max_node_index_(0)
    , debug_mode_(false)
{}

// ============================================================================
// CIRCUIT ASSEMBLY
// ============================================================================

void MNASolver::addBranch(std::shared_ptr<BranchTemplate> branch) {
    if (!branch) {
        throw std::invalid_argument("MNASolver::addBranch: null branch pointer");
    }
    branches_.push_back(branch);
    max_node_index_ = std::max({ max_node_index_,
                                branch->getNodeFrom(),
                                branch->getNodeTo() });
}

void MNASolver::clearBranches() {
    branches_.clear();
    max_node_index_ = 0;
}

int MNASolver::getNumNodes() const {
    return max_node_index_;   // Count of non-ground nodes (ground = node 0)
}

// ============================================================================
// NODE CONTIGUITY VALIDATION
// ============================================================================
// Detects gaps in node numbering (e.g. nodes {1, 3} without node 2).
// A gap produces an all-zero column in A, making A·Y·Aᵀ singular.
//
// Algorithm: collect all non-ground node indices referenced by the branch
// list and verify they form the set {1, 2, …, max_node_index_}.

void MNASolver::validateNodeContiguity() const {
    std::set<int> nodes_used;
    for (const auto& b : branches_) {
        if (b->getNodeFrom() > 0) nodes_used.insert(b->getNodeFrom());
        if (b->getNodeTo()   > 0) nodes_used.insert(b->getNodeTo());
    }

    for (int n = 1; n <= max_node_index_; ++n) {
        if (nodes_used.find(n) == nodes_used.end()) {
            throw std::runtime_error(
                "MNASolver: node index " + std::to_string(n) +
                " is not referenced by any branch, but max node index is " +
                std::to_string(max_node_index_) +
                ". Node indices must be contiguous (1 … " +
                std::to_string(max_node_index_) +
                "). Check CircuitGenerator node assignments.");
        }
    }
}

// ============================================================================
// MATRIX BUILDERS
// ============================================================================

// ----------------------------------------------------------------------------
// buildIncidenceMatrix
// ----------------------------------------------------------------------------
// Constructs the UNSIGNED incidence matrix A_temp [2N × M].
//
// Layout: each branch k occupies two consecutive rows:
//   row 2k   → terminal 1 of branch k  (node_from)
//   row 2k+1 → terminal 2 of branch k  (node_to)
//
// Entry A_temp(row, col) = 1.0 if terminal (row) is connected to node (col+1).
// Ground connections (node 0) produce no entry (row remains zero).
//
// NOTE: solve() transposes A_temp to obtain A [M × 2N] before use.
//
// For the rationale of using unsigned entries rather than the signed
// convention of Eq. (3.10), see the class header documentation.

Eigen::MatrixXd MNASolver::buildIncidenceMatrix() {
    const int N = static_cast<int>(branches_.size());
    const int M = getNumNodes();

    Eigen::MatrixXd A_temp = Eigen::MatrixXd::Zero(2 * N, M);

    for (int k = 0; k < N; ++k) {
        const int node_from = branches_[k]->getNodeFrom();
        const int node_to   = branches_[k]->getNodeTo();

        // Terminal 1 row: connects to node_from
        if (node_from > 0) {
            A_temp(2 * k,     node_from - 1) = 1.0;   // 1-based → 0-based col
        }
        // Terminal 2 row: connects to node_to
        if (node_to > 0) {
            A_temp(2 * k + 1, node_to   - 1) = 1.0;
        }
    }

    return A_temp;   // [2N × M]; caller transposes to [M × 2N]
}

// ----------------------------------------------------------------------------
// buildBranchYMatrix
// ----------------------------------------------------------------------------
// Constructs the block-diagonal branch admittance matrix Y [2N × 2N].
//
// Block k occupies the sub-matrix at rows/cols [2k, 2k+1]:
//   Y[2k  , 2k  ] = Y11   Y[2k  , 2k+1] = Y12
//   Y[2k+1, 2k  ] = Y21   Y[2k+1, 2k+1] = Y22
//
// Off-diagonal blocks between different branches are zero (block-diagonal).

Eigen::MatrixXcd MNASolver::buildBranchYMatrix(double f_Hz) {
    const int N = static_cast<int>(branches_.size());
    Eigen::MatrixXcd Y = Eigen::MatrixXcd::Zero(2 * N, 2 * N);

    for (int k = 0; k < N; ++k) {
        const auto Y_branch = branches_[k]->computeYParameters(f_Hz);

        if (Y_branch.size() != 2 ||
            Y_branch[0].size() != 2 ||
            Y_branch[1].size() != 2) {
            throw std::runtime_error(
                "MNASolver: branch " +
                std::to_string(branches_[k]->getBranchID()) +
                " (" + branches_[k]->getDescription() + ")" +
                " returned a Y-matrix with invalid dimensions (expected 2×2).");
        }

        const int idx = 2 * k;
        Y(idx,     idx    ) = Y_branch[0][0];   // Y11
        Y(idx,     idx + 1) = Y_branch[0][1];   // Y12
        Y(idx + 1, idx    ) = Y_branch[1][0];   // Y21
        Y(idx + 1, idx + 1) = Y_branch[1][1];   // Y22
    }

    return Y;
}

// ----------------------------------------------------------------------------
// buildSourceVector
// ----------------------------------------------------------------------------
// Constructs the per-terminal source voltage vector V_vec [2N × 1].
//
// For branch k:
//   V_vec[2k]   = terminal-1 voltage of branch k
//   V_vec[2k+1] = terminal-2 voltage of branch k
//
// For SRC_VoltageSource: getVoltageSourceVector() returns {0, V₀},
//   placing V₀ at terminal 2 (the active node side).
// For all passive branches: returns {0, 0}.
//
// The assembled V_vec is used in the RHS: −A·Y·V_vec  (Eq. 3.9).

Eigen::VectorXcd MNASolver::buildSourceVector(double f_Hz) {
    const int N = static_cast<int>(branches_.size());
    Eigen::VectorXcd V_vec = Eigen::VectorXcd::Zero(2 * N);

    for (int k = 0; k < N; ++k) {
        const auto V_branch = branches_[k]->getVoltageSourceVector(f_Hz);

        if (V_branch.size() != 2) {
            throw std::runtime_error(
                "MNASolver: branch " +
                std::to_string(branches_[k]->getBranchID()) +
                " returned a source vector with invalid size (expected 2).");
        }

        V_vec(2 * k    ) = V_branch[0];   // Terminal 1 voltage
        V_vec(2 * k + 1) = V_branch[1];   // Terminal 2 voltage
    }

    return V_vec;
}

// ============================================================================
// SOLVE — Equation (3.9): A·Y·Aᵀ·U = −A·Y·V
// ============================================================================

Eigen::VectorXcd MNASolver::solve(double f_Hz) {

    // --- Pre-conditions ------------------------------------------------------
    if (branches_.empty()) {
        throw std::runtime_error("MNASolver::solve: no branches in system.");
    }
    if (f_Hz <= 0.0) {
        throw std::runtime_error(
            "MNASolver::solve: frequency must be positive (got " +
            std::to_string(f_Hz) + " Hz).");
    }
    if (max_node_index_ == 0) {
        throw std::runtime_error(
            "MNASolver::solve: no non-ground nodes found. "
            "All branches connect to ground?");
    }

    // Node contiguity check: detects gaps like {1, 3} without node 2.
    // Throws std::runtime_error with a descriptive message if a gap exists.
    validateNodeContiguity();

    // --- Build system matrices -----------------------------------------------

    // A_temp [2N × M]; transpose to A [M × 2N] for Eq. (3.9).
    const Eigen::MatrixXd  A_temp  = buildIncidenceMatrix();
    const Eigen::MatrixXd  A_real  = A_temp.transpose();          // [M × 2N]
    const Eigen::MatrixXcd A       = A_real.cast<Complex>();       // [M × 2N] complex

    const Eigen::MatrixXcd Y       = buildBranchYMatrix(f_Hz);    // [2N × 2N]
    const Eigen::VectorXcd V_vec   = buildSourceVector(f_Hz);     // [2N × 1 ]

    // --- Debug output --------------------------------------------------------
    if (debug_mode_) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║  MNASolver Debug  —  f = "
                  << std::setw(10) << f_Hz / 1e9 << " GHz"
                  << "                        ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

        std::cout << "  Branches N = " << branches_.size()
                  << "   Non-ground nodes M = " << getNumNodes() << "\n\n";

        std::cout << "  Branch list:\n";
        for (std::size_t k = 0; k < branches_.size(); ++k) {
            std::cout << "    [" << k << "]  "
                      << branches_[k]->getDescription()
                      << "   nodes: "
                      << branches_[k]->getNodeFrom()
                      << " → " << branches_[k]->getNodeTo() << "\n";
        }

        std::cout << "\n  Incidence matrix A [" << A.rows()
                  << " × " << A.cols() << "]:\n" << A_real << "\n\n";

        std::cout << "  Branch Y-matrix [" << Y.rows()
                  << " × " << Y.cols() << "]:\n" << Y << "\n\n";

        std::cout << "  Source vector V_vec [" << V_vec.size()
                  << " × 1]:\n" << V_vec << "\n\n";
    }

    // --- Assemble LHS and RHS — Eq. (3.9) -----------------------------------
    //
    //   LHS = A · Y · Aᵀ     [M × M]
    //   RHS = −A · Y · V_vec  [M × 1]

    const Eigen::MatrixXcd LHS = A * Y * A.adjoint();   // adjoint = conjugate-transpose;
    // A is real so adjoint = transpose
    const Eigen::VectorXcd RHS = -(A * Y * V_vec);

    if (debug_mode_) {
        std::cout << "  LHS  A·Y·Aᵀ  [" << LHS.rows()
            << " × " << LHS.cols() << "]:\n" << LHS << "\n\n";
        std::cout << "  RHS  −A·Y·V  [" << RHS.size()
                  << " × 1]:\n"  << RHS << "\n\n";
    }

    // --- Solve using column-pivoting QR decomposition -----------------------
    //
    // colPivHouseholderQr is chosen over PartialPivLU because it handles
    // rank-deficient or nearly singular systems (which arise near waveguide
    // resonance frequencies) more robustly. At the matrix sizes encountered
    // in this application (typically 2×2 to 10×10), the additional cost is
    // negligible.

    const Eigen::VectorXcd U = LHS.colPivHouseholderQr().solve(RHS);

    // --- Residual quality check ----------------------------------------------
    const double rhs_norm        = RHS.norm();
    const double residual_norm   = (LHS * U - RHS).norm();
    const double relative_error  = (rhs_norm > 1e-15)
                                      ? residual_norm / rhs_norm
                                      : 0.0;

    if (debug_mode_) {
        std::cout << "  Solution U [" << U.size() << " × 1]:\n"
                  << U << "\n\n";
        std::cout << "  Relative residual error: "
                  << std::scientific << relative_error << "\n";
        if (relative_error > 1e-6) {
            std::cout << "  *** WARNING: elevated residual — check for "
                         "near-resonance or ill-conditioned branch impedances.\n";
        }
        std::cout << "══════════════════════════════════════════════════════════════\n\n";
    }

    if (relative_error > 1e-3) {
        std::cerr << "[MNASolver] WARNING: relative residual = "
                  << std::scientific << relative_error
                  << " at f = " << std::fixed << f_Hz / 1e9 << " GHz.\n"
                  << "  Possible causes: near-resonance singularity, "
                     "ill-conditioned branch impedances, or node gap.\n";
    }

    return U;
}

// ============================================================================
// SHIELDING EFFECTIVENESS — Equation (3.8)
// ============================================================================
//
//   SE = −20 · log₁₀(|2 · U_obs / V₀|)   [dB]
//
// Parameter mapping:
//   U        — node voltage vector from solve() (M elements, 0-based index)
//   obs_node — 1-based node index of the observation point
//   V0       — incident source voltage phasor (typically 1+0j)
//
// The observation node voltage is accessed as U(obs_node − 1) to convert
// from 1-based node numbering to 0-based Eigen indexing.
//
// Special case: U_obs = 0 (numerically) → SE → +∞ dB (perfect shield).
//   Returns std::numeric_limits<double>::infinity() in this case.
//
// Sign convention note:
//   Because solve() uses the unsigned-A convention, U_obs carries an overall
//   sign inversion relative to the theory. Since Eq. (3.8) uses |U_obs|,
//   this inversion has no effect on the computed SE value.

double MNASolver::computeSE(const Eigen::VectorXcd& U,
                            int                     obs_node,
                            Complex                 V0) const
{
    // --- Bounds check -------------------------------------------------------
    const int M = static_cast<int>(U.size());
    if (obs_node < 1 || obs_node > M) {
        throw std::out_of_range(
            "MNASolver::computeSE: obs_node = " + std::to_string(obs_node) +
            " is out of range [1, " + std::to_string(M) + "].");
    }

    // --- Source voltage magnitude check -------------------------------------
    const double V0_mag = std::abs(V0);
    if (V0_mag < 1e-30) {
        throw std::invalid_argument(
            "MNASolver::computeSE: |V₀| is effectively zero. "
            "Cannot compute SE ratio.");
    }

    // --- Extract observation voltage (1-based → 0-based) -------------------
    const Complex U_obs = U(obs_node - 1);
    const double  U_mag = std::abs(U_obs);

    // --- Perfect shield special case ----------------------------------------
    if (U_mag < 1e-30 * V0_mag) {
        return std::numeric_limits<double>::infinity();
    }

    // --- Equation (3.8) -----------------------------------------------------
    //   SE = −20 · log₁₀(|2 · U_obs / V₀|)
    const double ratio = (2.0 * U_mag) / V0_mag;
    return -20.0 * std::log10(ratio);
}

} // namespace EMCore
