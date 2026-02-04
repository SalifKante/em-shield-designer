#include "../../include/core/MNASolver.h"
#include <complex>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace EMCore {

using Complex = std::complex<double>;

MNASolver::MNASolver()
    : max_node_index_(0)
    , debug_mode_(false)
{}

void MNASolver::addBranch(std::shared_ptr<BranchTemplate> branch) {
    branches_.push_back(branch);

    // Update maximum node index
    max_node_index_ = std::max({max_node_index_,
                                branch->getNodeFrom(),
                                branch->getNodeTo()});
}

void MNASolver::clearBranches() {
    branches_.clear();
    max_node_index_ = 0;
}

int MNASolver::getNumNodes() const {
    return max_node_index_;
}

Eigen::MatrixXd MNASolver::buildIncidenceMatrix() {
    int N_branches = branches_.size();
    int M_nodes = getNumNodes();

    // A_temp is [2*N_branches × M_nodes] - TWO ROWS PER BRANCH (one per terminal)
    // This follows MATLAB convention: A_temp = zeros(8, 2) before transpose
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * N_branches, M_nodes);

    for (int k = 0; k < N_branches; ++k) {
        int node_from = branches_[k]->getNodeFrom();
        int node_to = branches_[k]->getNodeTo();

        // Row for terminal "from" (terminal A)
        int row_from = 2 * k;
        // Row for terminal "to" (terminal B)
        int row_to = 2 * k + 1;

        // Voltage at terminal = voltage at node
        if (node_from > 0) {  // Not ground (ground = node 0)
            A(row_from, node_from - 1) = 1.0;
        }
        if (node_to > 0) {  // Not ground
            A(row_to, node_to - 1) = 1.0;
        }
    }

    return A;
}

Eigen::MatrixXcd MNASolver::buildBranchYMatrix(double f_Hz) {
    int N_branches = branches_.size();

    // Y is [2*N_branches × 2*N_branches] block diagonal
    Eigen::MatrixXcd Y = Eigen::MatrixXcd::Zero(2 * N_branches, 2 * N_branches);

    for (int k = 0; k < N_branches; ++k) {
        // Get Y-parameters for this branch
        auto Y_branch = branches_[k]->computeYParameters(f_Hz);

        // Validate Y-matrix dimensions
        if (Y_branch.size() != 2 || Y_branch[0].size() != 2 || Y_branch[1].size() != 2) {
            throw std::runtime_error(
                "Branch " + std::to_string(branches_[k]->getBranchID()) +
                " (" + branches_[k]->getDescription() + ")" +
                " returned invalid Y-matrix dimensions (expected 2×2)"
                );
        }

        // Fill 2×2 block in global Y matrix
        int idx = 2 * k;
        Y(idx,     idx)     = Y_branch[0][0];  // Y11
        Y(idx,     idx + 1) = Y_branch[0][1];  // Y12
        Y(idx + 1, idx)     = Y_branch[1][0];  // Y21
        Y(idx + 1, idx + 1) = Y_branch[1][1];  // Y22
    }

    return Y;
}

Eigen::VectorXcd MNASolver::buildSourceVector(double f_Hz) {
    int N_branches = branches_.size();

    // V_vec is [2*N_branches × 1]
    Eigen::VectorXcd V_vec = Eigen::VectorXcd::Zero(2 * N_branches);

    for (int k = 0; k < N_branches; ++k) {
        auto V_branch = branches_[k]->getVoltageSourceVector(f_Hz);

        // Validate vector size
        if (V_branch.size() != 2) {
            throw std::runtime_error(
                "Branch " + std::to_string(branches_[k]->getBranchID()) +
                " returned invalid source vector size (expected 2)"
                );
        }

        V_vec(2 * k)     = V_branch[0];
        V_vec(2 * k + 1) = V_branch[1];
    }

    return V_vec;
}

Eigen::VectorXcd MNASolver::solve(double f_Hz) {
    // ========================================================================
    // VALIDATION
    // ========================================================================
    if (branches_.empty()) {
        throw std::runtime_error("MNASolver: No branches added to system");
    }

    if (f_Hz <= 0) {
        throw std::runtime_error("MNASolver: Frequency must be positive");
    }

    if (max_node_index_ == 0) {
        throw std::runtime_error("MNASolver: No valid nodes found (all nodes are ground?)");
    }

    // ========================================================================
    // BUILD SYSTEM MATRICES
    // ========================================================================

    // Build incidence matrix [2N × M] then transpose to [M × 2N]
    // This matches MATLAB: A_temp = zeros(8,2); A = A_temp';
    Eigen::MatrixXd A_temp = buildIncidenceMatrix();  // [2N × M]
    Eigen::MatrixXd A = A_temp.transpose();            // [M × 2N] ← TRANSPOSE!

    // Build branch Y-matrix [2N × 2N] (block diagonal)
    Eigen::MatrixXcd Y = buildBranchYMatrix(f_Hz);

    // Build source vector [2N × 1]
    Eigen::VectorXcd V_vec = buildSourceVector(f_Hz);

    // ========================================================================
    // DEBUG OUTPUT (OPTIONAL)
    // ========================================================================
    if (debug_mode_) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║  MNA System Debug Output @ f = " << f_Hz/1e9 << " GHz" << std::setw(20) << "║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

        std::cout << "System dimensions:\n";
        std::cout << "  Branches: " << branches_.size() << "\n";
        std::cout << "  Nodes (excl. GND): " << getNumNodes() << "\n";
        std::cout << "  Y-matrix size: " << Y.rows() << "×" << Y.cols() << "\n";
        std::cout << "  A-matrix size: " << A.rows() << "×" << A.cols() << "\n\n";

        std::cout << "Branch descriptions:\n";
        for (size_t k = 0; k < branches_.size(); ++k) {
            std::cout << "  Branch " << k << ": " << branches_[k]->getDescription() << "\n";
            std::cout << "    Nodes: " << branches_[k]->getNodeFrom()
                      << " → " << branches_[k]->getNodeTo() << "\n";
        }
        std::cout << "\n";

        std::cout << "Incidence matrix A [" << A.rows() << "×" << A.cols() << "]:\n";
        std::cout << A << "\n\n";

        std::cout << "Branch Y-matrix (block diagonal) [" << Y.rows() << "×" << Y.cols() << "]:\n";
        std::cout << Y << "\n\n";

        std::cout << "Source vector V_vec [" << V_vec.size() << "×1]:\n";
        std::cout << V_vec << "\n\n";
    }

    // ========================================================================
    // SOLVE MNA SYSTEM
    // ========================================================================

    // Convert A to complex for matrix operations
    Eigen::MatrixXcd A_complex = A.cast<Complex>();

    // MNA equation (standard formulation):
    // A * Y * A^T * U = -A * Y * V_vec
    //                    ↑ NEGATIVE SIGN (standard MNA convention)

    Eigen::MatrixXcd LHS = A_complex * Y * A_complex.transpose();  // [M×2N]*[2N×2N]*[2N×M] = [M×M]
    Eigen::VectorXcd RHS = -A_complex * Y * V_vec;                 // -[M×2N]*[2N×2N]*[2N×1] = [M×1]

    if (debug_mode_) {
        std::cout << "Reduced system LHS (A·Y·A^T) [" << LHS.rows() << "×" << LHS.cols() << "]:\n";
        std::cout << LHS << "\n\n";

        std::cout << "Reduced system RHS (-A·Y·V_vec) [" << RHS.size() << "×1]:\n";
        std::cout << RHS << "\n\n";
    }

    // Solve for node voltages using QR decomposition (robust solver)
    Eigen::VectorXcd U = LHS.colPivHouseholderQr().solve(RHS);

    // ========================================================================
    // VERIFY SOLUTION QUALITY
    // ========================================================================
    double relative_error = (LHS * U - RHS).norm() / RHS.norm();

    if (debug_mode_) {
        std::cout << "Solution node voltages U [" << U.size() << "×1]:\n";
        std::cout << U << "\n\n";
        std::cout << "Relative error: " << std::scientific << relative_error << "\n";

        if (relative_error > 1e-6) {
            std::cout << "⚠ WARNING: High relative error in solution!\n";
        }

        std::cout << "════════════════════════════════════════════════════════════\n\n";
    }

    // Warn if solution quality is poor
    if (relative_error > 1e-3) {
        std::cerr << "WARNING: MNA solution has high relative error ("
                  << relative_error << ")\n";
        std::cerr << "         Solution may be inaccurate. Check for:\n";
        std::cerr << "         - Singular or near-singular system matrix\n";
        std::cerr << "         - Ill-conditioned branches (very large/small impedances)\n";
        std::cerr << "         - Numerical issues at this frequency\n";
    }

    return U;
}

} // namespace EMCore
