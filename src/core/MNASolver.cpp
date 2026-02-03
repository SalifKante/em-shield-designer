#include "../../include/core/MNASolver.h"
#include <complex>
#include <algorithm>
#include <stdexcept>
#include <iostream>

namespace EMCore {
using Complex = std::complex<double>;

MNASolver::MNASolver() : max_node_index_(0) {}

void MNASolver::addBranch(std::shared_ptr<BranchTemplate> branch) {
    branches_.push_back(branch);
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

    // A is [2*N_branches × M_nodes] - TWO ROWS PER BRANCH (one per terminal)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * N_branches, M_nodes);

    for (int k = 0; k < N_branches; ++k) {
        int node_from = branches_[k]->getNodeFrom();
        int node_to = branches_[k]->getNodeTo();

        // Row for terminal "from" (terminal A)
        int row_from = 2 * k;
        // Row for terminal "to" (terminal B)
        int row_to = 2 * k + 1;

        // Voltage at terminal = voltage at node
        if (node_from > 0) {  // Not ground
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
        auto Y_branch = branches_[k]->computeYParameters(f_Hz);

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
        V_vec(2 * k)     = V_branch[0];
        V_vec(2 * k + 1) = V_branch[1];
    }

    return V_vec;
}

Eigen::VectorXcd MNASolver::solve(double f_Hz) {
    if (branches_.empty()) {
        throw std::runtime_error("No branches added to MNA solver");
    }

    // Build system matrices
    Eigen::MatrixXd A_temp = buildIncidenceMatrix();  // [2N × M]
    Eigen::MatrixXd A = A_temp.transpose();            // [M × 2N] ← TRANSPOSE!
    Eigen::MatrixXcd Y = buildBranchYMatrix(f_Hz);    // [2N × 2N]
    Eigen::VectorXcd V_vec = buildSourceVector(f_Hz); // [2N × 1]

    // Convert A to complex
    Eigen::MatrixXcd A_complex = A.cast<Complex>();

    // MNA equation: A * Y * A^T * U = A * Y * V_vec
    Eigen::MatrixXcd LHS = A_complex * Y * A_complex.transpose();
    Eigen::VectorXcd RHS = A_complex * Y * V_vec;

    // Solve for node voltages
    Eigen::VectorXcd U = LHS.colPivHouseholderQr().solve(RHS);

    return U;
}

} // namespace EMCore
