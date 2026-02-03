#ifndef MNASOLVER_H
#define MNASOLVER_H

#include "BranchTemplate.h"
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace EMCore {

class MNASolver {
public:
    // Constructor
    MNASolver();

    // Add branches to the system
    void addBranch(std::shared_ptr<BranchTemplate> branch);

    // Clear all branches
    void clearBranches();

    // Solve the system at a given frequency
    // Returns: node voltage vector [V1, V2, ..., V_M] (node 0 = ground is implicit)
    Eigen::VectorXcd solve(double f_Hz);

    // Get number of independent nodes (excluding ground)
    int getNumNodes() const;

    // Get number of branches
    int getNumBranches() const { return branches_.size(); }

    // Enable/disable debug output
    void setDebugMode(bool enable) { debug_mode_ = enable; }

private:
    std::vector<std::shared_ptr<BranchTemplate>> branches_;
    int max_node_index_;  // Highest node index used
    bool debug_mode_;     // Print detailed MNA matrices for debugging

    // Build incidence matrix A
    Eigen::MatrixXd buildIncidenceMatrix();

    // Build block-diagonal Y matrix from all branches
    Eigen::MatrixXcd buildBranchYMatrix(double f_Hz);

    // Build source voltage vector
    Eigen::VectorXcd buildSourceVector(double f_Hz);
};

} // namespace EMCore

#endif // MNASOLVER_H
