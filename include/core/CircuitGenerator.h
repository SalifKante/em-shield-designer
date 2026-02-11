#ifndef CIRCUITGENERATOR_H
#define CIRCUITGENERATOR_H

#include "MNASolver.h"
#include "SRC_VoltageSource.h"
#include "AP_SlotAperture.h"
#include "AP_SlotWithCover.h"
#include "TL_EmptyCavity.h"
#include "TL_DielectricCavity.h"
#include "LOAD_Impedance.h"
#include "PhysicsConstants.h"

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace EMCore {

// ============================================================================
// SECTION CONFIGURATION
// ============================================================================
// Defines one physical section of a cascade enclosure.
// Each section has:
//   - Cavity dimensions and type (empty or dielectric)
//   - An aperture connecting it to the previous section (or exterior)
//   - An optional observation point at position p from the front wall
//
// The CircuitGenerator converts a vector of SectionConfig into MNA branches.
// ============================================================================

struct SectionConfig {
    // --- Cavity parameters ---
    double depth = 0.300;           // d_i: section depth [m]
    double obs_position = 0.150;    // p_i: observation point position [m]
    bool has_observation = true;    // Whether to record SE at this section

    // --- Aperture parameters (front aperture for section 0, inter-section for others) ---
    double aperture_l = 0.08;       // l_i: aperture width [m]
    double aperture_w = 0.08;       // w_i: aperture height [m]

    // --- Optional: aperture cover ---
    bool has_cover = false;
    double cover_gap = 0.001;       // tau: cover gap [m] (only if has_cover)

    // --- Optional: dielectric fill ---
    bool has_dielectric = false;
    double dielectric_h = 0.0;     // h: dielectric height [m]
    double dielectric_er = 1.0;    // epsilon_r: relative permittivity

    // --- Validation ---
    bool isValid(std::string& error_msg) const {
        if (depth <= 0) { error_msg = "Section depth must be positive"; return false; }
        if (obs_position < 0 || obs_position > depth) {
            error_msg = "Observation position must be within [0, depth]";
            return false;
        }
        if (aperture_l <= 0 || aperture_w <= 0) {
            error_msg = "Aperture dimensions must be positive";
            return false;
        }
        if (has_cover && cover_gap <= 0) {
            error_msg = "Cover gap must be positive when cover is enabled";
            return false;
        }
        if (has_dielectric && (dielectric_h <= 0 || dielectric_er <= 0)) {
            error_msg = "Dielectric parameters must be positive when dielectric is enabled";
            return false;
        }
        return true;
    }
};

// ============================================================================
// ENCLOSURE CONFIGURATION
// ============================================================================
// Shared parameters for the entire enclosure + list of sections.
// ============================================================================

struct EnclosureConfig {
    // --- Shared enclosure dimensions ---
    double a = 0.300;               // Enclosure width [m]
    double b = 0.120;               // Enclosure height [m]
    double t = 0.0015;              // Wall thickness [m]

    // --- Source parameters ---
    double Z_source = Z_0;          // Source impedance [Ohm]
    double V_source = 1.0;          // Source voltage magnitude [V]

    // --- Section list ---
    std::vector<SectionConfig> sections;

    // --- Validation ---
    bool isValid(std::string& error_msg) const {
        if (a <= 0 || b <= 0) { error_msg = "Enclosure dimensions must be positive"; return false; }
        if (t <= 0) { error_msg = "Wall thickness must be positive"; return false; }
        if (sections.empty()) { error_msg = "At least one section required"; return false; }
        for (size_t i = 0; i < sections.size(); ++i) {
            std::string sec_err;
            if (!sections[i].isValid(sec_err)) {
                error_msg = "Section " + std::to_string(i+1) + ": " + sec_err;
                return false;
            }
        }
        return true;
    }
};

// ============================================================================
// OBSERVATION POINT
// ============================================================================
// Records which MNA node corresponds to which physical observation point.
// ============================================================================

struct ObservationPoint {
    int node_id;            // MNA node index (1-based, 0 = ground)
    int section_index;      // Which section (0-based)
    std::string label;      // Display label, e.g. "P1", "P2"
};

// ============================================================================
// CIRCUIT GENERATOR
// ============================================================================
// Converts an EnclosureConfig into MNA branches and observation points.
//
// Circuit topology for N-section cascade:
//   Branch 0: Source             (Node 0 -> Node 1)
//   Branch 1: Front aperture     (Node 1 -> Node 0, shunt)
//   For section 1:
//     Branch 2: TL segment p1    (Node 1 -> Node 2)  [obs P1]
//     Branch 3: TL segment d1-p1 (Node 2 -> Node 3)
//   For section i > 1:
//     Branch: Inter-section aperture (Node_start -> Node 0, shunt)
//     Branch: TL segment p_i        (Node_start -> Node_obs)
//     Branch: TL segment d_i - p_i  (Node_obs -> Node_end)
//
// Last section's final TL connects to Node 0 (ground = shorted back wall).
//
// Formula:
//   num_branches = 3*N + 1
//   num_nodes    = 2*N (non-ground)
//   Observation point for section i at Node 2*i
// ============================================================================

class CircuitGenerator {
public:

    // ========================================================================
    // GENERATE CIRCUIT
    // ========================================================================
    // Takes an enclosure configuration and populates an MNASolver with
    // the appropriate branches. Returns observation points.
    //
    // The solver should be empty (freshly constructed) before calling this.
    // ========================================================================

    static std::vector<ObservationPoint> generate(
        const EnclosureConfig& config,
        MNASolver& solver,
        bool verbose = false)
    {
        // Validate configuration
        std::string error_msg;
        if (!config.isValid(error_msg)) {
            throw std::invalid_argument("CircuitGenerator: " + error_msg);
        }

        const int N = static_cast<int>(config.sections.size());
        std::vector<ObservationPoint> obs_points;
        int bid = 0;        // Branch ID counter
        int node = 1;       // Current node counter (starts at 1; 0 = ground)

        if (verbose) {
            std::cout << "\n" << std::string(60, '=') << "\n";
            std::cout << "CircuitGenerator: Building " << N << "-section cascade\n";
            std::cout << std::string(60, '=') << "\n";
            std::cout << "Enclosure: a=" << config.a*1000 << "mm, b="
                      << config.b*1000 << "mm, t=" << config.t*1000 << "mm\n";
            std::cout << "Source: Zs=" << std::setprecision(3) << config.Z_source
                      << " Ohm, V0=" << config.V_source << " V\n\n";
        }

        // ==================================================================
        // Branch 0: Source (Node 0 -> Node 1)
        // ==================================================================
        auto src = std::make_shared<SRC_VoltageSource>(
            0, node, bid++,
            Complex(config.V_source, 0.0),
            config.Z_source);
        solver.addBranch(src);

        if (verbose) {
            std::cout << "  Branch " << (bid-1) << ": Source (0 -> " << node << ")\n";
        }

        // ==================================================================
        // Branch 1: Front aperture (Node 1 -> Node 0, shunt)
        // ==================================================================
        const auto& sec0 = config.sections[0];
        addAperture(config, sec0, node, 0, bid, solver, verbose);

        // ==================================================================
        // Loop over sections
        // ==================================================================
        for (int i = 0; i < N; ++i) {
            const auto& sec = config.sections[i];

            if (verbose) {
                std::cout << "  --- Section " << (i+1)
                << ": d=" << sec.depth*1000 << "mm"
                << ", p=" << sec.obs_position*1000 << "mm ---\n";
            }

            // Inter-section aperture (sections 2, 3, ... have aperture at their front)
            if (i > 0) {
                addAperture(config, sec, node, 0, bid, solver, verbose);
            }

            // TL segment: p_i (node -> node+1 = observation point)
            int node_obs = node + 1;
            addTL(config, sec, sec.obs_position, node, node_obs, bid, solver, verbose);

            // Record observation point
            if (sec.has_observation) {
                ObservationPoint op;
                op.node_id = node_obs;
                op.section_index = i;
                op.label = "P" + std::to_string(i + 1);
                obs_points.push_back(op);

                if (verbose) {
                    std::cout << "    ** Observation " << op.label
                              << " at Node " << node_obs << "\n";
                }
            }

            // TL segment: d_i - p_i
            double L_remainder = sec.depth - sec.obs_position;
            int node_end;

            if (i < N - 1) {
                // Not last section: connect to next junction node
                node_end = node_obs + 1;
            } else {
                // Last section: connect to ground (shorted back wall)
                node_end = 0;
            }

            addTL(config, sec, L_remainder, node_obs, node_end, bid, solver, verbose);

            // Advance node counter
            if (i < N - 1) {
                node = node_obs + 1;  // Junction node for next section
            }
        }

        // ==================================================================
        // Summary
        // ==================================================================
        if (verbose) {
            std::cout << "\nCircuit summary:\n";
            std::cout << "  Branches: " << solver.getNumBranches()
                      << " (expected: " << (3*N + 1) << ")\n";
            std::cout << "  Nodes: " << solver.getNumNodes()
                      << " (expected: " << (2*N) << ")\n";
            std::cout << "  Observation points: " << obs_points.size() << "\n";
            for (const auto& op : obs_points) {
                std::cout << "    " << op.label << " -> Node " << op.node_id
                          << " (Section " << (op.section_index + 1) << ")\n";
            }
            std::cout << std::string(60, '=') << "\n\n";
        }

        // Verify counts
        int expected_branches = 3 * N + 1;
        int expected_nodes = 2 * N;
        if (solver.getNumBranches() != expected_branches) {
            std::cerr << "WARNING: Branch count " << solver.getNumBranches()
            << " != expected " << expected_branches << "\n";
        }
        if (solver.getNumNodes() != expected_nodes) {
            std::cerr << "WARNING: Node count " << solver.getNumNodes()
            << " != expected " << expected_nodes << "\n";
        }

        return obs_points;
    }

    // ========================================================================
    // COMPUTE SE AT OBSERVATION POINTS
    // ========================================================================
    // Convenience function: solve MNA at given frequency and return SE [dB]
    // for each observation point.
    // ========================================================================

    static std::vector<double> computeSE(
        MNASolver& solver,
        const std::vector<ObservationPoint>& obs_points,
        double f_Hz)
    {
        auto U = solver.solve(f_Hz);
        std::vector<double> SE_values;
        SE_values.reserve(obs_points.size());

        for (const auto& op : obs_points) {
            // Node IDs are 1-based, but U vector is 0-indexed
            // U(0) = V at Node 1, U(1) = V at Node 2, etc.
            int u_index = op.node_id - 1;
            Complex V_obs = U(u_index);
            double se = -20.0 * std::log10(std::abs(2.0 * V_obs));
            SE_values.push_back(se);
        }

        return SE_values;
    }

private:

    // ========================================================================
    // HELPER: Add aperture branch
    // ========================================================================
    static void addAperture(
        const EnclosureConfig& config,
        const SectionConfig& sec,
        int node_from, int node_to,
        int& bid,
        MNASolver& solver,
        bool verbose)
    {
        if (sec.has_cover) {
            auto ap = std::make_shared<AP_SlotWithCover>(
                node_from, node_to, bid++,
                config.a, config.b,
                sec.aperture_l, sec.aperture_w,
                config.t, sec.cover_gap);
            solver.addBranch(ap);

            if (verbose) {
                std::cout << "  Branch " << (bid-1) << ": SlotWithCover ("
                          << node_from << " -> " << node_to << ") l="
                          << sec.aperture_l*1000 << "mm, tau="
                          << sec.cover_gap*1000 << "mm\n";
            }
        } else {
            auto ap = std::make_shared<AP_SlotAperture>(
                node_from, node_to, bid++,
                config.a, config.b,
                sec.aperture_l, sec.aperture_w,
                config.t);
            solver.addBranch(ap);

            if (verbose) {
                std::cout << "  Branch " << (bid-1) << ": SlotAperture ("
                          << node_from << " -> " << node_to << ") l="
                          << sec.aperture_l*1000 << "mm\n";
            }
        }
    }

    // ========================================================================
    // HELPER: Add transmission line branch
    // ========================================================================
    static void addTL(
        const EnclosureConfig& config,
        const SectionConfig& sec,
        double length,
        int node_from, int node_to,
        int& bid,
        MNASolver& solver,
        bool verbose)
    {
        if (sec.has_dielectric) {
            auto tl = std::make_shared<TL_DielectricCavity>(
                node_from, node_to, bid++,
                config.a, config.b, length,
                sec.dielectric_h, sec.dielectric_er);
            solver.addBranch(tl);

            if (verbose) {
                std::cout << "  Branch " << (bid-1) << ": DielectricTL ("
                          << node_from << " -> " << node_to << ") L="
                          << length*1000 << "mm, h=" << sec.dielectric_h*1000
                          << "mm, er=" << sec.dielectric_er << "\n";
            }
        } else {
            auto tl = std::make_shared<TL_EmptyCavity>(
                node_from, node_to, bid++,
                config.a, config.b, length);
            solver.addBranch(tl);

            if (verbose) {
                std::cout << "  Branch " << (bid-1) << ": EmptyCavityTL ("
                          << node_from << " -> " << node_to << ") L="
                          << length*1000 << "mm\n";
            }
        }
    }
};

} // namespace EMCore

#endif // CIRCUITGENERATOR_H
