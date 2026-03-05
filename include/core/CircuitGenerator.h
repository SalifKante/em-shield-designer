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
// TOPOLOGY SELECTOR
// ============================================================================
// Selects the circuit topology used by CircuitGenerator::generate().
//
// CASCADE (Figure 3.10b):
//   N sections arranged in depth — each section sits behind the previous one
//   in the propagation direction. All sections share the same cross-section
//   width a and height b.
//
//   Circuit layout (N=2):
//     Source → ap¹ → [TL_p₁ → P₁ → TL_{d₁−p₁}] → ap² → [TL_p₂ → P₂ → TL_{d₂−p₂}] → GND
//
//   Branch count: 3N + 1
//   Node count:   2N  (non-ground)
//   Eq. (3.25) Y-matrix applies.
//
// STAR_BRANCH (Figure 3.11b):
//   Section 1 is the "spine" (the primary section that faces the incident
//   wave). Sections 2 … N are lateral sub-chambers that each couple
//   directly to section 1's output junction through their own aperture.
//   Sections 2 … N are independent of each other; they share no common
//   aperture and connect only through section 1.
//
//   Circuit layout (N=3):
//
//     Source → ap¹ → [TL_p₁ → P₁ → TL_{d₁−p₁}] → JUNCTION (node J)
//                                                          │
//                                    ┌────────────ap²→[TL_p₂ → P₂ → TL_{d₂−p₂}]→ GND
//                                    └────────────ap³→[TL_p₃ → P₃ → TL_{d₃−p₃}]→ GND
//
//   Branch count: 3N + 1  (same formula as CASCADE)
//   Node count:   N + 2   (non-ground):
//                   node 1       = source input / ap¹ shunt
//                   node 2       = P₁ (spine obs)
//                   node J=3     = junction (spine output)
//                   nodes 4…N+2  = P₂ … Pₙ (one per side branch)
//
//   Note: Sections 2 … N may each carry a different enclosure width aᵢ,
//   set via SectionConfig::section_width_a.
//   Eq. (3.26) Y-matrix structure applies.
// ============================================================================

enum class TopologyType {
    CASCADE,        ///< N sections cascaded in depth (Figure 3.10)
    STAR_BRANCH     ///< Section 1 spine + N−1 lateral sub-chambers (Figure 3.11)
};

// ============================================================================
// SECTION CONFIGURATION
// ============================================================================

struct SectionConfig {

    // --- Cavity geometry ---
    double depth           = 0.300;    ///< dᵢ: section depth [m]
    double obs_position    = 0.150;    ///< pᵢ: observation point offset from section
    ///<     front wall [m].  Must satisfy
    ///<     0 < pᵢ < dᵢ  (strictly positive,
    ///<     strictly less than depth).
    bool   has_observation = true;     ///< Record SE at this observation point

    // --- Optional: per-section cross-section width override ---
    // If section_width_a <= 0, EnclosureConfig::a is used (default).
    // Set a positive value here to support sections with different widths
    // (STAR_BRANCH topology, Figure 3.11).
    double section_width_a = -1.0;     ///< Override enclosure broad dimension [m]

    // --- Aperture parameters ---
    double aperture_l      = 0.080;    ///< lᵢ: aperture width [m]
    double aperture_w      = 0.080;    ///< wᵢ: aperture height [m]

    // --- Optional: aperture cover (Eqs. 3.22 / 3.24) ---
    bool   has_cover       = false;
    double cover_gap       = 0.001;    ///< τ: cover gap thickness [m]
    double cover_eps_r     = 1.0;      ///< εᵣ of gap filler (1.0 = air)

    // --- Optional: dielectric fill (Eqs. 3.19 – 3.21) ---
    bool   has_dielectric  = false;
    double dielectric_h    = 0.0;      ///< h: dielectric layer thickness [m]
    double dielectric_er   = 1.0;      ///< εᵣ of dielectric (must be ≥ 1.0)

    // -----------------------------------------------------------------------
    // Validation
    // -----------------------------------------------------------------------
    bool isValid(std::string& error_msg) const {

        if (depth <= 0.0) {
            error_msg = "Section depth must be positive";
            return false;
        }
        // obs_position must be strictly inside (0, depth) so that neither
        // TL segment has zero length (which TL_EmptyCavity rejects).
        if (obs_position <= 0.0) {
            error_msg = "Observation position must be strictly positive (obs_position > 0)";
            return false;
        }
        if (obs_position >= depth) {
            error_msg = "Observation position must be strictly less than depth (obs_position < depth)";
            return false;
        }
        if (aperture_l <= 0.0 || aperture_w <= 0.0) {
            error_msg = "Aperture dimensions must be positive";
            return false;
        }
        if (has_cover) {
            if (cover_gap <= 0.0) {
                error_msg = "Cover gap must be positive when cover is enabled";
                return false;
            }
            if (cover_eps_r < 1.0) {
                error_msg = "Cover filler permittivity must be >= 1.0";
                return false;
            }
        }
        if (has_dielectric) {
            if (dielectric_h <= 0.0) {
                error_msg = "Dielectric thickness must be positive";
                return false;
            }
            // TL_DielectricCavity requires epsilon_r >= 1.0 (not merely > 0)
            if (dielectric_er < 1.0) {
                error_msg = "Dielectric permittivity must be >= 1.0";
                return false;
            }
        }
        return true;
    }
};

// ============================================================================
// ENCLOSURE CONFIGURATION
// ============================================================================

struct EnclosureConfig {

    // --- Shared enclosure cross-section ---
    double a = 0.300;                  ///< Broad dimension (width) [m]
    double b = 0.120;                  ///< Narrow dimension (height) [m]
    double t = 0.0015;                 ///< Front wall thickness [m]

    // --- Source ---
    double Z_source = Z_0;             ///< Source impedance [Ω] (default: 120π)
    double V_source = 1.0;             ///< Source voltage magnitude [V]

    // --- Topology ---
    TopologyType topology = TopologyType::CASCADE;

    // --- Sections ---
    std::vector<SectionConfig> sections;

    // -----------------------------------------------------------------------
    // Validation
    // -----------------------------------------------------------------------
    bool isValid(std::string& error_msg) const {
        if (a <= 0.0 || b <= 0.0) {
            error_msg = "Enclosure dimensions (a, b) must be positive";
            return false;
        }
        if (t <= 0.0) {
            error_msg = "Wall thickness must be positive";
            return false;
        }
        if (Z_source <= 0.0) {
            error_msg = "Source impedance must be positive";
            return false;
        }
        if (sections.empty()) {
            error_msg = "At least one section is required";
            return false;
        }
        if (topology == TopologyType::STAR_BRANCH && sections.size() < 2) {
            error_msg = "STAR_BRANCH topology requires at least 2 sections "
                        "(1 spine + 1 side branch)";
            return false;
        }
        for (std::size_t i = 0; i < sections.size(); ++i) {
            std::string sec_err;
            if (!sections[i].isValid(sec_err)) {
                error_msg = "Section " + std::to_string(i + 1) + ": " + sec_err;
                return false;
            }
        }
        return true;
    }
};

// ============================================================================
// OBSERVATION POINT
// ============================================================================

struct ObservationPoint {
    int         node_id;        ///< MNA node index (1-based; 0 = ground)
    int         section_index;  ///< Section index (0-based)
    std::string label;          ///< Display label, e.g. "P1", "P2"
};

// ============================================================================
// CIRCUIT GENERATOR
// ============================================================================
// Converts an EnclosureConfig into a fully populated MNASolver and returns
// the list of observation points.
//
// Two topology generators are provided:
//   generateCascade()    — Figure 3.10 / Eq. (3.25)
//   generateStarBranch() — Figure 3.11 / Eq. (3.26)
//
// Both are dispatched automatically by generate() based on
// EnclosureConfig::topology.
// ============================================================================

class CircuitGenerator {
public:

    // ========================================================================
    // PRIMARY ENTRY POINT
    // ========================================================================

    /**
     * @brief Populate solver with branches for the given enclosure config.
     *
     * The solver must be empty (freshly constructed or cleared) before calling.
     * Dispatches to generateCascade() or generateStarBranch() based on
     * config.topology.
     *
     * @param config   Validated enclosure configuration.
     * @param solver   Empty MNASolver to populate.
     * @param verbose  Print circuit layout to std::cout if true.
     * @returns        List of observation points (node_id, section, label).
     * @throws std::invalid_argument on invalid config.
     */
    static std::vector<ObservationPoint> generate(
        const EnclosureConfig& config,
        MNASolver&             solver,
        bool                   verbose = false)
    {
        std::string error_msg;
        if (!config.isValid(error_msg)) {
            throw std::invalid_argument("CircuitGenerator::generate: " + error_msg);
        }

        switch (config.topology) {
        case TopologyType::CASCADE:
            return generateCascade(config, solver, verbose);
        case TopologyType::STAR_BRANCH:
            return generateStarBranch(config, solver, verbose);
        default:
            throw std::invalid_argument(
                "CircuitGenerator::generate: unknown topology type");
        }
    }

    // ========================================================================
    // SE COMPUTATION — Equation (3.8)
    // ========================================================================
    /**
     * @brief Solve MNA at f_Hz and return SE [dB] for each observation point.
     *
     * Delegates to MNASolver::computeSE() which implements Eq. (3.8):
     *   SE = −20·log₁₀(|2·U_obs / V₀|)
     *
     * V₀ is taken from config.V_source (the source voltage set at construction
     * time). This is mandatory — hardcoding V₀ = 1 would give wrong results
     * whenever config.V_source ≠ 1.
     *
     * @param solver      Populated MNASolver (after generate() has been called).
     * @param obs_points  Observation point list returned by generate().
     * @param config      Enclosure config (provides V₀ = config.V_source).
     * @param f_Hz        Frequency [Hz].
     * @returns           SE [dB] for each observation point, in order.
     */
    static std::vector<double> computeSE(
        MNASolver&                          solver,
        const std::vector<ObservationPoint>& obs_points,
        const EnclosureConfig&              config,
        double                              f_Hz)
    {
        const Eigen::VectorXcd U  = solver.solve(f_Hz);
        const Complex          V0(config.V_source, 0.0);

        std::vector<double> SE_values;
        SE_values.reserve(obs_points.size());

        for (const auto& op : obs_points) {
            // Delegate entirely to MNASolver::computeSE() which encapsulates
            // Eq. (3.8) with all required guards (bounds check, |V0|≈0, perfect shield).
            SE_values.push_back(solver.computeSE(U, op.node_id, V0));
        }
        return SE_values;
    }

private:

    // ========================================================================
    // CASCADE GENERATOR — Figure 3.10 / Eq. (3.25)
    // ========================================================================
    //
    // Topology for N sections:
    //
    //   node_chain: 0→1 (source)
    //   for each section i = 0 … N−1:
    //     ap_i:        node_in  → 0        (shunt to ground)
    //     TL_p_i:      node_in  → node_obs (observation node for P_i)
    //     TL_{d-p}_i:  node_obs → node_out
    //     where node_out = node_obs+1 for i < N−1
    //                    = 0           for i = N−1 (short-circuit back wall)
    //
    // Branch count: 3N + 1
    // Node count:   2N (non-ground)

    static std::vector<ObservationPoint> generateCascade(
        const EnclosureConfig& cfg,
        MNASolver&             solver,
        bool                   verbose)
    {
        const int N = static_cast<int>(cfg.sections.size());
        std::vector<ObservationPoint> obs_points;
        int bid  = 0;
        int node = 1;   // Current input node; advances section by section

        if (verbose) {
            printHeader("CASCADE", N, cfg);
        }

        // --- Branch 0: Source (0 → node 1) ----------------------------------
        solver.addBranch(std::make_shared<SRC_VoltageSource>(
            0, node, bid++,
            Complex(cfg.V_source, 0.0),
            cfg.Z_source));

        if (verbose) {
            std::cout << "  [" << std::setw(2) << (bid-1)
            << "]  Source         0 → " << node << "\n";
        }

        // --- Sections -------------------------------------------------------
        for (int i = 0; i < N; ++i) {
            const SectionConfig& sec = cfg.sections[i];
            const double a_i = (sec.section_width_a > 0.0)
                                   ? sec.section_width_a : cfg.a;

            if (verbose) {
                std::cout << "  --- Section " << (i+1)
                << "  a=" << a_i*1e3 << "mm"
                << "  d=" << sec.depth*1e3 << "mm"
                << "  p=" << sec.obs_position*1e3 << "mm ---\n";
            }

            // Aperture: shunt at node (node → 0)
            addAperture(cfg, sec, a_i, node, 0, bid, solver, verbose);

            // TL segment #1: node → obs_node  (length = p)
            const int obs_node = node + 1;
            addTL(cfg, sec, a_i, sec.obs_position, node, obs_node, bid, solver, verbose);

            // Record observation point
            if (sec.has_observation) {
                obs_points.push_back({ obs_node, i, "P" + std::to_string(i + 1) });
                if (verbose) {
                    std::cout << "      ** P" << (i+1) << " at node " << obs_node << "\n";
                }
            }

            // TL segment #2: obs_node → node_out  (length = d − p)
            const double L_rem   = sec.depth - sec.obs_position;
            const int    node_out = (i < N - 1) ? (obs_node + 1) : 0;  // 0 = ground
            addTL(cfg, sec, a_i, L_rem, obs_node, node_out, bid, solver, verbose);

            // Advance input node for next section
            if (i < N - 1) {
                node = obs_node + 1;
            }
        }

        if (verbose) {
            printSummary(solver, obs_points, 3*N+1, 2*N, "CASCADE");
        }

        verifyCount(solver, obs_points, 3*N+1, 2*N, "CASCADE");
        return obs_points;
    }

    // ========================================================================
    // STAR_BRANCH GENERATOR — Figure 3.11 / Eq. (3.26)
    // ========================================================================
    //
    // Topology:
    //   Section 0 = spine (primary cavity, faces the incident wave).
    //   Sections 1 … N−1 = side branches, each coupling directly to the
    //   junction node at the output of the spine via its own aperture.
    //   Branches are INDEPENDENT: no aperture connects section 2 to section 3.
    //
    // Circuit layout (N=3):
    //   Source (0→1) → ap¹(1→0) → TL_p₁(1→2)[P₁] → TL_{d₁-p₁}(2→3)
    //   From junction node 3:
    //     ap²(3→0) → TL_p₂(3→4)[P₂] → TL_{d₂-p₂}(4→0)
    //     ap³(3→0) → TL_p₃(3→5)[P₃] → TL_{d₃-p₃}(5→0)
    //
    // Branch count: 3N + 1  (same formula as CASCADE)
    // Node count:   N + 2   (non-ground):
    //   node 1       = input / ap¹ shunt
    //   node 2       = P₁ (spine obs)
    //   node 3       = junction (spine output)
    //   nodes 4…N+2  = P₂ … Pₙ (one per side branch)

    static std::vector<ObservationPoint> generateStarBranch(
        const EnclosureConfig& cfg,
        MNASolver&             solver,
        bool                   verbose)
    {
        const int N = static_cast<int>(cfg.sections.size());
        std::vector<ObservationPoint> obs_points;
        int bid = 0;

        if (verbose) {
            printHeader("STAR_BRANCH", N, cfg);
        }

        // --- Branch 0: Source (0 → node 1) ----------------------------------
        solver.addBranch(std::make_shared<SRC_VoltageSource>(
            0, 1, bid++,
            Complex(cfg.V_source, 0.0),
            cfg.Z_source));

        if (verbose) {
            std::cout << "  [" << std::setw(2) << (bid-1)
            << "]  Source         0 → 1\n";
        }

        // --- SPINE (section 0) ----------------------------------------------
        {
            const SectionConfig& spine = cfg.sections[0];
            const double a_spine = (spine.section_width_a > 0.0)
                                       ? spine.section_width_a : cfg.a;

            if (verbose) {
                std::cout << "  --- Spine (Section 1)"
                          << "  a=" << a_spine*1e3 << "mm"
                          << "  d=" << spine.depth*1e3 << "mm"
                          << "  p=" << spine.obs_position*1e3 << "mm ---\n";
            }

            // ap¹: shunt at node 1
            addAperture(cfg, spine, a_spine, 1, 0, bid, solver, verbose);

            // TL_p₁: node 1 → node 2  (P₁)
            addTL(cfg, spine, a_spine, spine.obs_position, 1, 2, bid, solver, verbose);

            if (spine.has_observation) {
                obs_points.push_back({ 2, 0, "P1" });
                if (verbose) {
                    std::cout << "      ** P1 at node 2\n";
                }
            }

            // TL_{d₁-p₁}: node 2 → node 3  (junction)
            const double L_spine_rem = spine.depth - spine.obs_position;
            addTL(cfg, spine, a_spine, L_spine_rem, 2, 3, bid, solver, verbose);

            if (verbose) {
                std::cout << "      Junction at node 3\n";
            }
        }

        // --- SIDE BRANCHES (sections 1 … N−1) --------------------------------
        // Each branch hangs off the junction node (node 3).
        // obs node for branch i (0-based among branches) = 4 + i
        for (int i = 1; i < N; ++i) {
            const SectionConfig& sec = cfg.sections[i];
            const double a_i = (sec.section_width_a > 0.0)
                                   ? sec.section_width_a : cfg.a;

            const int junction = 3;               // All branches share node 3
            const int obs_node = 3 + i;           // node 4, 5, 6, … for P₂, P₃, …

            if (verbose) {
                std::cout << "  --- Branch " << i
                          << " (Section " << (i+1) << ")"
                          << "  a=" << a_i*1e3 << "mm"
                          << "  d=" << sec.depth*1e3 << "mm"
                          << "  p=" << sec.obs_position*1e3 << "mm ---\n";
            }

            // apⁱ: shunt at junction node 3 (3 → 0)
            addAperture(cfg, sec, a_i, junction, 0, bid, solver, verbose);

            // TL_pᵢ: junction → obs_node  (Pᵢ)
            addTL(cfg, sec, a_i, sec.obs_position, junction, obs_node, bid, solver, verbose);

            if (sec.has_observation) {
                obs_points.push_back({ obs_node, i, "P" + std::to_string(i + 1) });
                if (verbose) {
                    std::cout << "      ** P" << (i+1) << " at node " << obs_node << "\n";
                }
            }

            // TL_{dᵢ−pᵢ}: obs_node → 0 (ground = short-circuit end wall)
            const double L_rem = sec.depth - sec.obs_position;
            addTL(cfg, sec, a_i, L_rem, obs_node, 0, bid, solver, verbose);
        }

        const int expected_branches = 3 * N + 1;
        const int expected_nodes    = N + 2;

        if (verbose) {
            printSummary(solver, obs_points, expected_branches, expected_nodes, "STAR_BRANCH");
        }

        verifyCount(solver, obs_points, expected_branches, expected_nodes, "STAR_BRANCH");
        return obs_points;
    }

    // ========================================================================
    // HELPERS: addAperture, addTL
    // ========================================================================

    static void addAperture(
        const EnclosureConfig& cfg,
        const SectionConfig&   sec,
        double                 a_eff,   // effective enclosure width for this section
        int node_from, int node_to,
        int& bid,
        MNASolver& solver,
        bool verbose)
    {
        if (sec.has_cover) {
            // AP_SlotWithCover — air or dielectric gap (Eqs. 3.22 / 3.24)
            std::shared_ptr<BranchTemplate> ap;
            if (sec.cover_eps_r > 1.0 + 1e-9) {
                ap = std::make_shared<AP_SlotWithCover>(
                    node_from, node_to, bid++,
                    a_eff, cfg.b,
                    sec.aperture_l, sec.aperture_w,
                    cfg.t, sec.cover_gap,
                    sec.cover_eps_r);   // Dielectric-filled gap (Eq. 3.24)
            } else {
                ap = std::make_shared<AP_SlotWithCover>(
                    node_from, node_to, bid++,
                    a_eff, cfg.b,
                    sec.aperture_l, sec.aperture_w,
                    cfg.t, sec.cover_gap);              // Air gap (Eq. 3.22)
            }
            solver.addBranch(ap);
            if (verbose) {
                std::cout << "  [" << std::setw(2) << (bid-1)
                << "]  SlotWithCover  " << node_from << " → " << node_to
                << "  l=" << sec.aperture_l*1e3 << "mm"
                << "  τ=" << sec.cover_gap*1e3 << "mm"
                << (sec.cover_eps_r > 1.0 + 1e-9
                        ? "  [Eq.3.24 dielectric]" : "  [Eq.3.22 air]")
                << "\n";
            }
        } else {
            // AP_SlotAperture — open slot (Eq. 3.13)
            solver.addBranch(std::make_shared<AP_SlotAperture>(
                node_from, node_to, bid++,
                a_eff, cfg.b,
                sec.aperture_l, sec.aperture_w,
                cfg.t));
            if (verbose) {
                std::cout << "  [" << std::setw(2) << (bid-1)
                << "]  SlotAperture   " << node_from << " → " << node_to
                << "  l=" << sec.aperture_l*1e3 << "mm"
                << "  [Eq.3.13]\n";
            }
        }
    }

    static void addTL(
        const EnclosureConfig& cfg,
        const SectionConfig&   sec,
        double                 a_eff,
        double                 length,
        int node_from, int node_to,
        int& bid,
        MNASolver& solver,
        bool verbose)
    {
        if (sec.has_dielectric) {
            // TL_DielectricCavity — Eq. (3.19) with Lichtenecker εeff default
            solver.addBranch(std::make_shared<TL_DielectricCavity>(
                node_from, node_to, bid++,
                a_eff, cfg.b, length,
                sec.dielectric_h, sec.dielectric_er));
            if (verbose) {
                std::cout << "  [" << std::setw(2) << (bid-1)
                << "]  DielectricTL   " << node_from << " → " << node_to
                << "  L=" << length*1e3 << "mm"
                << "  h=" << sec.dielectric_h*1e3 << "mm"
                << "  εᵣ=" << sec.dielectric_er
                << "  [Eq.3.19]\n";
            }
        } else {
            // TL_EmptyCavity — Eqs. (3.14)–(3.17)
            solver.addBranch(std::make_shared<TL_EmptyCavity>(
                node_from, node_to, bid++,
                a_eff, cfg.b, length));
            if (verbose) {
                std::cout << "  [" << std::setw(2) << (bid-1)
                << "]  EmptyCavityTL  " << node_from << " → " << node_to
                << "  L=" << length*1e3 << "mm"
                << "  [Eqs.3.14-3.17]\n";
            }
        }
    }

    // ========================================================================
    // HELPERS: verbose output and count verification
    // ========================================================================

    static void printHeader(const std::string& topo_name,
                            int N,
                            const EnclosureConfig& cfg)
    {
        std::cout << "\n" << std::string(64, '=') << "\n";
        std::cout << "CircuitGenerator — " << topo_name
                  << "  N=" << N << " sections\n";
        std::cout << std::string(64, '=') << "\n";
        std::cout << "  Enclosure: a=" << cfg.a*1e3 << "mm"
                  << "  b=" << cfg.b*1e3 << "mm"
                  << "  t=" << cfg.t*1e3 << "mm\n";
        std::cout << "  Source:    Zs=" << cfg.Z_source
                  << " Ω  V0=" << cfg.V_source << " V\n\n";
    }

    static void printSummary(const MNASolver&                      solver,
                             const std::vector<ObservationPoint>&  ops,
                             int expected_branches,
                             int expected_nodes,
                             const std::string& topo_name)
    {
        std::cout << "\n  " << std::string(60, '-') << "\n";
        std::cout << "  " << topo_name << " summary:\n";
        std::cout << "    Branches: " << solver.getNumBranches()
                  << "  (expected " << expected_branches << ")\n";
        std::cout << "    Nodes:    " << solver.getNumNodes()
                  << "  (expected " << expected_nodes << ")\n";
        std::cout << "    Observation points:\n";
        for (const auto& op : ops) {
            std::cout << "      " << op.label
                      << " → node " << op.node_id
                      << "  (section " << (op.section_index+1) << ")\n";
        }
        std::cout << std::string(64, '=') << "\n\n";
    }

    static void verifyCount(const MNASolver& solver,
                            const std::vector<ObservationPoint>& /*ops*/,
                            int expected_branches,
                            int expected_nodes,
                            const std::string& topo_name)
    {
        if (solver.getNumBranches() != expected_branches) {
            std::cerr << "[CircuitGenerator] WARNING (" << topo_name
                      << "): branch count = " << solver.getNumBranches()
                      << ", expected " << expected_branches << ".\n";
        }
        if (solver.getNumNodes() != expected_nodes) {
            std::cerr << "[CircuitGenerator] WARNING (" << topo_name
                      << "): node count = " << solver.getNumNodes()
                      << ", expected " << expected_nodes << ".\n";
        }
    }
};

} // namespace EMCore

#endif // CIRCUITGENERATOR_H
