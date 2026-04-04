// ============================================================================
// main.cpp — EMShieldDesigner application entry point
// ============================================================================
// Responsibilities:
//   1. Initialise the Qt application object (HiDPI, metadata).
//   2. Show the StartupWindow; hand off to MainWindow on user confirmation.
//   3. Provide a compile-time validation suite (EMSHIELD_RUN_TESTS) that
//      reproduces the MATLAB reference values used during development.
//   4. Wrap the event loop in a top-level try/catch so unhandled C++
//      exceptions produce a human-readable message instead of a silent crash.
//
// Build modes:
//   Release / normal:  compile without EMSHIELD_RUN_TESTS defined.
//                      The GUI launches normally; no console output.
//   Validation build:  add  -DEMSHIELD_RUN_TESTS  to the compiler flags.
//                      The Qt event loop is NOT entered; the validation suite
//                      runs on stdout and the process exits with 0 (all pass)
//                      or 1 (at least one test failed).
// ============================================================================

#ifdef _WIN32
#  include <windows.h>   // SetConsoleOutputCP — UTF-8 console on Windows
#endif

#include "../mainwindow.h"
#include "../StartupWindow.h"

// Core engine
#include "../include/core/PhysicsConstants.h"
#include "../include/core/CircuitGenerator.h"
#include "../include/core/MNASolver.h"

#include <QApplication>
#include <QMessageBox>

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <stdexcept>

using namespace EMCore;

// ============================================================================
// SECTION 1 — VALIDATION SUITE
// ============================================================================
// Compiled only when -DEMSHIELD_RUN_TESTS is defined.
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │  IMPORTANT — REFERENCE VALUE COMPATIBILITY NOTE                        │
// │                                                                         │
// │  All MATLAB reference values below (24.142022 dB, 23.971428 dB, etc.) │
// │  were produced with:                                                    │
// │    c = 3.0×10⁸ m/s  (rounded)                                          │
// │    fstep = (f_stop − f_start) / num_points   (NOT the /(n−1) linspace) │
// │                                                                         │
// │  The corrected PhysicsConstants.h uses C_LIGHT = 299,792,458 m/s      │
// │  (exact SI definition).  This shifts every cutoff frequency and wave-  │
// │  guide propagation constant by +0.069 %, causing SE values near wave-  │
// │  guide resonances to deviate from the reference values by up to        │
// │  several dB.  The numerical tests below will therefore report FAIL     │
// │  until the MATLAB reference suite is re-run with:                      │
// │    physconst('LightSpeed')  instead of  3e8                            │
// │  and the updated reference values are substituted here.                │
// │                                                                         │
// │  The frequency step formula (/ num_points, NOT /(num_points−1)) is    │
// │  intentionally preserved in these tests so that each test evaluates    │
// │  the engine at exactly the same frequency as the MATLAB reference.     │
// │  Do NOT change it to /(n−1) here; that change belongs only in the      │
// │  GUI sweep (mainwindow.cpp) where inclusive linspace is desired.       │
// └─────────────────────────────────────────────────────────────────────────┘

#ifdef EMSHIELD_RUN_TESTS

// ---------------------------------------------------------------------------
// Helper: run one SE test and report PASS / FAIL
// ---------------------------------------------------------------------------
static bool checkSE(const std::string& label,
                    double             computed,
                    double             reference,
                    double             tolerance = 0.001)
{
    const double diff = std::abs(computed - reference);
    const bool   pass = diff < tolerance;
    std::cout << "    " << label
              << ": " << std::fixed << std::setprecision(6) << computed << " dB"
              << "  (ref " << reference << " dB"
              << ",  Δ = " << std::setprecision(6) << diff << " dB)"
              << "  " << (pass ? "PASS ✓" : "FAIL ✗") << "\n";
    return pass;
}

// ---------------------------------------------------------------------------
// Validation frequency: "Point 53" in the 200-point sweep
//
// Definition matches the original MATLAB reference dataset:
//   f_step = (f_stop − f_start) / num_points    ← NOT /(n−1) — see note above
//   f_53   = f_start + 52 × f_step
// ---------------------------------------------------------------------------
static double point53Frequency()
{
    constexpr double f_start    = 1.0e6;
    constexpr double f_stop     = 2.0e9;
    constexpr int    num_points = 200;
    const double     fstep      = (f_stop - f_start) / num_points;
    return f_start + 52.0 * fstep;
}

// ---------------------------------------------------------------------------
// TEST 1 — Single-section (1-Section baseline)
//
// Reference (c = 3e8):  SE_P1 @ Point 53 = 24.142022 dB
// ---------------------------------------------------------------------------
static bool testOneSectionBaseline()
{
    std::cout << "\n--- TEST 1: 1-Section baseline ---\n";

    EnclosureConfig cfg;
    cfg.a = 0.300;  cfg.b = 0.120;  cfg.t = 0.0015;
    cfg.topology = TopologyType::CASCADE;

    SectionConfig sec;
    sec.depth         = 0.300;
    sec.obs_position  = 0.150;
    sec.has_observation = true;
    sec.aperture_l    = 0.08;
    sec.aperture_w    = 0.08;
    cfg.sections      = { sec };

    MNASolver solver;
    const auto obs = CircuitGenerator::generate(cfg, solver, false);

    const double f53 = point53Frequency();
    const auto   SE  = CircuitGenerator::computeSE(solver, obs, cfg, f53);

    std::cout << "  Point 53 — f = " << std::fixed << std::setprecision(3)
              << f53 / 1e6 << " MHz\n";
    return checkSE("SE_P1", SE[0], 24.142022);
}

// ---------------------------------------------------------------------------
// TEST 2 — Two-section identical cascade (Figure 3.10)
//
// Reference (c = 3e8):
//   SE_P1 @ Point 53 = 23.971428 dB
//   SE_P2 @ Point 53 = 58.362361 dB
// ---------------------------------------------------------------------------
static bool testTwoSectionIdentical()
{
    std::cout << "\n--- TEST 2: 2-Section identical (Figure 3.10) ---\n";

    EnclosureConfig cfg;
    cfg.a = 0.300;  cfg.b = 0.120;  cfg.t = 0.0015;
    cfg.topology = TopologyType::CASCADE;

    SectionConfig sec;
    sec.depth         = 0.300;
    sec.obs_position  = 0.150;
    sec.has_observation = true;
    sec.aperture_l    = 0.08;
    sec.aperture_w    = 0.08;
    cfg.sections      = { sec, sec };

    MNASolver solver;
    const auto obs = CircuitGenerator::generate(cfg, solver, false);

    const double f53 = point53Frequency();
    const auto   SE  = CircuitGenerator::computeSE(solver, obs, cfg, f53);

    std::cout << "  Point 53 — f = " << std::fixed << std::setprecision(3)
              << f53 / 1e6 << " MHz\n";
    const bool p1 = checkSE("SE_P1", SE[0], 23.971428);
    const bool p2 = checkSE("SE_P2", SE[1], 58.362361);

    // Full sweep — mean SE summary
    constexpr int    n_pts   = 200;
    constexpr double f_start = 1.0e6;
    constexpr double f_stop  = 2.0e9;
    const double     fstep   = (f_stop - f_start) / n_pts;

    std::vector<double> mean_acc(obs.size(), 0.0);
    for (int i = 0; i < n_pts; ++i) {
        const double f  = f_start + i * fstep;
        const auto   se = CircuitGenerator::computeSE(solver, obs, cfg, f);
        for (std::size_t oi = 0; oi < obs.size(); ++oi)
            mean_acc[oi] += se[oi];
    }
    std::cout << "  Full-sweep mean SE (200 pts):\n";
    for (std::size_t oi = 0; oi < obs.size(); ++oi) {
        std::cout << "    SE_" << obs[oi].label << " mean = "
                  << std::fixed << std::setprecision(2)
                  << mean_acc[oi] / n_pts << " dB\n";
    }
    std::cout << "    MATLAB reference means: P1 ≈ 16.03 dB, P2 ≈ 34.62 dB\n";

    return p1 && p2;
}

// ---------------------------------------------------------------------------
// TEST 3 — Three-section identical cascade (Figure 3.10, N=3)
//
// Reference (c = 3e8):
//   SE_P1 @ Point 53 = 23.971364 dB
//   SE_P2 @ Point 53 = 58.191795 dB
//   SE_P3 @ Point 53 = 92.582727 dB
// ---------------------------------------------------------------------------
static bool testThreeSectionIdentical()
{
    std::cout << "\n--- TEST 3: 3-Section identical (Figure 3.10, N=3) ---\n";

    EnclosureConfig cfg;
    cfg.a = 0.300;  cfg.b = 0.120;  cfg.t = 0.0015;
    cfg.topology = TopologyType::CASCADE;

    SectionConfig sec;
    sec.depth         = 0.300;
    sec.obs_position  = 0.150;
    sec.has_observation = true;
    sec.aperture_l    = 0.08;
    sec.aperture_w    = 0.08;
    cfg.sections      = { sec, sec, sec };

    MNASolver solver;
    const auto obs = CircuitGenerator::generate(cfg, solver, false);

    const double f53 = point53Frequency();
    const auto   SE  = CircuitGenerator::computeSE(solver, obs, cfg, f53);

    std::cout << "  Point 53 — f = " << std::fixed << std::setprecision(3)
              << f53 / 1e6 << " MHz\n";
    const bool p1 = checkSE("SE_P1", SE[0], 23.971364);
    const bool p2 = checkSE("SE_P2", SE[1], 58.191795);
    const bool p3 = checkSE("SE_P3", SE[2], 92.582727);

    constexpr int    n_pts   = 200;
    constexpr double f_start = 1.0e6;
    constexpr double f_stop  = 2.0e9;
    const double     fstep   = (f_stop - f_start) / n_pts;

    std::vector<double> mean_acc(obs.size(), 0.0);
    for (int i = 0; i < n_pts; ++i) {
        const double f  = f_start + i * fstep;
        const auto   se = CircuitGenerator::computeSE(solver, obs, cfg, f);
        for (std::size_t oi = 0; oi < obs.size(); ++oi)
            mean_acc[oi] += se[oi];
    }
    std::cout << "  Full-sweep mean SE (200 pts):\n";
    for (std::size_t oi = 0; oi < obs.size(); ++oi) {
        std::cout << "    SE_" << obs[oi].label << " mean = "
                  << std::fixed << std::setprecision(2)
                  << mean_acc[oi] / n_pts << " dB\n";
    }
    std::cout << "    MATLAB reference means: P1 ≈ 16.09 dB,"
                 " P2 ≈ 34.73 dB, P3 ≈ 53.12 dB\n";

    return p1 && p2 && p3;
}

// ---------------------------------------------------------------------------
// TEST 4 — Two-section cascade with different parameters
//
// Reference (c = 3e8):
//   SE_P1 @ Point 53 = 24.093145 dB
//   SE_P2 @ Point 53 = 69.811831 dB
// ---------------------------------------------------------------------------
static bool testTwoSectionDifferent()
{
    std::cout << "\n--- TEST 4: 2-Section different parameters ---\n";

    EnclosureConfig cfg;
    cfg.a = 0.300;  cfg.b = 0.120;  cfg.t = 0.0015;
    cfg.topology = TopologyType::CASCADE;

    SectionConfig sec1;
    sec1.depth         = 0.300;
    sec1.obs_position  = 0.150;
    sec1.has_observation = true;
    sec1.aperture_l    = 0.08;
    sec1.aperture_w    = 0.08;

    SectionConfig sec2;
    sec2.depth         = 0.200;
    sec2.obs_position  = 0.100;
    sec2.has_observation = true;
    sec2.aperture_l    = 0.05;
    sec2.aperture_w    = 0.05;

    cfg.sections = { sec1, sec2 };

    MNASolver solver;
    const auto obs = CircuitGenerator::generate(cfg, solver, false);

    const double f53 = point53Frequency();
    const auto   SE  = CircuitGenerator::computeSE(solver, obs, cfg, f53);

    std::cout << "  Point 53 — f = " << std::fixed << std::setprecision(3)
              << f53 / 1e6 << " MHz\n";
    const bool p1 = checkSE("SE_P1", SE[0], 24.093145);
    const bool p2 = checkSE("SE_P2", SE[1], 69.811831);

    constexpr int    n_pts   = 200;
    constexpr double f_start = 1.0e6;
    constexpr double f_stop  = 2.0e9;
    const double     fstep   = (f_stop - f_start) / n_pts;

    std::vector<double> mean_acc(obs.size(), 0.0);
    for (int i = 0; i < n_pts; ++i) {
        const double f  = f_start + i * fstep;
        const auto   se = CircuitGenerator::computeSE(solver, obs, cfg, f);
        for (std::size_t oi = 0; oi < obs.size(); ++oi)
            mean_acc[oi] += se[oi];
    }
    std::cout << "  Full-sweep mean SE (200 pts):\n";
    for (std::size_t oi = 0; oi < obs.size(); ++oi) {
        std::cout << "    SE_" << obs[oi].label << " mean = "
                  << std::fixed << std::setprecision(2)
                  << mean_acc[oi] / n_pts << " dB\n";
    }
    std::cout << "    MATLAB reference means: P1 ≈ 15.93 dB, P2 ≈ 47.70 dB\n";

    return p1 && p2;
}

// ---------------------------------------------------------------------------
// Run all validation tests; return 0 if all pass, 1 otherwise
// ---------------------------------------------------------------------------
static int runValidationSuite()
{
    std::cout << "\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << "EMShieldDesigner — Physics Engine Validation Suite\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << "\nC_LIGHT = " << std::fixed << std::setprecision(0)
              << C_LIGHT << " m/s\n";
    std::cout << "Z_0     = " << std::setprecision(6) << Z_0 << " Ω\n";
    std::cout << "Point 53 frequency = " << std::setprecision(6)
              << point53Frequency() / 1e6 << " MHz\n";
    std::cout << "\nNOTE: Reference values below were computed with c = 3×10⁸ m/s.\n";
    std::cout << "      FAIL results are expected until MATLAB references are\n";
    std::cout << "      re-computed with physconst('LightSpeed') = 299792458 m/s.\n";
    std::cout << std::string(70, '-') << "\n";

    bool all_pass = true;

    try { all_pass &= testOneSectionBaseline();    }
    catch (const std::exception& e)
    { std::cerr << "  EXCEPTION: " << e.what() << "\n"; all_pass = false; }

    try { all_pass &= testTwoSectionIdentical();   }
    catch (const std::exception& e)
    { std::cerr << "  EXCEPTION: " << e.what() << "\n"; all_pass = false; }

    try { all_pass &= testThreeSectionIdentical(); }
    catch (const std::exception& e)
    { std::cerr << "  EXCEPTION: " << e.what() << "\n"; all_pass = false; }

    try { all_pass &= testTwoSectionDifferent();   }
    catch (const std::exception& e)
    { std::cerr << "  EXCEPTION: " << e.what() << "\n"; all_pass = false; }

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "Overall result: " << (all_pass ? "ALL PASS ✓" : "FAILURES DETECTED ✗")
              << "\n";
    std::cout << std::string(70, '=') << "\n\n";

    return all_pass ? 0 : 1;
}

#endif // EMSHIELD_RUN_TESTS

// ============================================================================
// SECTION 2 — APPLICATION ENTRY POINT
// ============================================================================
int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

#ifdef EMSHIELD_RUN_TESTS
    return runValidationSuite();
#endif

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication app(argc, argv);
    app.setApplicationName("EMShieldDesigner");
    app.setApplicationDisplayName("EMShield Designer");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("TUSUR");

    // ── FIX 1 ───────────────────────────────────────────────────────────────
    // Prevent Qt from quitting automatically when MainWindow closes.
    // Must be set before any window is created.
    app.setQuitOnLastWindowClosed(false);
    // ────────────────────────────────────────────────────────────────────────

    try {
        auto* startup = new StartupWindow;

        // ── FIX 2 ───────────────────────────────────────────────────────────
        // Quick Simulation: show MainWindow, hide startup.
        // When MainWindow is destroyed, bring startup back.
        // deleteLater() removed — startup must stay alive.
        QObject::connect(startup, &StartupWindow::quickSimulationClicked,
                         [startup]()
                         {
                             startup->hide();

                             auto* mw = new MainWindow;
                             mw->setAttribute(Qt::WA_DeleteOnClose);

                             // MainWindow closed → startup reappears
                             QObject::connect(mw, &QObject::destroyed,
                                              startup, [startup]()
                                              {
                                                  startup->show();
                                              });

                             mw->show();
                         });
        // ────────────────────────────────────────────────────────────────────

        // ── FIX 3 ───────────────────────────────────────────────────────────
        // Closing the startup window itself quits the application cleanly.
        QObject::connect(startup, &StartupWindow::startupClosed,
                         &app, &QApplication::quit);
        // ────────────────────────────────────────────────────────────────────

        startup->show();
        return app.exec();

    } catch (const std::exception& e) {
        QMessageBox::critical(
            nullptr,
            "Fatal Error — EMShieldDesigner",
            QString("An unhandled exception reached the application boundary:\n\n")
                + QString::fromStdString(e.what())
                + "\n\nThe application will now exit.");
        return 1;

    } catch (...) {
        QMessageBox::critical(
            nullptr,
            "Fatal Error — EMShieldDesigner",
            "An unknown exception reached the application boundary.\n\n"
            "The application will now exit.");
        return 1;
    }
}
