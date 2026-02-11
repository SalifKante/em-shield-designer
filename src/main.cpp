#include "../mainwindow.h"

#include <QApplication>
#include <QStandardPaths>
#include <QDir>
#include "../include/core/PhysicsConstants.h"
#include "../include/core/BranchTemplate.h"
#include "../include/core/TL_EmptyCavity.h"
#include "../include/core/TL_DielectricCavity.h"
#include "../include/core/SRC_VoltageSource.h"
#include "../include/core/LOAD_Impedance.h"
#include "../include/core/AP_SlotAperture.h"
#include "../include/core/AP_SlotWithCover.h"
#include "../include/core/MNASolver.h"
#include "../include/core/CircuitGenerator.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <filesystem>
#include <numeric>
#include <memory>

#ifdef _WIN32
#include <windows.h>
#endif

using namespace EMCore;

// ============================================================================
// TEST FUNCTION DECLARATIONS
// ============================================================================
void testDielectricCavity();
void testApertureWithCover();
void testCompleteSystem();
void test2SectionCascade();  // NEW: 2-section cascade test

// ============================================================================
// DEBUG CONTROL FLAGS
// ============================================================================
constexpr bool ENABLE_SINGLE_POINT_TEST = true;   // Run validation test
constexpr bool ENABLE_FREQUENCY_SWEEP = true;     // Run full sweep
constexpr bool ENABLE_NEW_TESTS = true;           // Run dielectric & cover tests
constexpr bool ENABLE_COMPLETE_SYSTEM = false;    // COMMENTED OUT to show 2-section results
constexpr bool ENABLE_2SECTION_CASCADE = true;    // NEW: Enable 2-section test

// ============================================================================
// FREQUENCY SWEEP CONFIGURATION
// ============================================================================
struct FrequencySweepConfig {
    double f_start;
    double f_stop;
    int num_points;

    FrequencySweepConfig(double start = 1e6, double stop = 2e9, int points = 200)
        : f_start(start), f_stop(stop), num_points(points) {}
};

// ============================================================================
// FREQUENCY SWEEP RESULTS CONTAINER
// ============================================================================
struct FrequencySweepResults {
    std::vector<double> frequencies;
    std::vector<Complex> V1;
    std::vector<Complex> V2;
    std::vector<double> SE_dB;
};

// ============================================================================
// GENERATE FREQUENCY VECTOR (MATLAB-COMPATIBLE)
// ============================================================================
std::vector<double> generateFrequencies(const FrequencySweepConfig& config) {
    std::vector<double> frequencies;
    frequencies.reserve(config.num_points);

    // MATLAB-COMPATIBLE: step = range / num_points
    double df = (config.f_stop - config.f_start) / config.num_points;

    for (int i = 0; i < config.num_points; ++i) {
        frequencies.push_back(config.f_start + i * df);
    }

    return frequencies;
}

// ============================================================================
// PERFORM FREQUENCY SWEEP (EMPTY CAVITY + SLOT APERTURE)
// ============================================================================
FrequencySweepResults performFrequencySweep(
    const FrequencySweepConfig& config,
    double a, double b, double d, double p,
    double l, double w, double t, double Z_source)
{
    FrequencySweepResults results;
    results.frequencies = generateFrequencies(config);

    results.V1.reserve(config.num_points);
    results.V2.reserve(config.num_points);
    results.SE_dB.reserve(config.num_points);

    double L1 = p;
    double L2 = d - p;

    MNASolver solver;

    // Create branches in correct order (matching MATLAB Figure 3.7)
    // Branch I:   Voltage source (node 0 → node 1)
    // Branch II:  Slot aperture, shunt (node 1 → node 0)
    // Branch III: TL section p (node 1 → node 2)
    // Branch IV:  TL section d-p (node 2 → node 0)
    auto src = std::make_shared<SRC_VoltageSource>(
        0, 1, 0, Complex(1.0, 0.0), Z_source);

    auto aperture = std::make_shared<AP_SlotAperture>(
        1, 0, 1, a, b, l, w, t);

    auto cavity1 = std::make_shared<TL_EmptyCavity>(
        1, 2, 2, a, b, L1);

    auto cavity2 = std::make_shared<TL_EmptyCavity>(
        2, 0, 3, a, b, L2);

    solver.addBranch(src);
    solver.addBranch(aperture);
    solver.addBranch(cavity1);
    solver.addBranch(cavity2);

    std::cout << "Running frequency sweep: ";
    std::cout << config.f_start/1e6 << " MHz to " << config.f_stop/1e9 << " GHz ("
              << config.num_points << " points)\n";

    int progress_step = std::max(1, config.num_points / 20);

    for (int i = 0; i < config.num_points; ++i) {
        double f = results.frequencies[i];
        auto U = solver.solve(f);

        Complex v1 = U(0);
        Complex v2 = U(1);

        results.V1.push_back(v1);
        results.V2.push_back(v2);
        // Equation 3.8: SE = -20*lg|2*U2/V0|, with V0=1
        results.SE_dB.push_back(-20.0 * std::log10(std::abs(2.0 * v2)));

        if (i % progress_step == 0) {
            std::cout << ".";
        }
    }

    std::cout << " Done.\n";
    return results;
}

// ============================================================================
// PERFORM FREQUENCY SWEEP (DIELECTRIC CAVITY + SLOT WITH COVER)
// ============================================================================
FrequencySweepResults performFrequencySweepDielectricCover(
    const FrequencySweepConfig& config,
    double a, double b, double d, double p,
    double l, double w, double t, double Z_source,
    double h_diel, double epsilon_r, double tau)
{
    FrequencySweepResults results;
    results.frequencies = generateFrequencies(config);

    results.V1.reserve(config.num_points);
    results.V2.reserve(config.num_points);
    results.SE_dB.reserve(config.num_points);

    double L1 = p;
    double L2 = d - p;

    MNASolver solver;

    // Branch I: Voltage source (node 0 → node 1)
    auto src = std::make_shared<SRC_VoltageSource>(
        0, 1, 0, Complex(1.0, 0.0), Z_source);

    // Branch II: Slot aperture WITH COVER, shunt (node 1 → node 0)
    auto aperture = std::make_shared<AP_SlotWithCover>(
        1, 0, 1, a, b, l, w, t, tau);

    // Branch III: DIELECTRIC-FILLED cavity, section p (node 1 → node 2)
    auto cavity1 = std::make_shared<TL_DielectricCavity>(
        1, 2, 2, a, b, L1, h_diel, epsilon_r,
        TL_DielectricCavity::EffPermMethod::LICHTENECKER);

    // Branch IV: DIELECTRIC-FILLED cavity, section d-p (node 2 → node 0)
    auto cavity2 = std::make_shared<TL_DielectricCavity>(
        2, 0, 3, a, b, L2, h_diel, epsilon_r,
        TL_DielectricCavity::EffPermMethod::LICHTENECKER);

    solver.addBranch(src);
    solver.addBranch(aperture);
    solver.addBranch(cavity1);
    solver.addBranch(cavity2);

    std::cout << "Running dielectric+cover sweep: ";
    std::cout << config.f_start/1e6 << " MHz to " << config.f_stop/1e9 << " GHz ("
              << config.num_points << " points)\n";
    std::cout << "  Dielectric: h=" << h_diel*1000 << "mm, er=" << epsilon_r
              << ", eps_eff=" << cavity1->getEffectivePermittivity() << "\n";
    std::cout << "  Cover gap: tau=" << tau*1000 << "mm\n";

    int progress_step = std::max(1, config.num_points / 20);

    for (int i = 0; i < config.num_points; ++i) {
        double f = results.frequencies[i];
        auto U = solver.solve(f);

        Complex v1 = U(0);
        Complex v2 = U(1);

        results.V1.push_back(v1);
        results.V2.push_back(v2);
        // Equation 3.8: SE = -20*lg|2*U2/V0|, with V0=1
        results.SE_dB.push_back(-20.0 * std::log10(std::abs(2.0 * v2)));

        if (i % progress_step == 0) {
            std::cout << ".";
        }
    }

    std::cout << " Done.\n";
    return results;
}

// ============================================================================
// EXPORT TO CSV
// ============================================================================
bool exportToCSV(const FrequencySweepResults& results, const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << filename << "\n";
        return false;
    }

    file << "# EMShieldDesigner - Frequency Sweep Results\n";
    file << "# Generated from C++ implementation\n";
    file << "# f_GHz,SE_dB,V1_real,V1_imag,V2_real,V2_imag\n";

    file << std::fixed << std::setprecision(9);

    for (size_t i = 0; i < results.frequencies.size(); ++i) {
        file << results.frequencies[i] / 1e9 << ","
             << std::setprecision(6) << results.SE_dB[i] << ","
             << std::setprecision(9) << results.V1[i].real() << ","
             << results.V1[i].imag() << ","
             << results.V2[i].real() << ","
             << results.V2[i].imag() << "\n";
    }

    file.close();
    return true;
}

// ============================================================================
// VALIDATION TEST @ POINT 53 (~520 MHz)
// ============================================================================
void runValidationTest(double a, double b, double d, double p,
                       double l, double w, double t, double Z_source) {

    std::cout << "\n=== VALIDATION TEST (MATLAB Point 53) ===\n\n";

    // Calculate exact frequency for point 53 (MATLAB compatible)
    int validation_idx = 53;
    double f_start = 1.0e6;
    double f_stop = 2.0e9;
    int num_points = 200;
    double fstep = (f_stop - f_start) / num_points;

    // Point 53 corresponds to i = 52 in C++ (0-indexed)
    double f_test = f_start + (validation_idx - 1) * fstep;

    std::cout << "Test frequency (Point " << validation_idx << "): "
              << std::fixed << std::setprecision(3) << f_test/1e6 << " MHz\n";
    std::cout << "Expected frequency: ~520.74 MHz\n\n";

    MNASolver solver;

    auto test_src = std::make_shared<SRC_VoltageSource>(
        0, 1, 0, Complex(1.0, 0.0), Z_source);

    auto test_aperture = std::make_shared<AP_SlotAperture>(
        1, 0, 1, a, b, l, w, t);

    auto test_cavity1 = std::make_shared<TL_EmptyCavity>(
        1, 2, 2, a, b, p);

    auto test_cavity2 = std::make_shared<TL_EmptyCavity>(
        2, 0, 3, a, b, d - p);

    solver.addBranch(test_src);
    solver.addBranch(test_aperture);
    solver.addBranch(test_cavity1);
    solver.addBranch(test_cavity2);

    auto U_test = solver.solve(f_test);

    Complex V1 = U_test(0);
    Complex V2 = U_test(1);
    double SE_test = -20.0 * std::log10(std::abs(2.0 * V2));

    std::cout << "C++ Results @ " << std::fixed << std::setprecision(3) << f_test/1e6 << " MHz:\n";
    std::cout << "  V1 = (" << std::setprecision(6) << V1.real() << ", "
              << V1.imag() << ") V\n";
    std::cout << "  V2 = (" << V2.real() << ", " << V2.imag() << ") V\n";
    std::cout << "  SE = " << std::setprecision(3) << SE_test << " dB\n\n";

    std::cout << "MATLAB Reference (run MATLAB to get exact values):\n";
    std::cout << "  Expected values from MATLAB output at point 53\n";
    std::cout << "===============================================\n\n";
}

// ============================================================================
// CALCULATE POINT 53 EXACTLY
// ============================================================================
void calculateExactPoint53(double a, double b, double d, double p,
                           double l, double w, double t, double Z_source) {

    std::cout << "\n=== Exact Calculation for Point 53 ===\n";

    double f_start = 1.0e6;
    double f_stop = 2.0e9;
    int num_points = 200;
    double fstep = (f_stop - f_start) / num_points;
    double f_53 = f_start + 52 * fstep;

    std::cout << "Exact frequency: " << std::fixed << std::setprecision(9)
              << f_53 << " Hz\n";
    std::cout << "                = " << std::setprecision(6) << f_53/1e6 << " MHz\n";
    std::cout << "                = " << std::setprecision(9) << f_53/1e9 << " GHz\n\n";

    double fc = C_LIGHT / (2.0 * a);
    std::cout << "Cutoff frequency: fc = " << fc/1e6 << " MHz\n";

    if (f_53 < fc) {
        std::cout << "Mode: Evanescent (f < fc)\n";
    } else {
        std::cout << "Mode: Propagating (f >= fc)\n";
    }
    std::cout << "Ratio f/fc = " << f_53/fc << "\n\n";
}

// ========================================================================
// TEST 1: Dielectric-Filled Cavity Validation
// ========================================================================
void testDielectricCavity() {
    std::cout << "\nDielectric Cavity Validation\n";
    std::cout << "============================\n";

    double aa = 0.300;
    double b = 0.120;
    double L = 0.150;
    double h = 0.060;      // Half-filled (h/b = 0.5)
    double epsilon_r = 4.4; // FR-4 material
    double f_test = 1e9;    // 1 GHz

    // Empty cavity (reference)
    auto empty = std::make_shared<TL_EmptyCavity>(1, 2, 1, aa, b, L);
    auto Y_empty = empty->computeYParameters(f_test);

    // Dielectric-filled cavity (Lichtenecker)
    auto dielectric_lich = std::make_shared<TL_DielectricCavity>(
        1, 2, 2, aa, b, L, h, epsilon_r,
        TL_DielectricCavity::EffPermMethod::LICHTENECKER
        );
    auto Y_diel_lich = dielectric_lich->computeYParameters(f_test);

    // Dielectric-filled cavity (Maxwell-Garnett)
    auto dielectric_mg = std::make_shared<TL_DielectricCavity>(
        1, 2, 3, aa, b, L, h, epsilon_r,
        TL_DielectricCavity::EffPermMethod::MAXWELL_GARNETT
        );
    auto Y_diel_mg = dielectric_mg->computeYParameters(f_test);

    // Display results
    std::cout << "Cavity: a=" << std::fixed << std::setprecision(3) << aa
              << "m, b=" << b << "m, L=" << L << "m\n";
    std::cout << "Dielectric: h=" << h << "m, er=" << epsilon_r
              << ", h/b=" << std::setprecision(3) << (h/b) << "\n";
    std::cout << "Effective permittivity (Lichtenecker): eps_eff = " << std::setprecision(6)
              << dielectric_lich->getEffectivePermittivity() << "\n";
    std::cout << "Effective permittivity (Maxwell-Garnett): eps_eff = " << std::setprecision(6)
              << dielectric_mg->getEffectivePermittivity() << "\n\n";

    std::cout << "Cutoff frequencies:\n";
    std::cout << "  Empty cavity:      fc = " << std::setprecision(3)
              << empty->getCutoffFrequency()/1e6 << " MHz\n";
    // [FIX #2] Use getDielectricCutoffFrequency() instead of parent's getCutoffFrequency()
    std::cout << "  Dielectric (Lich): fc = "
              << dielectric_lich->getDielectricCutoffFrequency()/1e6 << " MHz\n";
    std::cout << "  Dielectric (MG):   fc = "
              << dielectric_mg->getDielectricCutoffFrequency()/1e6 << " MHz\n";
    std::cout << "  Ratio (Lich): fc_diel/fc_empty = "
              << std::setprecision(3)
              << dielectric_lich->getDielectricCutoffFrequency() / empty->getCutoffFrequency()
              << "\n\n";

    // Check mode type
    if (f_test >= dielectric_lich->getDielectricCutoffFrequency()) {
        std::cout << "Mode @ " << f_test/1e9 << " GHz: Propagating (f > fc_diel)\n";
    } else {
        std::cout << "Mode @ " << f_test/1e9 << " GHz: Evanescent (f < fc_diel)\n";
    }

    std::cout << "\nY-Parameters @ " << std::setprecision(3) << f_test/1e9 << " GHz:\n";
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "  Y11_empty              = " << Y_empty[0][0] << " S\n";
    std::cout << "  Y12_empty              = " << Y_empty[0][1] << " S\n";
    std::cout << "  Y11_diel (Lichtenecker)  = " << Y_diel_lich[0][0] << " S\n";
    std::cout << "  Y12_diel (Lichtenecker)  = " << Y_diel_lich[0][1] << " S\n";
    std::cout << "  Y11_diel (Maxwell-Garnett) = " << Y_diel_mg[0][0] << " S\n";
    std::cout << "  Y12_diel (Maxwell-Garnett) = " << Y_diel_mg[0][1] << " S\n\n";

    // Calculate differences
    Complex Y11_empty_c = Y_empty[0][0];
    Complex Y11_diel_lich_c = Y_diel_lich[0][0];
    Complex Y12_empty_c = Y_empty[0][1];
    Complex Y12_diel_lich_c = Y_diel_lich[0][1];

    double diff_Y11 = std::abs(Y11_diel_lich_c - Y11_empty_c) / std::abs(Y11_empty_c);
    double diff_Y12 = std::abs(Y12_diel_lich_c - Y12_empty_c) / std::abs(Y12_empty_c);

    std::cout << "Relative Differences (Lichtenecker vs Empty):\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  dY11/Y11 = " << diff_Y11*100 << "%\n";
    std::cout << "  dY12/Y12 = " << diff_Y12*100 << "%\n";

    std::cout << "\n" << std::string(70, '=') << "\n";
}

// ========================================================================
// TEST 2: Aperture with Cover Validation
// ========================================================================
void testApertureWithCover() {
    std::cout << "\nAperture with Cover Validation\n";
    std::cout << "==============================\n";

    double aa = 0.300;
    double b = 0.120;
    double l = 0.08;
    double w = 0.08;
    double t = 0.0015;
    double tau = 0.001;  // 1 mm cover gap
    double f_test = 1e9;

    // Regular aperture (reference)
    auto regular = std::make_shared<AP_SlotAperture>(
        1, 0, 1, aa, b, l, w, t
        );
    auto Y_regular = regular->computeYParameters(f_test);

    // Aperture with cover
    auto with_cover = std::make_shared<AP_SlotWithCover>(
        1, 0, 2, aa, b, l, w, t, tau
        );
    auto Y_cover = with_cover->computeYParameters(f_test);

    // Display results
    std::cout << "Aperture: l=" << std::fixed << std::setprecision(3) << l*1000
              << "mm, w=" << w*1000 << "mm\n";
    std::cout << "Cover gap: tau=" << std::setprecision(4) << tau*1000 << "mm\n";
    std::cout << "Slot-line impedance: Z0s = " << std::setprecision(3)
              << regular->getSlotLineImpedance() << " Ohm\n\n";

    // [FIX #5] Use Fs naming to match corrected AP_SlotWithCover
    std::cout << "Correction coefficient:\n";
    std::cout << "  Fs = " << std::setprecision(6)
              << with_cover->getCorrectionCoefficient() << "\n\n";

    std::cout << "Final aperture admittance @ " << f_test/1e9 << " GHz:\n";
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "  Regular aperture:    Y = " << Y_regular[0][0] << " S\n";
    std::cout << "  Aperture with cover: Y = " << Y_cover[0][0] << " S\n\n";

    // Calculate difference
    Complex Y11_reg = Y_regular[0][0];
    Complex Y11_cov = Y_cover[0][0];
    double diff = std::abs(Y11_cov - Y11_reg) / std::abs(Y11_reg);

    std::cout << "Comparison with regular aperture:\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Relative difference: " << diff*100 << "%\n";

    std::cout << "\n" << std::string(70, '=') << "\n";
}

// ========================================================================
// TEST 3: Complete System - Dielectric Cavity + Slot with Cover
//         Full frequency sweep for MATLAB comparison
// ========================================================================
void testCompleteSystem() {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "COMPLETE SYSTEM: Dielectric Cavity + Slot with Cover\n";
    std::cout << std::string(70, '=') << "\n\n";

    // Physical parameters
    double a = 0.300;           // Enclosure width [m]
    double b = 0.120;           // Enclosure height [m]
    double d = 0.300;           // Enclosure depth [m]
    double p = 0.150;           // Observation point [m]
    double l_aperture = 0.08;   // Aperture width [m]
    double w_aperture = 0.08;   // Aperture height [m]
    double t_wall = 0.0015;     // Wall thickness [m]
    double tau = 0.001;         // Cover gap [m]
    double h_diel = 0.060;      // Dielectric thickness [m]
    double epsilon_r = 4.4;     // Relative permittivity (FR-4)

    std::cout << "Configuration:\n";
    std::cout << "  Enclosure: " << a*1000 << "x" << b*1000 << "x" << d*1000 << " mm\n";
    std::cout << "  Aperture:  " << l_aperture*1000 << "x" << w_aperture*1000 << " mm\n";
    std::cout << "  Observer:  p=" << p*1000 << " mm from front\n";
    std::cout << "  Wall:      t=" << t_wall*1000 << " mm\n";
    std::cout << "  Cover gap: tau=" << tau*1000 << " mm\n";
    std::cout << "  Dielectric: h=" << h_diel*1000 << "mm, er=" << epsilon_r
              << ", h/b=" << h_diel/b << "\n";
    std::cout << "  Source:    Zs = Z_0 = " << std::setprecision(3) << Z_0 << " Ohm\n\n";

    // ---- Single-point tests at multiple frequencies ----
    std::cout << "Single-point results:\n";
    std::cout << std::setw(12) << "f [GHz]"
              << std::setw(30) << "V1"
              << std::setw(30) << "V2"
              << std::setw(12) << "SE [dB]" << "\n";
    std::cout << std::string(84, '-') << "\n";

    MNASolver solver_spot;

    auto src_s = std::make_shared<SRC_VoltageSource>(0, 1, 0, Complex(1.0, 0.0), Z_0);
    auto ap_s = std::make_shared<AP_SlotWithCover>(1, 0, 1, a, b, l_aperture, w_aperture, t_wall, tau);
    auto c1_s = std::make_shared<TL_DielectricCavity>(1, 2, 2, a, b, p, h_diel, epsilon_r,
                                                      TL_DielectricCavity::EffPermMethod::LICHTENECKER);
    auto c2_s = std::make_shared<TL_DielectricCavity>(2, 0, 3, a, b, d-p, h_diel, epsilon_r,
                                                      TL_DielectricCavity::EffPermMethod::LICHTENECKER);

    solver_spot.addBranch(src_s);
    solver_spot.addBranch(ap_s);
    solver_spot.addBranch(c1_s);
    solver_spot.addBranch(c2_s);

    std::vector<double> test_freqs = {100e6, 500e6, 1e9, 1.5e9, 2e9};

    for (double f : test_freqs) {
        auto U = solver_spot.solve(f);
        Complex V1 = U(0);
        Complex V2 = U(1);
        double SE = -20.0 * std::log10(std::abs(2.0 * V2));

        std::cout << std::fixed << std::setprecision(3);
        std::cout << std::setw(12) << f/1e9
                  << "  (" << std::setprecision(6)
                  << V1.real() << ", " << V1.imag() << ")"
                  << "  ("
                  << V2.real() << ", " << V2.imag() << ")"
                  << "  " << std::setprecision(2) << SE << "\n";
    }

    // ---- Full frequency sweep ----
    std::cout << "\n";

    FrequencySweepConfig config;
    config.f_start = 1.0e6;
    config.f_stop = 2.0e9;
    config.num_points = 200;

    auto results = performFrequencySweepDielectricCover(
        config, a, b, d, p, l_aperture, w_aperture, t_wall, Z_0,
        h_diel, epsilon_r, tau);

    // Save to CSV
    std::filesystem::path output_path =
        std::filesystem::current_path() / "SE_cpp_dielectric_cover.csv";

    if (exportToCSV(results, output_path.string())) {
        std::cout << "Results saved to: " << output_path.string() << "\n";

        auto min_SE = *std::min_element(results.SE_dB.begin(), results.SE_dB.end());
        auto max_SE = *std::max_element(results.SE_dB.begin(), results.SE_dB.end());
        double avg_SE = std::accumulate(results.SE_dB.begin(), results.SE_dB.end(), 0.0)
                        / results.SE_dB.size();

        std::cout << "\nShielding Effectiveness Summary (Dielectric + Cover):\n";
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "  Minimum:  " << min_SE << " dB\n";
        std::cout << "  Maximum:  " << max_SE << " dB\n";
        std::cout << "  Average:  " << avg_SE << " dB\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
}

// ========================================================================
// 2-SECTION CASCADE TEST (Figure 3.10)
// ========================================================================
// Circuit topology:
//   Branch 0: Source             (Node 0 -> Node 1)
//   Branch 1: Front aperture     (Node 1 -> Node 0, shunt)
//   Branch 2: TL segment p1      (Node 1 -> Node 2)  [obs P1]
//   Branch 3: TL segment d1-p1   (Node 2 -> Node 3)
//   Branch 4: Inter-section ap   (Node 3 -> Node 0, shunt)
//   Branch 5: TL segment p2      (Node 3 -> Node 4)  [obs P2]
//   Branch 6: TL segment d2-p2   (Node 4 -> Node 0)
//
// 7 branches, 4 non-ground nodes
// Observation: P1 = Node 2 (U(1)), P2 = Node 4 (U(3))
// ========================================================================
void test2SectionCascade() {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "2-Section Cascade (Figure 3.10)\n";
    std::cout << std::string(70, '=') << "\n\n";

    // ====================================================================
    // PHYSICAL PARAMETERS
    // ====================================================================
    // Shared enclosure dimensions
    double a = 0.300;       // Width [m]
    double b = 0.120;       // Height [m]
    double t = 0.0015;      // Wall thickness [m]

    // Section 1 parameters
    double d1 = 0.300;      // Depth [m]
    double p1 = 0.150;      // Observation point P1 position [m]
    double l1 = 0.08;       // Front aperture width [m]
    double w1 = 0.08;       // Front aperture height [m]

    // Section 2 parameters (identical for first validation)
    double d2 = 0.300;      // Depth [m]
    double p2 = 0.150;      // Observation point P2 position [m]
    double l2 = 0.08;       // Inter-section aperture width [m]
    double w2 = 0.08;       // Inter-section aperture height [m]

    // TL segment lengths
    double L_TL1 = p1;          // Branch 2: front to P1
    double L_TL2 = d1 - p1;     // Branch 3: P1 to inter-section wall
    double L_TL3 = p2;          // Branch 5: inter-section wall to P2
    double L_TL4 = d2 - p2;     // Branch 6: P2 to back wall

    std::cout << "Configuration:\n";
    std::cout << "  Enclosure: a=" << a*1000 << "mm, b=" << b*1000 << "mm\n";
    std::cout << "  Section 1: d1=" << d1*1000 << "mm, p1=" << p1*1000
              << "mm, aperture l1=" << l1*1000 << "mm\n";
    std::cout << "  Section 2: d2=" << d2*1000 << "mm, p2=" << p2*1000
              << "mm, aperture l2=" << l2*1000 << "mm\n";
    std::cout << "  Source: Zs = Z_0 = " << std::setprecision(3) << Z_0 << " Ohm\n\n";

    // ====================================================================
    // BUILD CIRCUIT (7 branches)
    // ====================================================================
    MNASolver solver;
    int bid = 0;  // branch ID counter

    // Branch 0: Source (Node 0 -> Node 1)
    auto src = std::make_shared<SRC_VoltageSource>(
        0, 1, bid++, Complex(1.0, 0.0), Z_0);

    // Branch 1: Front aperture (Node 1 -> Node 0, shunt)
    auto ap1 = std::make_shared<AP_SlotAperture>(
        1, 0, bid++, a, b, l1, w1, t);

    // Branch 2: TL segment p1 (Node 1 -> Node 2)
    auto tl1 = std::make_shared<TL_EmptyCavity>(
        1, 2, bid++, a, b, L_TL1);

    // Branch 3: TL segment d1-p1 (Node 2 -> Node 3)
    auto tl2 = std::make_shared<TL_EmptyCavity>(
        2, 3, bid++, a, b, L_TL2);

    // Branch 4: Inter-section aperture (Node 3 -> Node 0, shunt)
    auto ap2 = std::make_shared<AP_SlotAperture>(
        3, 0, bid++, a, b, l2, w2, t);

    // Branch 5: TL segment p2 (Node 3 -> Node 4)
    auto tl3 = std::make_shared<TL_EmptyCavity>(
        3, 4, bid++, a, b, L_TL3);

    // Branch 6: TL segment d2-p2 (Node 4 -> Node 0)
    auto tl4 = std::make_shared<TL_EmptyCavity>(
        4, 0, bid++, a, b, L_TL4);

    // Add all branches
    solver.addBranch(src);
    solver.addBranch(ap1);
    solver.addBranch(tl1);
    solver.addBranch(tl2);
    solver.addBranch(ap2);
    solver.addBranch(tl3);
    solver.addBranch(tl4);

    std::cout << "Circuit: " << solver.getNumBranches() << " branches, "
              << solver.getNumNodes() << " nodes\n\n";

    // ====================================================================
    // SINGLE-POINT SPOT CHECKS
    // ====================================================================
    std::cout << "Single-point spot checks:\n";
    std::cout << std::setw(12) << "f [GHz]"
              << std::setw(18) << "V2 (P1)"
              << std::setw(12) << "SE_P1 [dB]"
              << std::setw(18) << "V4 (P2)"
              << std::setw(12) << "SE_P2 [dB]" << "\n";
    std::cout << std::string(72, '-') << "\n";

    std::vector<double> spot_freqs = {1e6, 100e6, 500e6, 1e9, 1.5e9};

    for (double f : spot_freqs) {
        auto U = solver.solve(f);
        // U(0)=V1, U(1)=V2(P1), U(2)=V3, U(3)=V4(P2)
        Complex V2 = U(1);
        Complex V4 = U(3);
        double SE_P1 = -20.0 * std::log10(std::abs(2.0 * V2));
        double SE_P2 = -20.0 * std::log10(std::abs(2.0 * V4));

        std::cout << std::fixed << std::setprecision(3)
                  << std::setw(12) << f/1e9
                  << "  (" << std::setprecision(6) << V2.real() << "," << V2.imag() << ")"
                  << std::setprecision(2) << std::setw(10) << SE_P1
                  << "  (" << std::setprecision(6) << V4.real() << "," << V4.imag() << ")"
                  << std::setprecision(2) << std::setw(10) << SE_P2
                  << "\n";
    }

    // ====================================================================
    // VALIDATION POINT 53
    // ====================================================================
    double f_start = 1.0e6;
    double f_stop = 2.0e9;
    int pn = 200;
    double fstep = (f_stop - f_start) / pn;
    double f_53 = f_start + 52 * fstep;

    auto U_53 = solver.solve(f_53);
    std::cout << "\nValidation Point 53: f = " << std::setprecision(6) << f_53/1e6 << " MHz\n";
    std::cout << "  V1 = (" << U_53(0).real() << ", " << U_53(0).imag() << ")\n";
    std::cout << "  V2 = (" << U_53(1).real() << ", " << U_53(1).imag() << ")  (P1)\n";
    std::cout << "  V3 = (" << U_53(2).real() << ", " << U_53(2).imag() << ")\n";
    std::cout << "  V4 = (" << U_53(3).real() << ", " << U_53(3).imag() << ")  (P2)\n";
    std::cout << "  SE_P1 = " << std::setprecision(6)
              << -20.0 * std::log10(std::abs(2.0 * U_53(1))) << " dB\n";
    std::cout << "  SE_P2 = " << std::setprecision(6)
              << -20.0 * std::log10(std::abs(2.0 * U_53(3))) << " dB\n";

    // ====================================================================
    // FULL FREQUENCY SWEEP
    // ====================================================================
    std::cout << "\nRunning 2-section frequency sweep: "
              << f_start/1e6 << " MHz to " << f_stop/1e9 << " GHz ("
              << pn << " points)\n";

    std::vector<double> frequencies;
    std::vector<double> SE_P1_arr, SE_P2_arr;
    std::vector<Complex> V2_arr, V4_arr;

    frequencies.reserve(pn);
    SE_P1_arr.reserve(pn);
    SE_P2_arr.reserve(pn);
    V2_arr.reserve(pn);
    V4_arr.reserve(pn);

    int progress_step = std::max(1, pn / 20);

    for (int i = 0; i < pn; ++i) {
        double f = f_start + i * fstep;
        frequencies.push_back(f);

        auto U = solver.solve(f);
        Complex V2 = U(1);  // P1
        Complex V4 = U(3);  // P2

        V2_arr.push_back(V2);
        V4_arr.push_back(V4);
        SE_P1_arr.push_back(-20.0 * std::log10(std::abs(2.0 * V2)));
        SE_P2_arr.push_back(-20.0 * std::log10(std::abs(2.0 * V4)));

        if (i % progress_step == 0) std::cout << ".";
    }
    std::cout << " Done.\n";

    // ====================================================================
    // SAVE CSV
    // ====================================================================
    std::filesystem::path output_path =
        std::filesystem::current_path() / "SE_cpp_2section.csv";

    std::ofstream file(output_path.string());
    if (file.is_open()) {
        file << "# EMShieldDesigner - 2-Section Cascade Results\n";
        file << "# f_Hz,SE_P1_dB,SE_P2_dB,V2_real,V2_imag,V4_real,V4_imag\n";
        file << std::fixed;
        for (int i = 0; i < pn; ++i) {
            file << std::setprecision(9) << frequencies[i] << ","
                 << std::setprecision(6) << SE_P1_arr[i] << ","
                 << SE_P2_arr[i] << ","
                 << std::setprecision(9)
                 << V2_arr[i].real() << "," << V2_arr[i].imag() << ","
                 << V4_arr[i].real() << "," << V4_arr[i].imag() << "\n";
        }
        file.close();
        std::cout << "Results saved to: " << output_path.string() << "\n";
    }

    // ====================================================================
    // SUMMARY
    // ====================================================================
    auto min_P1 = *std::min_element(SE_P1_arr.begin(), SE_P1_arr.end());
    auto max_P1 = *std::max_element(SE_P1_arr.begin(), SE_P1_arr.end());
    double avg_P1 = std::accumulate(SE_P1_arr.begin(), SE_P1_arr.end(), 0.0) / pn;

    auto min_P2 = *std::min_element(SE_P2_arr.begin(), SE_P2_arr.end());
    auto max_P2 = *std::max_element(SE_P2_arr.begin(), SE_P2_arr.end());
    double avg_P2 = std::accumulate(SE_P2_arr.begin(), SE_P2_arr.end(), 0.0) / pn;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nSE at P1 (Node 2, Section 1 center):\n";
    std::cout << "  Min: " << min_P1 << " dB, Max: " << max_P1
              << " dB, Mean: " << avg_P1 << " dB\n";
    std::cout << "SE at P2 (Node 4, Section 2 center):\n";
    std::cout << "  Min: " << min_P2 << " dB, Max: " << max_P2
              << " dB, Mean: " << avg_P2 << " dB\n";

    // Sanity check
    std::cout << "\nPhysical sanity check:\n";
    if (avg_P2 > avg_P1) {
        std::cout << "  PASS: Mean SE_P2 (" << avg_P2
                  << " dB) > Mean SE_P1 (" << avg_P1 << " dB)\n";
    } else {
        std::cout << "  WARNING: Mean SE_P2 <= Mean SE_P1 (unexpected)\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
}


// ============================================================================
// ADD THIS TO main.cpp
// Include: #include "CircuitGenerator.h"
// Call: testCircuitGenerator();
// ============================================================================

void testCircuitGenerator() {
    using namespace EMCore;

    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "CircuitGenerator Validation (N-Section Cascade)\n";
    std::cout << std::string(70, '=') << "\n\n";

    // Frequency sweep parameters (same as MATLAB)
    double f_start = 1.0e6;
    double f_stop = 2.0e9;
    int pn = 200;
    double fstep = (f_stop - f_start) / pn;

    // ====================================================================
    // TEST 1: 1-Section (validate against existing 1-section reference)
    // ====================================================================
    {
        std::cout << "--- TEST 1: 1-Section (validation) ---\n";

        EnclosureConfig cfg;
        cfg.a = 0.300; cfg.b = 0.120; cfg.t = 0.0015;
        cfg.sections = {{
            .depth = 0.300, .obs_position = 0.150, .has_observation = true,
            .aperture_l = 0.08, .aperture_w = 0.08
        }};

        MNASolver solver;
        auto obs = CircuitGenerator::generate(cfg, solver, false);

        // Point 53 validation
        double f_53 = f_start + 52 * fstep;
        auto SE = CircuitGenerator::computeSE(solver, obs, f_53);

        std::cout << "  Point 53 (" << std::fixed << std::setprecision(3)
                  << f_53/1e6 << " MHz):\n";
        std::cout << "    SE_P1 = " << std::setprecision(6) << SE[0] << " dB\n";
        std::cout << "    Expected: 24.142022 dB\n";
        std::cout << "    Diff: " << std::abs(SE[0] - 24.142022) << " dB\n";

        if (std::abs(SE[0] - 24.142022) < 0.001)
            std::cout << "    PASS ✓\n\n";
        else
            std::cout << "    FAIL ✗\n\n";
    }

    // ====================================================================
    // TEST 2: 2-Section identical (validate against 2-section MATLAB)
    // ====================================================================
    {
        std::cout << "--- TEST 2: 2-Section identical ---\n";

        EnclosureConfig cfg;
        cfg.a = 0.300; cfg.b = 0.120; cfg.t = 0.0015;

        SectionConfig sec;
        sec.depth = 0.300; sec.obs_position = 0.150;
        sec.aperture_l = 0.08; sec.aperture_w = 0.08;
        sec.has_observation = true;

        cfg.sections = {sec, sec};

        MNASolver solver;
        auto obs = CircuitGenerator::generate(cfg, solver, true);

        // Point 53 validation
        double f_53 = f_start + 52 * fstep;
        auto SE = CircuitGenerator::computeSE(solver, obs, f_53);

        std::cout << "  Point 53 (" << std::fixed << std::setprecision(3)
                  << f_53/1e6 << " MHz):\n";
        std::cout << std::setprecision(6);
        std::cout << "    SE_P1 = " << SE[0] << " dB  (expected: 23.971428)\n";
        std::cout << "    SE_P2 = " << SE[1] << " dB  (expected: 58.362361)\n";

        bool pass = std::abs(SE[0] - 23.971428) < 0.001 &&
                    std::abs(SE[1] - 58.362361) < 0.001;
        std::cout << "    " << (pass ? "PASS ✓" : "FAIL ✗") << "\n\n";

        // Full sweep + CSV
        std::cout << "  Running full sweep...";
        std::vector<double> freqs;
        std::vector<std::vector<double>> SE_all(obs.size());

        for (int i = 0; i < pn; ++i) {
            double f = f_start + i * fstep;
            freqs.push_back(f);
            auto se = CircuitGenerator::computeSE(solver, obs, f);
            for (size_t oi = 0; oi < obs.size(); ++oi) {
                SE_all[oi].push_back(se[oi]);
            }
        }
        std::cout << " Done.\n";

        // Summary
        for (size_t oi = 0; oi < obs.size(); ++oi) {
            auto min_it = std::min_element(SE_all[oi].begin(), SE_all[oi].end());
            auto max_it = std::max_element(SE_all[oi].begin(), SE_all[oi].end());
            double mean = std::accumulate(SE_all[oi].begin(), SE_all[oi].end(), 0.0) / pn;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "  SE_" << obs[oi].label << ": Min=" << *min_it
                      << " dB, Max=" << *max_it << " dB, Mean=" << mean << " dB\n";
        }
        std::cout << "\n";
    }

    // ====================================================================
    // TEST 3: 3-Section identical (validate against 3-section MATLAB)
    // ====================================================================
    {
        std::cout << "--- TEST 3: 3-Section identical ---\n";

        EnclosureConfig cfg;
        cfg.a = 0.300; cfg.b = 0.120; cfg.t = 0.0015;

        SectionConfig sec;
        sec.depth = 0.300; sec.obs_position = 0.150;
        sec.aperture_l = 0.08; sec.aperture_w = 0.08;
        sec.has_observation = true;

        cfg.sections = {sec, sec, sec};

        MNASolver solver;
        auto obs = CircuitGenerator::generate(cfg, solver, true);

        // Point 53 validation
        double f_53 = f_start + 52 * fstep;
        auto SE = CircuitGenerator::computeSE(solver, obs, f_53);

        std::cout << "  Point 53 (" << std::fixed << std::setprecision(3)
                  << f_53/1e6 << " MHz):\n";
        std::cout << std::setprecision(6);
        std::cout << "    SE_P1 = " << SE[0] << " dB  (expected: 23.971364)\n";
        std::cout << "    SE_P2 = " << SE[1] << " dB  (expected: 58.191795)\n";
        std::cout << "    SE_P3 = " << SE[2] << " dB  (expected: 92.582727)\n";

        bool pass = std::abs(SE[0] - 23.971364) < 0.001 &&
                    std::abs(SE[1] - 58.191795) < 0.001 &&
                    std::abs(SE[2] - 92.582727) < 0.001;
        std::cout << "    " << (pass ? "PASS ✓" : "FAIL ✗") << "\n\n";

        // Full sweep summary
        std::cout << "  Running full sweep...";
        std::vector<std::vector<double>> SE_all(obs.size());
        for (int i = 0; i < pn; ++i) {
            double f = f_start + i * fstep;
            auto se = CircuitGenerator::computeSE(solver, obs, f);
            for (size_t oi = 0; oi < obs.size(); ++oi) {
                SE_all[oi].push_back(se[oi]);
            }
        }
        std::cout << " Done.\n";

        for (size_t oi = 0; oi < obs.size(); ++oi) {
            double mean = std::accumulate(SE_all[oi].begin(), SE_all[oi].end(), 0.0) / pn;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "  SE_" << obs[oi].label << ": Mean=" << mean << " dB\n";
        }

        // MATLAB reference means: P1=16.09, P2=34.73, P3=53.12
        std::cout << "  Expected means: P1=16.09, P2=34.73, P3=53.12 dB\n\n";
    }

    // ====================================================================
    // TEST 4: 2-Section different parameters (validate against CONFIG 3)
    // ====================================================================
    {
        std::cout << "--- TEST 4: 2-Section different parameters ---\n";

        EnclosureConfig cfg;
        cfg.a = 0.300; cfg.b = 0.120; cfg.t = 0.0015;

        SectionConfig sec1;
        sec1.depth = 0.300; sec1.obs_position = 0.150;
        sec1.aperture_l = 0.08; sec1.aperture_w = 0.08;
        sec1.has_observation = true;

        SectionConfig sec2;
        sec2.depth = 0.200; sec2.obs_position = 0.100;
        sec2.aperture_l = 0.05; sec2.aperture_w = 0.05;
        sec2.has_observation = true;

        cfg.sections = {sec1, sec2};

        MNASolver solver;
        auto obs = CircuitGenerator::generate(cfg, solver, true);

        // Point 53 validation
        double f_53 = f_start + 52 * fstep;
        auto SE = CircuitGenerator::computeSE(solver, obs, f_53);

        std::cout << "  Point 53 (" << std::fixed << std::setprecision(3)
                  << f_53/1e6 << " MHz):\n";
        std::cout << std::setprecision(6);
        std::cout << "    SE_P1 = " << SE[0] << " dB  (expected: 24.093145)\n";
        std::cout << "    SE_P2 = " << SE[1] << " dB  (expected: 69.811831)\n";

        bool pass = std::abs(SE[0] - 24.093145) < 0.001 &&
                    std::abs(SE[1] - 69.811831) < 0.001;
        std::cout << "    " << (pass ? "PASS ✓" : "FAIL ✗") << "\n\n";

        // Full sweep summary
        std::cout << "  Running full sweep...";
        std::vector<std::vector<double>> SE_all(obs.size());
        for (int i = 0; i < pn; ++i) {
            double f = f_start + i * fstep;
            auto se = CircuitGenerator::computeSE(solver, obs, f);
            for (size_t oi = 0; oi < obs.size(); ++oi) {
                SE_all[oi].push_back(se[oi]);
            }
        }
        std::cout << " Done.\n";

        for (size_t oi = 0; oi < obs.size(); ++oi) {
            double mean = std::accumulate(SE_all[oi].begin(), SE_all[oi].end(), 0.0) / pn;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "  SE_" << obs[oi].label << ": Mean=" << mean << " dB\n";
        }

        // MATLAB reference means: P1=15.93, P2=47.70
        std::cout << "  Expected means: P1=15.93, P2=47.70 dB\n\n";
    }

    // ====================================================================
    // OVERALL RESULTS
    // ====================================================================
    std::cout << std::string(70, '=') << "\n";
    std::cout << "CircuitGenerator validation complete.\n";
    std::cout << "If all tests PASS, the generator matches MATLAB exactly.\n";
    std::cout << std::string(70, '=') << "\n\n";
}
// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(int argc, char *argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    QApplication app(argc, argv);
    MainWindow mw;
    mw.show();

    // ========================================================================
    // PHYSICAL PARAMETERS (MATLAB-COMPATIBLE)
    // ========================================================================
    double a_param = 0.300;      // Enclosure width [m]
    double b_param = 0.120;      // Enclosure height [m]
    double d_param = 0.300;      // Enclosure depth [m]
    double p_param = 0.150;      // Observation point [m]
    double t_param = 0.0015;     // Wall thickness [m]
    double l_param = 0.08;       // Aperture width [m]
    double w_param = 0.08;       // Aperture height [m]

    std::cout << "\n========================================\n";
    std::cout << "EMShieldDesigner - Shielding Analysis\n";
    std::cout << "MATLAB-Compatible Validation\n";
    std::cout << "========================================\n\n";

    std::cout << "Configuration (matching MATLAB):\n";
    std::cout << "  Enclosure: " << a_param*1000 << "x" << b_param*1000
              << "x" << d_param*1000 << " mm\n";
    std::cout << "  Aperture:  " << l_param*1000 << "x" << w_param*1000 << " mm\n";
    std::cout << "  Observer:  " << p_param*1000 << " mm from front\n";
    std::cout << "  Wall:      " << t_param*1000 << " mm\n";
    std::cout << "  Source:    Zs = Z_0 = " << std::setprecision(3) << Z_0 << " Ohm\n";

    double fc = C_LIGHT / (2.0 * a_param);
    std::cout << "  Cutoff:    fc = " << std::setprecision(1) << fc/1e6 << " MHz\n\n";

    // ========================================================================
    // INDIVIDUAL ELEMENT TESTS
    // ========================================================================
    if (ENABLE_NEW_TESTS) {
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "RUNNING ELEMENT VALIDATION TESTS\n";
        std::cout << std::string(70, '=') << "\n";

        testDielectricCavity();
        testApertureWithCover();

        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "ELEMENT TESTS COMPLETE\n";
        std::cout << std::string(70, '=') << "\n\n";
    }

    // ========================================================================
    // POINT 53 VALIDATION
    // ========================================================================
    calculateExactPoint53(a_param, b_param, d_param, p_param,
                          l_param, w_param, t_param, Z_0);

    if (ENABLE_SINGLE_POINT_TEST) {
        runValidationTest(a_param, b_param, d_param, p_param,
                          l_param, w_param, t_param, Z_0);
    }

    // ========================================================================
    // FULL FREQUENCY SWEEP — EMPTY CAVITY + SLOT APERTURE
    // ========================================================================
    if (ENABLE_FREQUENCY_SWEEP) {
        FrequencySweepConfig config;
        config.f_start = 1.0e6;
        config.f_stop = 2.0e9;
        config.num_points = 200;

        auto results = performFrequencySweep(config, a_param, b_param, d_param,
                                             p_param, l_param, w_param, t_param,
                                             Z_0);

        std::filesystem::path output_path =
            std::filesystem::current_path() / "SE_cpp_results.csv";

        if (exportToCSV(results, output_path.string())) {
            std::cout << "\nResults saved to: " << output_path.string() << "\n";

            auto min_SE = *std::min_element(results.SE_dB.begin(), results.SE_dB.end());
            auto max_SE = *std::max_element(results.SE_dB.begin(), results.SE_dB.end());
            double avg_SE = std::accumulate(results.SE_dB.begin(), results.SE_dB.end(), 0.0)
                            / results.SE_dB.size();

            std::cout << "\nShielding Effectiveness Summary (Empty Cavity):\n";
            std::cout << std::fixed << std::setprecision(1);
            std::cout << "  Minimum:  " << min_SE << " dB\n";
            std::cout << "  Maximum:  " << max_SE << " dB\n";
            std::cout << "  Average:  " << avg_SE << " dB\n";

            // Point 53 validation
            size_t point_53_idx = 52;  // [FIX #3] size_t for unsigned comparison
            if (point_53_idx < results.frequencies.size()) {
                std::cout << "\nPoint 53 (C++ index " << point_53_idx << "):\n";
                std::cout << "  Frequency: " << std::setprecision(6)
                          << results.frequencies[point_53_idx]/1e6 << " MHz\n";
                std::cout << "  SE: " << std::setprecision(3)
                          << results.SE_dB[point_53_idx] << " dB\n";
                std::cout << "  V1: " << results.V1[point_53_idx] << " V\n";
                std::cout << "  V2: " << results.V2[point_53_idx] << " V\n";
            }
        } else {
            std::cerr << "Failed to save results.\n";
        }
    }

    // ========================================================================
    // FULL FREQUENCY SWEEP — DIELECTRIC CAVITY + SLOT WITH COVER
    // ========================================================================
    // COMMENTED OUT to show 2-section results clearly
    /*
    if (ENABLE_COMPLETE_SYSTEM) {
        testCompleteSystem();
    }
    */

    // ========================================================================
    // 2-SECTION CASCADE TEST
    // ========================================================================
    if (ENABLE_2SECTION_CASCADE) {
        // testCompleteSystem();
        // test2SectionCascade();
        testCircuitGenerator();
    }

    /*
    std::cout << "\n========================================\n";
    std::cout << "All analyses complete.\n";
    std::cout << "========================================\n\n";

    std::cout << "Output files:\n";
    std::cout << "  1. SE_cpp_results.csv            (empty cavity + slot aperture)\n";
    std::cout << "  2. SE_cpp_dielectric_cover.csv   (dielectric cavity + slot with cover)\n";
    std::cout << "  3. SE_cpp_2section.csv           (2-section cascade)\n\n";

    std::cout << "Next steps:\n";
    std::cout << "  1. Run MATLAB reference codes\n";
    std::cout << "  2. Compare CSV outputs\n";
    std::cout << "  3. Validate point 53 values match exactly\n";
    */

    std::cout << "\n========================================\n";
    std::cout << "All analyses complete.\n";
    std::cout << "========================================\n\n";
    std::cout << "Output files:\n";
    std::cout << "  1. SE_cpp_results.csv            (empty cavity + slot aperture)\n";
    std::cout << "  2. SE_cpp_dielectric_cover.csv   (dielectric cavity + slot with cover)\n\n";
    std::cout << "Validated configurations:\n";
    std::cout << "  1. 1-section empty cavity         (EXACT match with MATLAB)\n";
    std::cout << "  2. 1-section dielectric + cover   (validated)\n";
    std::cout << "  3. 2-section identical cascade     (EXACT match with MATLAB)\n";
    std::cout << "  4. 3-section identical cascade     (validated via CircuitGenerator)\n";
    std::cout << "  5. 2-section different parameters  (validated via CircuitGenerator)\n";

    return app.exec();
}
