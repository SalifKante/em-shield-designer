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

// ============================================================================
// DEBUG CONTROL FLAGS
// ============================================================================
constexpr bool ENABLE_SINGLE_POINT_TEST = true;   // Run validation test
constexpr bool ENABLE_FREQUENCY_SWEEP = true;     // Run full sweep
constexpr bool ENABLE_NEW_TESTS = true;           // Run dielectric & cover tests

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
// PERFORM FREQUENCY SWEEP
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

    // Create branches in correct order (matching MATLAB)
    auto src = std::make_shared<SRC_VoltageSource>(
        0, 1, 0, Complex(1.0, 0.0), Z_source);

    auto aperture = std::make_shared<AP_SlotAperture>(
        1, 0, 1, a, b, l, w, t);

    auto cavity1 = std::make_shared<TL_EmptyCavity>(
        1, 2, 2, a, b, L1);

    auto cavity2 = std::make_shared<TL_EmptyCavity>(
        2, 0, 3, a, b, L2);

    // Add branches in MATLAB order
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
    int validation_idx = 53;  // Point 53 (0-indexed in C++, 1-indexed in MATLAB)
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

    // Create test circuit (same as MATLAB)
    auto test_src = std::make_shared<SRC_VoltageSource>(
        0, 1, 0, Complex(1.0, 0.0), Z_source);

    auto test_aperture = std::make_shared<AP_SlotAperture>(
        1, 0, 1, a, b, l, w, t);

    auto test_cavity1 = std::make_shared<TL_EmptyCavity>(
        1, 2, 2, a, b, p);

    auto test_cavity2 = std::make_shared<TL_EmptyCavity>(
        2, 0, 3, a, b, d - p);

    // Add branches in MATLAB order
    solver.addBranch(test_src);
    solver.addBranch(test_aperture);
    solver.addBranch(test_cavity1);
    solver.addBranch(test_cavity2);

    // Solve
    auto U_test = solver.solve(f_test);

    // Extract results
    Complex V1 = U_test(0);
    Complex V2 = U_test(1);
    double SE_test = -20.0 * std::log10(std::abs(2.0 * V2));

    // Display results
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

    // Calculate exact frequency
    double f_start = 1.0e6;
    double f_stop = 2.0e9;
    int num_points = 200;
    double fstep = (f_stop - f_start) / num_points;
    double f_53 = f_start + 52 * fstep;  // Point 53 (0-indexed: i=52)

    std::cout << "Exact frequency: " << std::fixed << std::setprecision(9)
              << f_53 << " Hz\n";
    std::cout << "                = " << std::setprecision(6) << f_53/1e6 << " MHz\n";
    std::cout << "                = " << std::setprecision(9) << f_53/1e9 << " GHz\n\n";

    // Cutoff frequency for this enclosure
    double fc = 3e8 / (2.0 * a);
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

    // Dielectric-filled cavity (Lichtenecker - more accurate)
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
    std::cout << "Dielectric: h=" << h << "m, εᵣ=" << epsilon_r
              << ", h/b=" << std::setprecision(3) << (h/b) << "\n";
    std::cout << "Effective permittivity: εₑff = " << std::setprecision(6)
              << dielectric_lich->getEffectivePermittivity() << "\n\n";

    std::cout << "Cutoff frequencies:\n";
    std::cout << "  Empty cavity:      fc = " << std::setprecision(3)
              << empty->getCutoffFrequency()/1e6 << " MHz\n";
    std::cout << "  Dielectric cavity: fc = " << dielectric_lich->getCutoffFrequency()/1e6
              << " MHz\n";
    std::cout << "  Ratio: fc_diel / fc_empty = "
              << std::setprecision(3)
              << dielectric_lich->getCutoffFrequency() / empty->getCutoffFrequency()
              << "\n\n";

    // Check mode type
    if (f_test >= dielectric_lich->getCutoffFrequency()) {
        std::cout << "Mode: Propagating (f > fc)\n";
    } else {
        std::cout << "Mode: Evanescent (f < fc)\n";
    }

    std::cout << "Y-Parameters @ " << std::setprecision(3) << f_test/1e9 << " GHz:\n";
    std::cout << "  Y11_empty = " << std::scientific << std::setprecision(6)
              << Y_empty[0][0] << " S\n";
    std::cout << "  Y12_empty = " << Y_empty[0][1] << " S\n";
    std::cout << "  Y11_diel (Lichtenecker) = " << Y_diel_lich[0][0] << " S\n";
    std::cout << "  Y12_diel (Lichtenecker) = " << Y_diel_lich[0][1] << " S\n";
    std::cout << "  Y11_diel (Maxwell-Garnett) = " << Y_diel_mg[0][0] << " S\n";
    std::cout << "  Y12_diel (Maxwell-Garnett) = " << Y_diel_mg[0][1] << " S\n\n";

    // Calculate differences
    Complex Y11_empty = Y_empty[0][0];
    Complex Y11_diel_lich = Y_diel_lich[0][0];
    Complex Y12_empty = Y_empty[0][1];
    Complex Y12_diel_lich = Y_diel_lich[0][1];

    double diff_Y11 = std::abs(Y11_diel_lich - Y11_empty) / std::abs(Y11_empty);
    double diff_Y12 = std::abs(Y12_diel_lich - Y12_empty) / std::abs(Y12_empty);

    std::cout << "Relative Differences:\n";
    std::cout << "  ΔY11/Y11 = " << std::fixed << std::setprecision(2)
              << diff_Y11*100 << "%\n";
    std::cout << "  ΔY12/Y12 = " << diff_Y12*100 << "%\n";

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
    double Z0s_regular = regular->getSlotLineImpedance();

    // Aperture with cover
    auto with_cover = std::make_shared<AP_SlotWithCover>(
        1, 0, 2, aa, b, l, w, t, tau
        );
    auto Y_cover = with_cover->computeYParameters(f_test);
    double Z0s_cover = with_cover->getSlotLineImpedance();

    // Display results
    std::cout << "Aperture: l=" << std::fixed << std::setprecision(3) << l
              << "m, w=" << w << "m\n";
    std::cout << "Cover gap: τ=" << std::setprecision(4) << tau << "m ("
              << tau*1000 << "mm)\n";
    std::cout << "Slot-line impedance: Z₀ₛ = " << std::setprecision(3)
              << Z0s_regular << " Ω\n\n";

    std::cout << "Correction coefficient:\n";
    std::cout << "  Fᵤ = " << std::setprecision(6)
              << with_cover->getCorrectionCoefficient() << "\n\n";

    std::cout << "Final aperture admittance:\n";
    std::cout << "  Regular aperture:    Y = " << std::scientific << std::setprecision(6)
              << Y_regular[0][0] << " S\n";
    std::cout << "  Aperture with cover: Y = " << Y_cover[0][0] << " S\n\n";

    // Calculate difference
    Complex Y11_reg = Y_regular[0][0];
    Complex Y11_cov = Y_cover[0][0];
    double diff = std::abs(Y11_cov - Y11_reg) / std::abs(Y11_reg);

    std::cout << "Comparison with regular aperture:\n";
    std::cout << "  Relative difference: " << std::fixed << std::setprecision(2)
              << diff*100 << "%\n";

    std::cout << "\n" << std::string(70, '=') << "\n";
}

// ========================================================================
// TEST 3: Complete System with Dielectric and Cover (UPDATED - no resonance check)
// ========================================================================
void testCompleteSystem() {
    std::cout << "\nComplete System (Dielectric + Cover)\n";
    std::cout << "=====================================\n";

    double a = 0.300;
    double b = 0.120;
    double d = 0.300;
    double p = 0.150;
    double l_aperture = 0.08;
    double w_aperture = 0.08;
    double t_wall = 0.0015;
    double tau = 0.001;
    double h_dielectric = 0.060;
    double epsilon_r = 4.4;
    double Z_source = 120.0 * M_PI;

    // Create MNA solver
    MNASolver solver;

    // Create branches
    auto src = std::make_shared<SRC_VoltageSource>(
        0, 1, 0, Complex(1.0, 0.0), Z_source);

    // Aperture with cover
    auto aperture = std::make_shared<AP_SlotWithCover>(
        1, 0, 1, a, b, l_aperture, w_aperture, t_wall, tau);

    // First cavity: dielectric-filled
    auto cavity1 = std::make_shared<TL_DielectricCavity>(
        1, 2, 2, a, b, p, h_dielectric, epsilon_r,
        TL_DielectricCavity::EffPermMethod::LICHTENECKER);

    // Second cavity: empty
    auto cavity2 = std::make_shared<TL_EmptyCavity>(
        2, 0, 3, a, b, d - p);

    // Add branches
    solver.addBranch(src);
    solver.addBranch(aperture);
    solver.addBranch(cavity1);
    solver.addBranch(cavity2);

    // Test at multiple frequencies
    std::vector<double> test_freqs = {100e6, 500e6, 1e9, 1.5e9, 2e9};

    std::cout << "Testing complete system with:\n";
    std::cout << "  - Aperture with cover (τ=" << tau*1000 << "mm)\n";
    std::cout << "  - Dielectric cavity (h=" << h_dielectric*1000 << "mm, εᵣ=" << epsilon_r << ")\n";
    std::cout << "  - Empty cavity\n\n";

    std::cout << "Results:\n";
    std::cout << std::setw(12) << "f [GHz]"
              << std::setw(15) << "V1 [V]"
              << std::setw(15) << "V2 [V]"
              << std::setw(12) << "SE [dB]" << "\n";
    std::cout << std::string(58, '-') << "\n";

    for (double f : test_freqs) {
        auto U = solver.solve(f);
        Complex V1 = U(0);
        Complex V2 = U(1);
        double SE = -20.0 * std::log10(std::abs(2.0 * V2));

        std::cout << std::fixed << std::setprecision(3);
        std::cout << std::setw(12) << f/1e9
                  << std::setw(15) << "(" << std::setprecision(4)
                  << V1.real() << "," << V1.imag() << ")"
                  << std::setw(15) << "("
                  << V2.real() << "," << V2.imag() << ")"
                  << std::setw(12) << std::setprecision(1) << SE << "\n";
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(int argc, char *argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

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
    double Z_source = 120.0 * M_PI;  // Source impedance [Ω]

    std::cout << "\n========================================\n";
    std::cout << "EMShieldDesigner - Aperture Analysis\n";
    std::cout << "MATLAB-Compatible Validation\n";
    std::cout << "========================================\n\n";

    std::cout << "Configuration (matching MATLAB):\n";
    std::cout << "  Enclosure: " << a_param*1000 << "×" << b_param*1000
              << "×" << d_param*1000 << " mm\n";
    std::cout << "  Aperture:  " << l_param*1000 << "×" << w_param*1000 << " mm\n";
    std::cout << "  Observer:  " << p_param*1000 << " mm from front\n";
    std::cout << "  Wall:      " << t_param*1000 << " mm\n";
    std::cout << "  Source:    Zs = " << std::setprecision(1) << Z_source << " Ω\n";

    // Calculate cutoff frequency
    double fc = 3e8 / (2.0 * a_param);
    std::cout << "  Cutoff:    fc = " << std::setprecision(1) << fc/1e6 << " MHz\n\n";

    // ========================================================================
    // NEW TESTS: DIELECTRIC AND COVER VALIDATION
    // ========================================================================
    if (ENABLE_NEW_TESTS) {
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "RUNNING NEW PHYSICS MODEL VALIDATION TESTS\n";
        std::cout << std::string(70, '=') << "\n";

        testDielectricCavity();
        testApertureWithCover();

        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "NEW TESTS COMPLETE\n";
        std::cout << std::string(70, '=') << "\n\n";
    }

    // ========================================================================
    // CALCULATE EXACT POINT 53 DETAILS
    // ========================================================================
    calculateExactPoint53(a_param, b_param, d_param, p_param,
                          l_param, w_param, t_param, Z_source);

    // ========================================================================
    // VALIDATION TEST AT POINT 53 (~520 MHz)
    // ========================================================================
    if (ENABLE_SINGLE_POINT_TEST) {
        runValidationTest(a_param, b_param, d_param, p_param,
                          l_param, w_param, t_param, Z_source);
    }

    // ========================================================================
    // FULL FREQUENCY SWEEP (1 MHz - 2 GHz, 200 points)
    // ========================================================================
    if (ENABLE_FREQUENCY_SWEEP) {
        FrequencySweepConfig config;
        config.f_start = 1.0e6;     // Match MATLAB: f = 1e6 (1 MHz)
        config.f_stop = 2.0e9;      // Match MATLAB: fmax = 2e9 (2 GHz)
        config.num_points = 200;    // Match MATLAB: pn = 200

        auto results = performFrequencySweep(config, a_param, b_param, d_param,
                                             p_param, l_param, w_param, t_param,
                                             Z_source);

        // Save results
        std::filesystem::path output_path =
            std::filesystem::current_path() / "SE_cpp_results.csv";

        if (exportToCSV(results, output_path.string())) {
            std::cout << "\nResults saved to: " << output_path.string() << "\n";

            // Display summary
            auto min_SE = *std::min_element(results.SE_dB.begin(), results.SE_dB.end());
            auto max_SE = *std::max_element(results.SE_dB.begin(), results.SE_dB.end());
            double avg_SE = std::accumulate(results.SE_dB.begin(), results.SE_dB.end(), 0.0)
                            / results.SE_dB.size();

            std::cout << "\nShielding Effectiveness Summary:\n";
            std::cout << "  Minimum:  " << std::setprecision(1) << min_SE << " dB\n";
            std::cout << "  Maximum:  " << max_SE << " dB\n";
            std::cout << "  Average:  " << std::setprecision(1) << avg_SE << " dB\n";

            // Find point 53 in results
            int point_53_idx = 52;  // 0-indexed (point 53 in MATLAB is index 52 in C++)
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

        std::cout << "\n========================================\n";
        std::cout << "Analysis complete.\n";
        std::cout << "========================================\n\n";

        std::cout << "Next steps:\n";
        std::cout << "1. Run MATLAB code to get reference values\n";
        std::cout << "2. Compare MATLAB output with C++ results\n";
        std::cout << "3. Look at point 53 for validation (~520 MHz)\n";
    }

    // Test Complete System (optional - can be run separately)
    // testCompleteSystem();

    return a.exec();
}
