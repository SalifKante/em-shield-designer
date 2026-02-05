#include "../mainwindow.h"

#include <QApplication>
#include <QStandardPaths>
#include <QDir>
#include "../include/core/PhysicsConstants.h"
#include "../include/core/BranchTemplate.h"
#include "../include/core/TL_EmptyCavity.h"
#include "../include/core/SRC_VoltageSource.h"
#include "../include/core/LOAD_Impedance.h"
#include "../include/core/AP_SlotAperture.h"
#include "../include/core/MNASolver.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <filesystem>
#include <numeric>

#ifdef _WIN32
#include <windows.h>
#endif

#include <memory>

// ============================================================================
// DEBUG CONTROL FLAGS
// ============================================================================
constexpr bool ENABLE_SINGLE_POINT_TEST = true;   // Run validation test
constexpr bool ENABLE_FREQUENCY_SWEEP = true;     // Run full sweep

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
    std::vector<EMCore::Complex> V1;
    std::vector<EMCore::Complex> V2;
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

    EMCore::MNASolver solver;

    // Create branches in correct order (matching MATLAB)
    auto src = std::make_shared<EMCore::SRC_VoltageSource>(
        0, 1, 0, EMCore::Complex(1.0, 0.0), Z_source);

    auto aperture = std::make_shared<EMCore::AP_SlotAperture>(
        1, 0, 1, a, b, l, w, t);

    auto cavity1 = std::make_shared<EMCore::TL_EmptyCavity>(
        1, 2, 2, a, b, L1);

    auto cavity2 = std::make_shared<EMCore::TL_EmptyCavity>(
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

        EMCore::Complex v1 = U(0);
        EMCore::Complex v2 = U(1);

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

    EMCore::MNASolver solver;

    // Create test circuit (same as MATLAB)
    auto test_src = std::make_shared<EMCore::SRC_VoltageSource>(
        0, 1, 0, EMCore::Complex(1.0, 0.0), Z_source);

    auto test_aperture = std::make_shared<EMCore::AP_SlotAperture>(
        1, 0, 1, a, b, l, w, t);

    auto test_cavity1 = std::make_shared<EMCore::TL_EmptyCavity>(
        1, 2, 2, a, b, p);

    auto test_cavity2 = std::make_shared<EMCore::TL_EmptyCavity>(
        2, 0, 3, a, b, d - p);

    // Add branches in MATLAB order
    solver.addBranch(test_src);
    solver.addBranch(test_aperture);
    solver.addBranch(test_cavity1);
    solver.addBranch(test_cavity2);

    // Solve
    auto U_test = solver.solve(f_test);

    // Extract results
    EMCore::Complex V1 = U_test(0);
    EMCore::Complex V2 = U_test(1);
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
    // Using the active values from MATLAB code:
    // p=0.150; d=0.300; b=0.120; a=0.300; t=0.0015;
    // w=0.08; l=0.08;

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

    return a.exec();
}
