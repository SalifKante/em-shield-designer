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

// Windows-specific header for console encoding
#ifdef _WIN32
#include <windows.h>
#endif

#include <memory>

// ============================================================================
// FREQUENCY SWEEP CONFIGURATION
// ============================================================================
struct FrequencySweepConfig {
    double f_start;      // Start frequency [Hz]
    double f_stop;       // Stop frequency [Hz]
    int num_points;      // Number of frequency points
    bool log_spacing;    // true = logarithmic, false = linear spacing

    FrequencySweepConfig(double start = 1e9, double stop = 20e9,
                         int points = 200, bool log = false)
        : f_start(start), f_stop(stop), num_points(points), log_spacing(log) {}
};

// ============================================================================
// FREQUENCY SWEEP RESULTS CONTAINER
// ============================================================================
struct FrequencySweepResults {
    std::vector<double> frequencies;
    std::vector<EMCore::Complex> V1;
    std::vector<EMCore::Complex> V2;
    std::vector<double> SE_dB;
    std::vector<double> transmission;

    double enclosure_width;
    double enclosure_height;
    double total_depth;
    double observation_point;
    double aperture_width;
    double aperture_height;
};

// ============================================================================
// GENERATE FREQUENCY VECTOR
// ============================================================================
std::vector<double> generateFrequencies(const FrequencySweepConfig& config) {
    std::vector<double> frequencies;
    frequencies.reserve(config.num_points);

    if (config.log_spacing) {
        double log_start = std::log10(config.f_start);
        double log_stop = std::log10(config.f_stop);
        double log_step = (log_stop - log_start) / (config.num_points - 1);

        for (int i = 0; i < config.num_points; ++i) {
            double log_f = log_start + i * log_step;
            frequencies.push_back(std::pow(10.0, log_f));
        }
    } else {
        double df = (config.f_stop - config.f_start) / (config.num_points - 1);

        for (int i = 0; i < config.num_points; ++i) {
            frequencies.push_back(config.f_start + i * df);
        }
    }

    return frequencies;
}

// ============================================================================
// PERFORM FREQUENCY SWEEP
// ============================================================================
FrequencySweepResults performFrequencySweep(
    const FrequencySweepConfig& config,
    double aa, double b, double d, double p,
    double l, double ww, double t, double Z_source)
{
    FrequencySweepResults results;

    results.enclosure_width = aa;
    results.enclosure_height = b;
    results.total_depth = d;
    results.observation_point = p;
    results.aperture_width = l;
    results.aperture_height = ww;

    results.frequencies = generateFrequencies(config);
    results.V1.reserve(config.num_points);
    results.V2.reserve(config.num_points);
    results.SE_dB.reserve(config.num_points);
    results.transmission.reserve(config.num_points);

    double L1 = p;
    double L2 = d - p;

    EMCore::MNASolver solver;

    auto src = std::make_shared<EMCore::SRC_VoltageSource>(
        0, 1, 0, EMCore::Complex(1.0, 0.0), Z_source
        );
    auto aperture = std::make_shared<EMCore::AP_SlotAperture>(
        1, 0, 1, aa, b, l, ww, t
        );
    auto cavity1 = std::make_shared<EMCore::TL_EmptyCavity>(
        1, 2, 2, aa, b, L1
        );
    auto cavity2 = std::make_shared<EMCore::TL_EmptyCavity>(
        2, 0, 3, aa, b, L2
        );

    solver.addBranch(src);
    solver.addBranch(aperture);
    solver.addBranch(cavity1);
    solver.addBranch(cavity2);

    std::cout << "\n========================================\n";
    std::cout << "Performing Frequency Sweep\n";
    std::cout << "========================================\n";
    std::cout << "Frequency Range: " << config.f_start/1e9 << " - "
              << config.f_stop/1e9 << " GHz (" << config.num_points << " points)\n\n";

    int progress_step = config.num_points / 20;
    if (progress_step == 0) progress_step = 1;

    for (int i = 0; i < config.num_points; ++i) {
        double f = results.frequencies[i];
        auto U = solver.solve(f);

        EMCore::Complex v1 = U(0);
        EMCore::Complex v2 = U(1);

        results.V1.push_back(v1);
        results.V2.push_back(v2);

        double se_db = -20.0 * std::log10(std::abs(2.0 * v2));
        results.SE_dB.push_back(se_db);

        double trans = std::abs(v2 / v1);
        results.transmission.push_back(trans);

        if (i % progress_step == 0) {
            int percent = (100 * i) / config.num_points;
            std::cout << "  Progress: " << std::setw(3) << percent << "% (f = "
                      << std::fixed << std::setprecision(2) << f/1e9 << " GHz)\n";
        }
    }

    std::cout << "  Progress: 100% - Complete!\n";
    std::cout << "========================================\n\n";

    return results;
}

// ============================================================================
// EXPORT TO CSV
// ============================================================================
bool exportToCSV(const FrequencySweepResults& results, const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open file: " << filename << "\n";
        return false;
    }

    file << "# EMShieldDesigner - Frequency Sweep Results\n";
    file << "# Single-section enclosure with aperture\n#\n";
    file << "f_GHz,SE_dB,Trans,V1_real,V1_imag,V2_real,V2_imag,|V1|,|V2|\n";

    file << std::fixed << std::setprecision(9);

    for (size_t i = 0; i < results.frequencies.size(); ++i) {
        file << results.frequencies[i] / 1e9 << ","
             << std::setprecision(6) << results.SE_dB[i] << ","
             << std::setprecision(9) << results.transmission[i] << ","
             << results.V1[i].real() << ","
             << results.V1[i].imag() << ","
             << results.V2[i].real() << ","
             << results.V2[i].imag() << ","
             << std::abs(results.V1[i]) << ","
             << std::abs(results.V2[i]) << "\n";
    }

    file.close();

    std::cout << "✓ Results exported to: " << filename << "\n";
    std::cout << "  Data points: " << results.frequencies.size() << "\n\n";

    return true;
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

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     EMShieldDesigner - Single-Section Aperture SE          ║\n";
    std::cout << "║       (Two-segment TL model, observation at p=150mm)       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // Physical parameters
    double aa = 0.050;
    double b = 0.025;
    double d = 0.300;
    double p = 0.150;
    double t = 0.0015;
    double l = 0.08;
    double ww = 0.08;
    double Z_source = 120.0 * M_PI;

    std::cout << "Physical Parameters:\n";
    std::cout << "  a = " << aa*1000 << " mm\n";
    std::cout << "  b = " << b*1000 << " mm\n";
    std::cout << "  d = " << d*1000 << " mm\n";
    std::cout << "  p = " << p*1000 << " mm\n";
    std::cout << "  l = " << l*1000 << " mm\n";
    std::cout << "  w = " << ww*1000 << " mm\n";
    std::cout << "  t = " << t*1000 << " mm\n";
    std::cout << "  Zs = " << Z_source << " Ohm\n\n";

    // ========================================================================
    // SINGLE-POINT TEST @ 6.035 GHz (for MATLAB comparison)
    // ========================================================================
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║         SINGLE-POINT TEST @ 6.035 GHz                      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    double f_test = 6.035e9;  // Match MATLAB frequency sweep point

    EMCore::MNASolver test_solver;
    test_solver.setDebugMode(true);

    auto test_src = std::make_shared<EMCore::SRC_VoltageSource>(
        0, 1, 0, EMCore::Complex(1.0, 0.0), Z_source
        );
    auto test_aperture = std::make_shared<EMCore::AP_SlotAperture>(
        1, 0, 1, aa, b, l, ww, t
        );
    auto test_cavity1 = std::make_shared<EMCore::TL_EmptyCavity>(
        1, 2, 2, aa, b, p
        );
    auto test_cavity2 = std::make_shared<EMCore::TL_EmptyCavity>(
        2, 0, 3, aa, b, d - p
        );

    std::cout << "Branch Descriptions:\n";
    std::cout << "  Branch 0: " << test_src->getDescription() << "\n";
    std::cout << "  Branch 1: " << test_aperture->getDescription() << "\n";
    std::cout << "  Branch 2: " << test_cavity1->getDescription() << "\n";
    std::cout << "  Branch 3: " << test_cavity2->getDescription() << "\n\n";

    std::cout << "=== Individual Branch Y-Parameters @ " << f_test/1e9 << " GHz ===\n\n";

    auto Y_src = test_src->computeYParameters(f_test);
    std::cout << "Branch 0 (Source) Y-matrix:\n";
    std::cout << "  Y11 = " << Y_src[0][0] << " S\n";
    std::cout << "  Y12 = " << Y_src[0][1] << " S\n\n";

    auto Y_ap = test_aperture->computeYParameters(f_test);
    std::cout << "Branch 1 (Aperture) Y-matrix:\n";
    std::cout << "  Y11 = " << Y_ap[0][0] << " S\n";
    std::cout << "  Y12 = " << Y_ap[0][1] << " S\n\n";

    auto Y_tl1 = test_cavity1->computeYParameters(f_test);
    std::cout << "Branch 2 (TL1) Y-matrix:\n";
    std::cout << "  Y11 = " << Y_tl1[0][0] << " S\n";
    std::cout << "  Y12 = " << Y_tl1[0][1] << " S\n\n";

    auto Y_tl2 = test_cavity2->computeYParameters(f_test);
    std::cout << "Branch 3 (TL2) Y-matrix:\n";
    std::cout << "  Y11 = " << Y_tl2[0][0] << " S\n";
    std::cout << "  Y12 = " << Y_tl2[0][1] << " S\n\n";

    test_solver.addBranch(test_src);
    test_solver.addBranch(test_aperture);
    test_solver.addBranch(test_cavity1);
    test_solver.addBranch(test_cavity2);

    std::cout << "=== MNA Solver Output ===\n\n";
    auto U_test = test_solver.solve(f_test);

    std::cout << "\n=== Final Results @ " << f_test/1e9 << " GHz ===\n";
    std::cout << "V1 = " << U_test(0) << " V\n";
    std::cout << "V2 = " << U_test(1) << " V\n";
    double SE_test = -20.0 * std::log10(std::abs(2.0 * U_test(1)));
    std::cout << "SE = " << SE_test << " dB\n";
    std::cout << "(Compare with MATLAB output)\n\n";

    // ========================================================================
    // FREQUENCY SWEEP
    // ========================================================================
    FrequencySweepConfig config;
    config.f_start = 1.0e9;
    config.f_stop = 20.0e9;
    config.num_points = 200;
    config.log_spacing = false;

    auto results = performFrequencySweep(config, aa, b, d, p, l, ww, t, Z_source);

    // ========================================================================
    // EXPORT RESULTS
    // ========================================================================
    std::string output_filename = "C:/Users/User/Desktop/Works/Cand/projets/em-shield-designer/SE_sweep_single_section_aperture.csv";
    exportToCSV(results, output_filename);

    std::cout << "========================================\n";
    std::cout << "Summary:\n";
    std::cout << "========================================\n";
    auto min_SE = *std::min_element(results.SE_dB.begin(), results.SE_dB.end());
    auto max_SE = *std::max_element(results.SE_dB.begin(), results.SE_dB.end());
    std::cout << "  Min SE: " << min_SE << " dB\n";
    std::cout << "  Max SE: " << max_SE << " dB\n";
    std::cout << "  Frequency range: " << config.f_start/1e9 << " - " << config.f_stop/1e9 << " GHz\n";
    std::cout << "========================================\n\n";

    return a.exec();
}
