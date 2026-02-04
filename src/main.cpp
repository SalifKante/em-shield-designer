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

#ifdef _WIN32
#include <windows.h>
#endif

#include <memory>

// ============================================================================
// DEBUG CONTROL FLAGS
// ============================================================================
constexpr bool ENABLE_APERTURE_DEBUG = false;     // Detailed aperture calculations
constexpr bool ENABLE_MNA_DEBUG = false;          // Detailed MNA matrices
constexpr bool ENABLE_YPARAMETER_PRINT = true;    // Print Y-parameters
constexpr bool ENABLE_SINGLE_POINT_TEST = true;   // Run 6.035 GHz test
constexpr bool ENABLE_FREQUENCY_SWEEP = true;     // Run full sweep

// ============================================================================
// FREQUENCY SWEEP CONFIGURATION
// ============================================================================
struct FrequencySweepConfig {
    double f_start;
    double f_stop;
    int num_points;
    bool log_spacing;

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
// GENERATE FREQUENCY VECTOR (MATLAB-COMPATIBLE)
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
        // MATLAB-COMPATIBLE: step = range / num_points (not num_points-1)
        // This matches MATLAB: fstep = (fmax - f) / pn
        double df = (config.f_stop - config.f_start) / config.num_points;

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
    solver.setDebugMode(false);

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

    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              FREQUENCY SWEEP IN PROGRESS                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    std::cout << "Range: " << config.f_start/1e9 << " - " << config.f_stop/1e9
              << " GHz (" << config.num_points << " points)\n\n";

    int progress_step = config.num_points / 10;
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
            std::cout << "  [" << std::string(percent/5, '█') << std::string(20-percent/5, '░')
                      << "] " << std::setw(3) << percent << "% @ "
                      << std::fixed << std::setprecision(1) << f/1e9 << " GHz\n";
        }
    }

    std::cout << "  [████████████████████] 100% - Complete!\n\n";

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
    std::cout << "║          EMShieldDesigner - Aperture Analysis              ║\n";
    std::cout << "║     Single-Section Enclosure (observation at p=150mm)      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // Physical parameters (EXACTLY matching MATLAB variable names and order!)
    double aa = 0.050;   // Enclosure width [m]  (matches MATLAB: a = 0.050)
    double b = 0.025;    // Enclosure height [m] (matches MATLAB: b = 0.025)
    double d = 0.300;    // Enclosure depth [m]  (matches MATLAB: d = 0.300)
    double p = 0.150;    // Observation point [m](matches MATLAB: p = 0.150)
    double t = 0.0015;   // Wall thickness [m]   (matches MATLAB: t = 0.0015)
    double l = 0.08;     // Aperture width [m]   (matches MATLAB: l = 0.08)
    double ww = 0.08;    // Aperture height [m]  (matches MATLAB: w = 0.08)
    double Z_source = 120.0 * M_PI;  // Source impedance (matches MATLAB: Zs = 120*pi)

    std::cout << "Configuration (matching MATLAB parameters):\n";
    std::cout << "  Enclosure: a×b×d = " << aa*1000 << "×" << b*1000 << "×" << d*1000 << " mm\n";
    std::cout << "  Aperture:  l×w = " << l*1000 << "×" << ww*1000 << " mm @ front face\n";
    std::cout << "  Observer:  p = " << p*1000 << " mm from front\n";
    std::cout << "  Wall:      t = " << t*1000 << " mm\n";
    std::cout << "  Source:    Zs = " << std::setprecision(1) << Z_source << " Ω\n\n";

    // ========================================================================
    // SINGLE-POINT TEST @ 6.035 GHz (EXACT MATLAB COMPARISON)
    // ========================================================================
    if (ENABLE_SINGLE_POINT_TEST) {
        std::cout << "╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║       VALIDATION TEST @ 6.035 GHz (vs MATLAB)              ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

        // MATLAB frequency grid: f_start=1e9, f_step=95MHz, point 53 → 6.035 GHz
        double f_test = 6.035e9;

        EMCore::MNASolver test_solver;
        test_solver.setDebugMode(ENABLE_MNA_DEBUG);

        // Create branches in SAME ORDER as MATLAB:
        // Branch 0: Source (Node 0 → Node 1)
        auto test_src = std::make_shared<EMCore::SRC_VoltageSource>(
            0, 1, 0, EMCore::Complex(1.0, 0.0), Z_source
            );

        // Branch 1: Aperture (Node 1 → Node 0)
        auto test_aperture = std::make_shared<EMCore::AP_SlotAperture>(
            1, 0, 1,   // node_from=1, node_to=0, branch_id=1
            aa,        // enclosure_width_a = 0.050 m
            b,         // enclosure_height_b = 0.025 m
            l,         // aperture_width_l = 0.08 m
            ww,        // aperture_height_w = 0.08 m
            t          // wall_thickness_t = 0.0015 m
            );

        // Branch 2: TL1 (Node 1 → Node 2, length p)
        auto test_cavity1 = std::make_shared<EMCore::TL_EmptyCavity>(
            1, 2, 2, aa, b, p
            );

        // Branch 3: TL2 (Node 2 → Node 0, length d-p)
        auto test_cavity2 = std::make_shared<EMCore::TL_EmptyCavity>(
            2, 0, 3, aa, b, d - p
            );

        std::cout << "Branch Configuration:\n";
        std::cout << "  Branch 0: " << test_src->getDescription() << "\n";
        std::cout << "  Branch 1: " << test_aperture->getDescription() << "\n";
        std::cout << "  Branch 2: " << test_cavity1->getDescription() << "\n";
        std::cout << "  Branch 3: " << test_cavity2->getDescription() << "\n\n";

        if (ENABLE_YPARAMETER_PRINT) {
            std::cout << "═══════════════════════════════════════════════════════════\n";
            std::cout << "Y-Parameters @ " << std::setprecision(3) << f_test/1e9 << " GHz:\n";
            std::cout << "═══════════════════════════════════════════════════════════\n";

            auto Y_src = test_src->computeYParameters(f_test);
            std::cout << "Branch 0 (Source):\n";
            std::cout << "  Y11 = " << std::scientific << std::setprecision(6)
                      << Y_src[0][0] << " S\n";
            std::cout << "  Y12 = " << Y_src[0][1] << " S\n\n";

            auto Y_ap = test_aperture->computeYParameters(f_test);
            std::cout << "Branch 1 (Aperture):\n";
            std::cout << "  Y11 = " << Y_ap[0][0] << " S\n";
            std::cout << "  Y12 = " << Y_ap[0][1] << " S\n\n";

            auto Y_tl1 = test_cavity1->computeYParameters(f_test);
            std::cout << "Branch 2 (TL1, length=" << p*1000 << "mm):\n";
            std::cout << "  Y11 = " << Y_tl1[0][0] << " S\n";
            std::cout << "  Y12 = " << Y_tl1[0][1] << " S\n";
            std::cout << "  Y21 = " << Y_tl1[1][0] << " S\n\n";

            auto Y_tl2 = test_cavity2->computeYParameters(f_test);
            std::cout << "Branch 3 (TL2, length=" << (d-p)*1000 << "mm):\n";
            std::cout << "  Y11 = " << Y_tl2[0][0] << " S\n";
            std::cout << "  Y12 = " << Y_tl2[0][1] << " S\n";
            std::cout << "═══════════════════════════════════════════════════════════\n\n";
        }

        test_solver.addBranch(test_src);
        test_solver.addBranch(test_aperture);
        test_solver.addBranch(test_cavity1);
        test_solver.addBranch(test_cavity2);

        auto U_test = test_solver.solve(f_test);

        std::cout << "═══════════════════════════════════════════════════════════\n";
        std::cout << "RESULTS @ " << std::setprecision(3) << f_test/1e9 << " GHz:\n";
        std::cout << "═══════════════════════════════════════════════════════════\n";
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  V1 = (" << U_test(0).real() << ", " << U_test(0).imag() << ") V\n";
        std::cout << "  V2 = (" << U_test(1).real() << ", " << U_test(1).imag() << ") V\n";
        double SE_test = -20.0 * std::log10(std::abs(2.0 * U_test(1)));
        std::cout << "  SE = " << std::setprecision(2) << SE_test << " dB\n";
        std::cout << "═══════════════════════════════════════════════════════════\n\n";

        std::cout << "→ Compare with MATLAB output @ 6.035 GHz\n";
        std::cout << "  Expected (MATLAB): V1=(-1.415,+0.266), V2=(+0.180,+0.961), SE=-5.82dB\n\n";
    }

    // ========================================================================
    // OPTIONAL: FREQUENCY SWEEP
    // ========================================================================
    if (ENABLE_FREQUENCY_SWEEP) {
        FrequencySweepConfig config;
        config.f_start = 1.0e9;   // Match MATLAB: f = 1e9
        config.f_stop = 20.0e9;   // Match MATLAB: fmax = 20e9
        config.num_points = 200;  // Match MATLAB: pn = 200
        config.log_spacing = false;

        auto results = performFrequencySweep(config, aa, b, d, p, l, ww, t, Z_source);

        std::filesystem::path exe_path = std::filesystem::current_path();
        std::filesystem::path project_root = exe_path.parent_path().parent_path();
        std::filesystem::path output_path = project_root / "SE_sweep_single_section_aperture.csv";
        std::string output_filename = output_path.string();

        std::cout << "╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    EXPORTING RESULTS                       ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

        exportToCSV(results, output_filename);

        auto min_SE = *std::min_element(results.SE_dB.begin(), results.SE_dB.end());
        auto max_SE = *std::max_element(results.SE_dB.begin(), results.SE_dB.end());

        std::cout << "Summary:\n";
        std::cout << "  Frequency: " << config.f_start/1e9 << " - " << config.f_stop/1e9 << " GHz\n";
        std::cout << "  Points:    " << config.num_points << "\n";
        std::cout << "  SE range:  " << std::setprecision(1) << min_SE << " to " << max_SE << " dB\n";
        std::cout << "  Output:    " << output_filename << "\n\n";
        std::cout << "✓ Analysis complete!\n\n";
    }

    return a.exec();
}
