#include "../mainwindow.h"

#include <QApplication>
#include "../include/core/PhysicsConstants.h"
#include "../include/core/BranchTemplate.h"
#include "../include/core/TL_EmptyCavity.h"
#include "../include/core/SRC_VoltageSource.h"
#include "../include/core/LOAD_Impedance.h"
#include "../include/core/MNASolver.h"

#include <iostream>
#include <iomanip>

// Windows-specific header for console encoding
#ifdef _WIN32
    #include <windows.h>
#endif

#include <memory>

int main(int argc, char *argv[])
{
    // ========================================================================
    // FIX UTF-8 CONSOLE ENCODING (Windows only)
    // ========================================================================
    #ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);  // Enable UTF-8 in Windows console
    #endif

    // ========================================================================
    // Qt APPLICATION SETUP
    // ========================================================================
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    // ========================================================================
    // PHYSICS CONSTANTS TEST
    // ========================================================================

    /*
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "=== Physics Constants Test ===\n";
    std::cout << "Speed of light: " << EMCore::C_LIGHT << " m/s\n";
    std::cout << "Permeability μ₀: " << EMCore::MU_0 << " H/m\n";
    std::cout << "Permittivity ε₀: " << EMCore::EPS_0 << " F/m\n";
    std::cout << "Impedance Z₀: " << EMCore::Z_0 << " Ω\n\n";

    std::cout << "=== Conversion Tests ===\n";
    std::cout << "20 dB → linear: " << EMCore::dB_to_linear(20.0) << "\n";
    std::cout << "10 linear → dB: " << EMCore::linear_to_dB(10.0) << " dB\n";
    std::cout << "Wavenumber @ 6 GHz: "
              << EMCore::frequency_to_wavenumber(6e9) << " rad/m\n";
    */

    // ========================================================================
    // BRANCH TEMPLATE TEST
    // ========================================================================

    // Test that BranchTemplate compiles
    /*
    std::cout << "BranchTemplate header compiled successfully!\n";

    // We can't instantiate it (it's abstract), but we can use the enum:
    EMCore::BranchTemplate::BranchType type = EMCore::BranchTemplate::BranchType::TL_EMPTY_CAVITY;
    std::cout << "Branch type enum works!\n";
    */

    /*
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "========================================\n";
    std::cout << "TL_EmptyCavity Validation Test\n";
    std::cout << "========================================\n\n";

    // ========================================================================
    // TEST CASE: Match your MATLAB reference
    // ========================================================================
    // Cavity dimensions (CHANGE THESE to match your MATLAB test case)
    double aa = 0.050;      // 50 mm width
    double b = 0.025;      // 25 mm height
    double L = 0.100;      // 100 mm length

    // Test frequency
    double f_test = 6.0e9; // 6 GHz

    // Create cavity branch
    EMCore::TL_EmptyCavity cavity(0, 1, 0, aa, b, L);

    // Print cavity info
    std::cout << "Cavity Configuration:\n";
    std::cout << "  " << cavity.getDescription() << "\n\n";

    // Check if valid
    std::string error_msg;
    if (!cavity.isValid(error_msg)) {
        std::cout << "ERROR: " << error_msg << "\n";
        return -1;
    }
    std::cout << "Validation: PASSED\n\n";

    // Compute Y-parameters
    std::cout << "Computing Y-parameters at f = " << f_test/1e9 << " GHz...\n\n";
    auto Y = cavity.computeYParameters(f_test);

    // Display results
    std::cout << "Y-Matrix (2x2):\n";
    std::cout << "Y11 = " << Y[0][0].real() << " + j*" << Y[0][0].imag() << " [S]\n";
    std::cout << "Y12 = " << Y[0][1].real() << " + j*" << Y[0][1].imag() << " [S]\n";
    std::cout << "Y21 = " << Y[1][0].real() << " + j*" << Y[1][0].imag() << " [S]\n";
    std::cout << "Y22 = " << Y[1][1].real() << " + j*" << Y[1][1].imag() << " [S]\n\n";

    // Magnitude and phase
    std::cout << "Y-Parameter Properties:\n";
    std::cout << "|Y11| = " << std::abs(Y[0][0]) << " S\n";
    std::cout << "|Y12| = " << std::abs(Y[0][1]) << " S\n";
    std::cout << "angle(Y11) = " << std::arg(Y[0][0]) * 180.0/M_PI << " degrees\n";
    std::cout << "angle(Y12) = " << std::arg(Y[0][1]) * 180.0/M_PI << " degrees\n\n";

    // Sanity checks
    std::cout << "Sanity Checks:\n";
    std::cout << "Y11 == Y22? " << (std::abs(Y[0][0] - Y[1][1]) < 1e-12 ? "YES" : "NO") << "\n";
    std::cout << "Y12 == Y21? " << (std::abs(Y[0][1] - Y[1][0]) < 1e-12 ? "YES" : "NO") << "\n\n";

    std::cout << "========================================\n";
    std::cout << "NOW COMPARE WITH YOUR MATLAB OUTPUT!\n";
    std::cout << "========================================\n";
    */

    /*

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "========================================\n";
    std::cout << "Branch Types Test\n";
    std::cout << "========================================\n\n";

    double f_test = 6.0e9;  // 6 GHz

    // Test 1: Voltage Source
    std::cout << "Test 1: Voltage Source\n";
    EMCore::SRC_VoltageSource source(0, 1, 0, EMCore::Complex(1.0, 0.0), 120.0*M_PI);
    std::cout << "  " << source.getDescription() << "\n";
    auto Y_src = source.computeYParameters(f_test);
    auto V_src = source.getVoltageSourceVector(f_test);
    std::cout << "  Y11 = " << Y_src[0][0] << " S\n";
    std::cout << "  V_vec = [" << V_src[0] << ", " << V_src[1] << "]\n\n";

    // Test 2: Empty Cavity
    std::cout << "Test 2: Empty Cavity\n";
    EMCore::TL_EmptyCavity cavity(1, 2, 1, 0.050, 0.025, 0.100);
    std::cout << "  " << cavity.getDescription() << "\n";
    auto Y_cav = cavity.computeYParameters(f_test);
    std::cout << "  Y11 = " << Y_cav[0][0] << " S\n";
    std::cout << "  Y12 = " << Y_cav[0][1] << " S\n\n";

    // Test 3: Load
    std::cout << "Test 3: Load Impedance\n";
    EMCore::LOAD_Impedance load(2, 0, 2, 50.0);  // 50 Ohm load
    std::cout << "  " << load.getDescription() << "\n";
    auto Y_load = load.computeYParameters(f_test);
    std::cout << "  Y11 = " << Y_load[0][0] << " S\n\n";

    std::cout << "========================================\n";
    std::cout << "All branch types compiled successfully!\n";
    std::cout << "========================================\n";
    */

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "========================================\n";
    std::cout << "1-Section SE Calculator Test\n";
    std::cout << "========================================\n\n";

    // Test frequency
    double f_test = 6.0e9;  // 6 GHz

    // Create MNA solver
    EMCore::MNASolver solver;

    // Add branches (must use shared_ptr for polymorphism)
    auto src = std::make_shared<EMCore::SRC_VoltageSource>(
        0, 1, 0, EMCore::Complex(1.0, 0.0), 120.0*M_PI
        );

    auto cavity = std::make_shared<EMCore::TL_EmptyCavity>(
        1, 2, 1, 0.050, 0.025, 0.100
        );

    auto load = std::make_shared<EMCore::LOAD_Impedance>(
        2, 0, 2, 50.0
        );

    solver.addBranch(src);
    solver.addBranch(cavity);
    solver.addBranch(load);

    std::cout << "Circuit topology:\n";
    std::cout << "  Node 0 (GND) -- [SRC] -- Node 1 -- [TL] -- Node 2 -- [LOAD] -- Node 0\n\n";

    // Solve
    std::cout << "Solving at f = " << f_test/1e9 << " GHz...\n\n";
    auto U = solver.solve(f_test);

    // Display results
    std::cout << "Node Voltages:\n";
    std::cout << "  V0 (ground) = 0 V (reference)\n";
    std::cout << "  V1 = " << U(0).real() << " + j*" << U(0).imag() << " V\n";
    std::cout << "  V2 = " << U(1).real() << " + j*" << U(1).imag() << " V\n\n";

    // Compute shielding effectiveness: SE = 20*log10(V1/V2)
    EMCore::Complex V1 = U(0);  // ← FIXED: Added EMCore::
    EMCore::Complex V2 = U(1);  // ← FIXED: Added EMCore::
    double SE_dB = 20.0 * std::log10(std::abs(V1 / V2));

    std::cout << "Shielding Effectiveness:\n";
    std::cout << "  |V1/V2| = " << std::abs(V1/V2) << "\n";
    std::cout << "  SE = " << SE_dB << " dB\n\n";

    std::cout << "========================================\n";


    return a.exec();
}
