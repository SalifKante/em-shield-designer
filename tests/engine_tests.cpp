// ============================================================================
// ENGINE REGRESSION TESTS
// ============================================================================
// Checks the MNA engine against reference values for the four validation
// structures of the journal article (Table 2: b = 4 mm, t = 0.1 mm,
// observation point at the centre of section 1).
//
// The reference values were computed with an independent Python
// implementation of the same equivalent circuit (Eqs. 3.8-3.17 with the
// corrected Eq. 3.15). The two implementations agree to 1e-9 dB over
// 1001 frequencies; the tolerance below is 0.01 dB.
//
// Build and run:
//   cmake --build build --config Debug --target engine_tests
//   ctest --test-dir build -C Debug --output-on-failure
// ============================================================================

#include "core/CircuitGenerator.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace EMCore;

namespace {

struct RefPoint { double f_GHz; double se_dB; };

constexpr double MM  = 1e-3;
constexpr double TOL = 0.01;   // dB

SectionConfig section(double a_override, double depth, double l, double w)
{
    SectionConfig s;
    s.section_width_a = a_override;
    s.depth           = depth;
    s.obs_position    = depth / 2.0;
    s.aperture_l      = l;
    s.aperture_w      = w;
    return s;
}

EnclosureConfig paperStructure(int id)
{
    EnclosureConfig c;
    c.b = 4.0 * MM;
    c.t = 0.1 * MM;
    switch (id) {
    case 1:
        c.a = 10.0 * MM;
        c.sections = { section(-1.0, 10.0 * MM, 3.0 * MM, 1.0 * MM) };
        break;
    case 2:
        c.a = 15.0 * MM;
        c.sections = { section(-1.0, 15.0 * MM, 3.0 * MM, 1.0 * MM) };
        break;
    case 3:
        c.a = 10.0 * MM;
        c.sections = { section(-1.0, 10.0 * MM, 2.0 * MM, 2.0 * MM),
                       section(-1.0,  5.0 * MM, 2.0 * MM, 2.0 * MM) };
        break;
    default:
        c.a = 15.0 * MM;
        c.topology = TopologyType::STAR_BRANCH;
        c.sections = { section(-1.0,     15.0 * MM, 2.0 * MM, 2.0 * MM),
                       section(7.5 * MM, 10.0 * MM, 2.0 * MM, 2.0 * MM),
                       section(7.5 * MM, 10.0 * MM, 2.0 * MM, 2.0 * MM) };
        break;
    }
    return c;
}

const std::vector<RefPoint> REFERENCE[4] = {
    { {1.0, 60.501381}, {5.0, 45.756364}, {10.0, 37.126016}, {18.0, 20.016422}, {26.0, 17.504408}, {38.0, 13.910679} },
    { {1.0, 63.815173}, {5.0, 48.169170}, {10.0, 35.489547}, {18.0, 26.611693}, {26.0, 23.435492}, {38.0, 15.813371} },
    { {1.0, 64.508939}, {5.0, 49.804359}, {10.0, 41.305256}, {18.0, 24.623152}, {26.0, 23.256750}, {38.0, 21.131619} },
    { {1.0, 67.896450}, {5.0, 52.292315}, {10.0, 39.747694}, {18.0, 31.459560}, {26.0, 28.658562}, {38.0, 25.069300} },
};

int checkStructure(int id)
{
    const EnclosureConfig cfg = paperStructure(id);
    MNASolver solver;
    const auto obs = CircuitGenerator::generate(cfg, solver);

    int failures = 0;
    for (const RefPoint& r : REFERENCE[id - 1]) {
        const double se  = CircuitGenerator::computeSE(solver, obs, cfg, r.f_GHz * 1e9).front();
        const double err = std::abs(se - r.se_dB);
        const bool   ok  = err <= TOL;
        if (!ok) ++failures;
        std::printf("  S%d  f = %5.1f GHz   SE = %10.6f dB   ref = %10.6f dB   %s\n",
                    id, r.f_GHz, se, r.se_dB, ok ? "ok" : "FAIL");
    }
    return failures;
}

int checkWallThicknessGuard()
{
    int failures = 0;
    EnclosureConfig cfg = paperStructure(1);
    std::string msg;

    const bool thinAccepted = cfg.isValid(msg);
    cfg.t = 1.0 * MM;                       // w = 1 mm slot: effective width < 0
    const bool thickRejected = !cfg.isValid(msg);

    if (!thinAccepted)  { ++failures; std::printf("  t = 0.1 mm should be accepted: FAIL\n"); }
    if (!thickRejected) { ++failures; std::printf("  t = 1.0 mm should be rejected: FAIL\n"); }
    if (thinAccepted && thickRejected) std::printf("  wall-thickness guard            ok\n");
    return failures;
}

} // namespace

int main()
{
    int failures = 0;
    std::printf("Engine regression tests (tolerance %.2f dB)\n", TOL);
    for (int id = 1; id <= 4; ++id) failures += checkStructure(id);
    failures += checkWallThicknessGuard();

    if (failures == 0) {
        std::printf("All tests passed.\n");
        return 0;
    }
    std::printf("%d check(s) FAILED.\n", failures);
    return 1;
}
