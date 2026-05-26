# CLAUDE.md — EMShieldDesigner

This file is read by Claude Code at the start of every session. It defines the
project, its conventions, its state, and how Claude must work in this codebase.
**Read it fully before touching any file.**

---

## Table of Contents

1. [Project Identity](#1-project-identity)
2. [Technology Stack](#2-technology-stack)
3. [Build Setup — This PC (Home, MSVC)](#3-build-setup--this-pc-home-msvc)
4. [Build Setup — Work PC (MinGW)](#4-build-setup--work-pc-mingw)
5. [Source Layout](#5-source-layout)
6. [Architecture](#6-architecture)
7. [Physics Formulas](#7-physics-formulas)
8. [Engineering Rules](#8-engineering-rules)
9. [Design Tokens](#9-design-tokens)
10. [Project State](#10-project-state)
11. [Planned Tasks](#11-planned-tasks)
12. [Known Issues and Trade-Offs](#12-known-issues-and-trade-offs)
13. [Working with Claude Code in This Repo](#13-working-with-claude-code-in-this-repo)
14. [Lessons Learned](#14-lessons-learned)

---

## 1. Project Identity

**Name:** EMShieldDesigner
**Owner:** Salif Kante, PhD candidate at TUSUR (Tomsk State University of
Control Systems and Radioelectronics)
**Repository:** https://github.com/SalifKante/em-shield-designer (single
branch `main`, no PR workflow — direct pushes to `main`)
**Defence date:** 21 May 2026 — completed successfully.
**Current phase:** Post-defence polish and finalisation. No deadline pressure.
The work that remains is quality-of-life refactoring, internationalisation,
and Release packaging.

**Purpose:** Desktop application that computes the electromagnetic shielding
effectiveness (SE) of metallic enclosures using a hybrid analytical method.
The method combines equivalent-circuit decomposition (per dissertation Chapter
3.1, Fig 3.7 / 3.10 / 3.11) with Modified Method of Nodal Potentials (MMNP)
for solving the resulting linear system. Two operating modes:

- **Quick Simulation** — parametric front-end. User enters enclosure
  dimensions, aperture size, frequency range. Engine builds the canonical
  multi-section circuit automatically.
- **Circuit Builder** — visual schematic editor. User drags and drops
  individual circuit elements (Source, Aperture, Cavity, Obs.Pt, …) and the
  engine computes SE for whatever topology was assembled.

Both modes produce SE-vs-frequency plots and CSV export. Validation against
FEM reference data on four test structures gave mean deviation 3.16–4.98 dB,
matching or beating the dissertation's own published 4.5–7.4 dB.

---

## 2. Technology Stack

| Layer | Technology |
|---|---|
| Language | C++20 (`set(CMAKE_CXX_STANDARD 20)`) |
| UI framework | Qt 6.10.1 (home PC, MSVC) / Qt 6.9.3 (work PC, MinGW) |
| Qt modules used | Core, Widgets, PrintSupport |
| Linear algebra | Eigen 3 (header-only, vendored at `external/eigen/eigen-5.0.0/`) |
| Plotting | QCustomPlot 2.x (vendored at project root: `qcustomplot.h/.cpp`) |
| Build system | CMake ≥ 3.19, Ninja Multi-Config generator |
| IDE | Visual Studio Code (primary, current) / Qt Creator (legacy, still works) |
| Compiler (home) | MSVC 2022 Build Tools (cl.exe v17.14.32) |
| Compiler (work) | MinGW 13.1.0 64-bit shipped with Qt 6.9.3 |
| Debugger | cppvsdbg (Microsoft) on home PC, gdb on work PC |
| Version control | git + GitHub |
| OS | Windows 10/11 (primary), no Linux/macOS support targeted |

The application is single-platform Windows for distribution. Cross-platform
fields like `MACOSX_BUNDLE` exist in CMakeLists.txt but are harmless leftovers.

---

## 3. Build Setup — This PC (Home, MSVC)

**Machine:** `Salif Kante`'s personal PC.
**Project path:** `C:\Users\Salif Kante\Desktop\cand\projects\em-shield-designer`

### Paths

| Component | Location |
|---|---|
| Qt | `C:\Qt\6.10.1\msvc2022_64` |
| Qt CMake package | `C:/Qt/6.10.1/msvc2022_64/lib/cmake/Qt6` |
| CMake (Qt-bundled) | `C:\Qt\Tools\CMake_64\bin\cmake.exe` |
| CMake (VS-bundled, actually used by CMake Tools) | `C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.EXE` |
| Ninja (Qt-bundled) | `C:\Qt\Tools\Ninja\ninja.exe` |
| MSVC compiler | `cl.exe` (resolved via VS 2022 Build Tools developer environment) |
| Visual Studio 2022 Build Tools | `C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools` |
| VS Code | Installed via standard installer, current version 1.120.0 |

### VS Code workspace files

All four live in `.vscode/` (gitignored — must be recreated per machine):

- `settings.json` — CMake configuration: prefix path, generator, build dir
- `c_cpp_properties.json` — IntelliSense fallback (becomes secondary once
  `compile_commands.json` exists)
- `tasks.json` — five tasks: Configure / Build Debug / Build Release / Clean / Run Debug
- `launch.json` — two F5 targets: Debug and Release, both extend PATH with Qt's bin

### Build procedure

```powershell
# In VS Code's integrated terminal, from project root:
# (1) Configure happens automatically on workspace open. Manual trigger:
#     Ctrl+Shift+P → "CMake: Configure"
# (2) Build:
#     Ctrl+Shift+B → "CMake: Build (Debug)"   (default task)
# (3) Run with debugger:
#     F5
# (4) Run without debugger:
$env:PATH = "C:\Qt\6.10.1\msvc2022_64\bin;" + $env:PATH
.\build\Debug\em-shield-designer.exe
```

### Build directory layout

VS Code's CMake Tools writes to `build/` directly (NOT into a nested
`build/Desktop_Qt_...` folder). Layout after a successful build:

```
build/
├── compile_commands.json    ← IntelliSense reads this
├── build-Debug.ninja
├── build-Release.ninja
├── build-RelWithDebInfo.ninja
├── build.ninja
├── CMakeCache.txt.prev
├── Debug/
│   ├── em-shield-designer.exe
│   ├── em-shield-designer.pdb
│   └── em-shield-designer.ilk
├── Release/
└── RelWithDebInfo/
```

**Note:** the legacy `build/Desktop_Qt_6_10_1_MSVC2022_64bit-Debug/` folder
from Qt Creator coexists in `build/`. Harmless — both build systems share
the parent directory without interfering. Do not delete unless asked.

### Build time benchmarks

- Fresh configure: ~5.3 seconds
- Full Debug build: ~16.6 seconds
- Incremental Debug build (1 file changed): ~3 seconds

---

## 4. Build Setup — Work PC (MinGW)

**Path on work PC:** `C:\Users\Salif Kante\Desktop\cand\projets\em-shield-designer\`
**Toolchain:** Qt 6.9.3 MinGW 13.1.0 64-bit, located at `C:\DesktopAppDev\Qt\6.9.3\mingw_64\`

This machine has Qt Creator working. When VS Code is set up here, the four
`.vscode/` files must be recreated with paths adjusted for MinGW. Replace:

- `CMAKE_PREFIX_PATH` from `C:/Qt/6.10.1/msvc2022_64` → `C:/DesktopAppDev/Qt/6.9.3/mingw_64`
- `compilerPath` from `cl.exe` → `C:/DesktopAppDev/Qt/Tools/mingw1310_64/bin/g++.exe`
- `intelliSenseMode` from `windows-msvc-x64` → `windows-gcc-x64`
- launch type from `cppvsdbg` → `cppdbg` with `MIMode: gdb`

**Status:** Work PC migration not yet executed. Repeat the home-PC procedure
when ready. Document path differences here as they're discovered.

---

## 5. Source Layout

```
em-shield-designer/                              project root
├── .git/                                        version control
├── .gitignore                                   excludes build/, .vscode/, *.exe, etc.
├── .qtcreator/                                  legacy Qt Creator state (untouched)
├── .vscode/                                     VS Code workspace (per-machine, gitignored)
├── build/                                       all build outputs (gitignored)
├── CMakeLists.txt                               build definition (see notes below)
│
├── src/                                         non-header implementations
│   ├── main.cpp                                 application entry point
│   └── core/
│       └── MNASolver.cpp                        MNA solve(f), assembleY() implementation
│
├── include/core/                                physics engine, header-only except MNASolver
│   ├── PhysicsConstants.h                       Z_0, c, ε₀, μ₀ exact values
│   ├── BranchTemplate.h                         abstract base for all circuit branches
│   ├── MNASolver.h                              MNA matrix assembly + solve
│   ├── SRC_VoltageSource.h                      excitation source (Z_0 series)
│   ├── LOAD_Impedance.h                         shunt termination / observation tap
│   ├── AP_SlotAperture.h                        Eq. 3.13 — open slot aperture
│   ├── AP_SlotWithCover.h                       Eq. 3.22/3.24 — covered aperture
│   ├── TL_EmptyCavity.h                         Eqs. 3.14–3.17 — air waveguide
│   ├── TL_DielectricCavity.h                    Eq. 3.19 — dielectric-loaded waveguide
│   └── CircuitGenerator.h                       EnclosureConfig → MNA circuit topology
│                                                 (CASCADE per Fig 3.10 / STAR_BRANCH per Fig 3.11)
│
├── external/eigen/eigen-5.0.0/                  Eigen 3 header-only (vendored)
│
├── resources/
│   ├── resources.qrc                            Qt resource manifest
│   ├── app.rc                                   Windows resource (embeds .ico into .exe)
│   └── icons/
│       ├── emshield.ico                         multi-resolution Windows icon
│       ├── emshield_16.png … emshield_256.png   PNG variants
│       └── emshield_preview.png
│
├── qcustomplot.h, qcustomplot.cpp               QCustomPlot 2.x (vendored)
│
├── mainwindow.h / .cpp / .ui                    Window 1: Quick Simulation
├── CircuitCanvas.h                              Window 1: schematic section visualiser
├── SectionItem.h                                Window 1: one drawn section on the canvas
├── PropertyPanel.h                              Window 1: properties editor (right panel)
│
├── CircuitBuilderWindow.h                       Window 2: Circuit Builder
├── StackLayerPanel.h                            Window 2: layer-stack info panel (right side)
│
├── StartupWindow.h                              mode picker (entry screen)
├── MessageDialog.h                              branded error/success dialog (replaces QMessageBox)
└── Styles.h                                     centralised EMStyle QSS helpers
```

### File-name conventions

- **Engine files in `include/core/`** use `snake_case` prefixes by category:
  `AP_*` = apertures, `TL_*` = transmission lines / cavities, `SRC_*` =
  sources, `LOAD_*` = loads/observation. `MNASolver`, `BranchTemplate`,
  `CircuitGenerator`, `PhysicsConstants` are standalone.
- **UI files at project root** use `PascalCase.h` (header-only by convention):
  `CircuitBuilderWindow`, `StackLayerPanel`, `MessageDialog`, `StartupWindow`,
  `PropertyPanel`, `SectionItem`, `CircuitCanvas`, `Styles`.
- Most UI classes are **header-only with `Q_OBJECT`**. Qt's AUTOMOC (enabled
  by `qt_standard_project_setup()` in CMakeLists.txt) handles them. Do NOT
  manually create `.cpp` files for these unless adding non-inline methods
  that benefit from out-of-line definitions.
- Exception: `mainwindow.cpp` exists as the only `.cpp` for the UI layer
  because `mainwindow.ui` requires it (Qt Designer convention).

### CMakeLists.txt notes

The file is shown in full in your project. Key facts:

- **`qt_add_executable`** lists every source file explicitly — there is NO
  glob pattern. If you add a new header, you MUST add it to the
  `qt_add_executable(...)` list, otherwise AUTOMOC will not see it.
- **`add_definitions(-DDEBUG_APERTURE)`** is enabled unconditionally. This
  compiles aperture-related `qDebug()` statements into BOTH Debug and Release
  builds. Should be made conditional on `$<CONFIG:Debug>` before final
  release distribution.
- **Windows resource script** (`resources/app.rc`) is included only when
  building on Windows (`if(WIN32)`).
- **Eigen** is added via `include_directories(${CMAKE_SOURCE_DIR}/external/eigen/eigen-5.0.0)`
  rather than `target_include_directories`. Works, but the modern target-based
  form is `target_include_directories(em-shield-designer PRIVATE …)`. Leave
  as-is unless refactoring is requested.
- **`qt_generate_deploy_app_script`** produces a deployment script that runs
  `windeployqt` at install time. Use this for Release packaging.

---

## 6. Architecture

Three-tier separation:

```
┌────────────────────────────┐
│  UI Layer (presentation)   │   StartupWindow → (mainwindow.* OR CircuitBuilderWindow.h)
│                            │   Property panels, canvas, plot widget
└──────────────┬─────────────┘
               │ EnclosureConfig (Quick Sim) / live CanvasElement list (Circuit Builder)
               ▼
┌────────────────────────────┐
│  Circuit-Generator Layer   │   CircuitGenerator::generate() — Quick Sim only
│                            │   Inline assembly inside runCompute() — Circuit Builder
└──────────────┬─────────────┘
               │ populated MNASolver
               ▼
┌────────────────────────────┐
│  Engine Layer (physics)    │   MNASolver, BranchTemplate subclasses
│                            │   Eigen linear solve, Eq. 3.8 SE computation
└────────────────────────────┘
```

### Equation map (dissertation Chapter 3.1 → code)

| Eq | Description | Implemented in |
|---|---|---|
| 3.8 | `SE = -20·log₁₀|2·U_obs/V₀|` | `MNASolver::computeSE()` |
| 3.9 | `A·Y·Aᵀ·U = -A·Y·V` MNA system | `MNASolver::solve(f)` |
| 3.10 | Incidence matrix `A` construction | `MNASolver::assembleA()` |
| 3.11 | Source vector `V` (only V₀ on source row) | `MNASolver::assembleV()` |
| 3.12 | `Y_src = 1/Z₀` | `SRC_VoltageSource::stamp()` |
| 3.13 | `Y_ap` slot-aperture admittance | `AP_SlotAperture::stamp()` |
| 3.14–3.17 | TL admittance: `Y₁₁=1/(jZ tan kgL)`, `Y₁₂=−1/(Z sin kgL)`, `kg=k₀√(1−(λ/2a)²)`, `Z_g=2πf·μ₀/k_g` | `TL_EmptyCavity::stamp()` |
| 3.18 | Single-section full `Y` matrix | implicit in `MNASolver` per-branch stamps |
| 3.19 | Dielectric-loaded `kg = (2π√εeff/λ)·√(1−(λ/2a√εeff)²)` | `TL_DielectricCavity::stamp()` |
| 3.20 | Maxwell-Garnett εeff | optional in `TL_DielectricCavity` |
| 3.21 | Lichtenecker εeff (default) | `TL_DielectricCavity::stamp()` |
| 3.22 | Air-gap covered aperture | `AP_SlotWithCover::stamp(air)` |
| 3.24 | Dielectric-gap covered aperture | `AP_SlotWithCover::stamp(dielectric)` |
| 3.25 | 2-section cascade Y matrix | `CircuitGenerator::generateCascade()` (N=2) |
| 3.26 | 3-section star-branch Y matrix | `CircuitGenerator::generateStarBranch()` (N=3) |

### Topology: CASCADE vs STAR_BRANCH

Quick Simulation chooses topology via `EnclosureConfig::topology`:

- **CASCADE (Fig 3.10b):** N sections stacked in depth. Each section feeds
  into the next via its own aperture wall. Branch count: 3N+1. Node count: 2N.
- **STAR_BRANCH (Fig 3.11b):** Section 1 is the spine. Sections 2…N hang as
  independent side branches off the spine's output junction. Branch count:
  3N+1. Node count: N+2.

Circuit Builder does not have an explicit topology selector — the topology is
defined by the user's element placement on the canvas. The runCompute logic
walks elements left-to-right and connects them by node index.

### Observation-point convention

In every model, the observation point `P_i` is located at offset `p_i` from
the front wall of section `i`. This splits the section's transmission line
into two halves: `TL_p` (front wall → observation node) and `TL_{d-p}`
(observation node → back wall). The back wall of the deepest section is
short-circuited to ground (Eq. 3.10's last column).

This convention is critical for understanding why Circuit Builder needed
the [T3.2-TEMPORARY] validator relaxation — see Lessons Learned section.

---

## 7. Physics Formulas

This section reproduces every equation from dissertation Chapter 3.1 in
plain-text Unicode form. Each formula is paired with its variable
definitions, a physical interpretation (for major equations), and a
pointer to the file/function that implements it.

**Units convention throughout:** SI units everywhere — metres for length,
hertz for frequency, ohms for impedance, siemens for admittance. UI
displays may use mm/GHz but always convert to SI before reaching engine
code.

### 7.1 — Shielding effectiveness and MNA system

#### Eq. 3.8 — Shielding effectiveness

```
SE = -20 · log₁₀ | 2·U₂ / V₀ |   [dB]
```

Variables:
- `U₂` — complex potential at the observation node (node 2 in the basic
  single-section model). Found from the MNA solve.
- `V₀` — source EMF magnitude. Always 1 V/m in our convention so that
  the field-reduction interpretation holds directly.
- The factor of 2 corrects for the Z₀ source impedance: without an
  enclosure, the load would receive V₀/2 (matched source-load divider),
  so the bare-source reference is V₀/2 and the ratio of "no-shield" to
  "with-shield" field amplitudes equals `|V₀/(2·U₂)|`.

**Physical interpretation:** SE measures how many decibels the
enclosure attenuates the incident plane wave at the observation point.
Higher = better shielding. 0 dB = no attenuation, 60 dB = field reduced
1000-fold, 120 dB = field reduced 10⁶-fold. Resonance dips can give
negative SE (the enclosure actually amplifies the field at that point).

Implemented in: `MNASolver.cpp::computeSE(double f)`.

---

#### Eq. 3.9 — Modified Nodal Potential system

```
A · Y · Aᵀ · U  =  −A · Y · V
```

Variables:
- `A` — incidence (connection) matrix, size `n_nodes × n_branches`.
  Each column corresponds to one branch, with `+1` at the "from" node,
  `−1` at the "to" node, `0` elsewhere.
- `Y` — diagonal branch-admittance matrix (or block-diagonal for
  2-port branches like transmission lines), size `n_branches × n_branches`.
- `Aᵀ` — transpose of `A`.
- `U` — vector of unknown node potentials, size `n_nodes`.
- `V` — vector of branch source EMFs, size `n_branches`. Only the
  excitation source row is non-zero (= V₀).

**Physical interpretation:** This is the matrix form of Kirchhoff's
current law plus Ohm's law for the entire circuit. The product `A·Y·Aᵀ`
is the "nodal admittance matrix" — its `(i,j)` entry is the sum of
admittances between nodes `i` and `j`. The right-hand side `−A·Y·V`
injects source currents at the appropriate nodes. Solving for `U` gives
all node potentials in ONE linear solve, regardless of circuit
complexity. This is the modified version of nodal analysis — it
handles ideal voltage sources (zero internal impedance) cleanly,
which classical nodal analysis cannot.

**Why MMNP wins over Thévenin–Helmholtz transformations:** for an
N-section enclosure, classical analytical reduction requires N separate
Thévenin transformations. MMNP solves all N observation potentials in
one matrix inversion. The Y matrix rebuilds per frequency point; the A
matrix is built once.

Implemented in: `MNASolver.cpp::solve(double f)`.

---

### 7.2 — MNA matrix construction

#### Eq. 3.10 — Incidence matrix structure (single-section example)

```
       branch:  I    II   III  IV

       node 1 [ -1   +1   +1    0 ]
A  =   node 2 [  0    0   -1   +1 ]
```

Variables:
- Branch I — source branch (`V₀` with internal impedance `Z₀`)
- Branch II — aperture branch (`Z_ap`)
- Branch III — transmission-line segment of length `p`
- Branch IV — transmission-line segment of length `d−p`
- Last row of `A` is dropped (ground reference). The full matrix
  before dropping ground row would have an additional row for the
  back-wall short.

**Physical interpretation:** Each column shows where one branch's
current enters (+1) and exits (−1) the network. Reading rows: node 1
sees source current arriving (−1 means leaving the source, arriving at
node 1; or equivalently, the source acts as a current generator into
node 1), and the aperture and front TL segment also connect there.
Node 2 sees the front TL segment leaving and the back TL segment
arriving. The structure scales: for N sections, A has `2N` rows and
`3N+1` columns (CASCADE) or `N+2` rows and `3N+1` columns (STAR_BRANCH).

Implemented in: `MNASolver.cpp::assembleA()`.

---

#### Eq. 3.11 — Source vector

```
       [ V₀ ]
       [  0 ]
V  =   [  0 ]
       [  0 ]
```

Variables:
- Only the source branch (Branch I) has a non-zero EMF entry. All
  other branches are passive — their `V` entries are zero.

**Physical interpretation:** This vector tells the MNA system "Branch I
is being driven by an ideal source of V₀ volts". The `−A·Y·V` product
then translates this branch-level excitation into a node-level current
injection.

Implemented in: `MNASolver.cpp::assembleV()`.

---

#### Eq. 3.12 — Source branch admittance

```
Y_src = 1 / Z₀
```

Variables:
- `Z₀ = 120π Ω ≈ 376.73 Ω` — wave impedance of free space.

The source branch is treated as an ideal voltage source `V₀` in series
with the wave impedance `Z₀`. The branch admittance is the inverse of
its impedance.

Implemented in: `SRC_VoltageSource.h::stamp()`.

---

### 7.3 — Single-section model (apertures and empty waveguide)

#### Eq. 3.13 — Open slot aperture admittance

```
Y_ap = (2a / l) · 1/(jZ_0s) · cot(πl / λ)
```

Variables:
- `a` — broad-wall width of the cavity (m)
- `l` — aperture length, the dimension PARALLEL to the E-field (m)
- `λ` — free-space wavelength = `c/f` (m)
- `Z_0s` — characteristic impedance of an equivalent coplanar
  strip transmission line whose conductor spacing equals `w`
  (the aperture WIDTH, perpendicular to E-field)
- `cot` = `1/tan`

**Physical interpretation:** A rectangular slot in a metallic wall
behaves like a short-circuited coplanar strip line of length `l/2`
(quarter-wave resonator) seen from both sides. The factor `(2a/l)`
accounts for the slot's position along the broad wall — slots offset
from the wall centre couple less strongly. `Z_ap = 1/Y_ap` is the
complex impedance that appears across the equivalent circuit's
aperture branch.

Implemented in: `AP_SlotAperture.h::stamp(double f)`.

---

#### Eq. 3.14 — Transmission line self-admittance

```
Y₁₁ = Y₂₂ = 1 / (j · Z_g · tan(k_g · p))
```

Variables:
- `Z_g` — characteristic impedance of the cavity-waveguide mode (Ω)
- `k_g` — propagation constant of the cavity-waveguide mode (rad/m)
- `p` — physical length of this TL segment (m)
- `j` — imaginary unit
- Indices `1`, `2` — input and output ports of the 2-port branch.

**Physical interpretation:** The TL segment is a 2-port network. Its
Y matrix is symmetric (passive, reciprocal). `Y₁₁` and `Y₂₂` are the
self-admittances looking into either port when the other port is
short-circuited. They represent how much current the port draws for
unit voltage on itself, accounting for the standing-wave structure
inside the segment. At resonance (`k_g·p = nπ`), `tan → 0` and
`Y₁₁ → ∞` — the segment looks like a short.

Implemented in: `TL_EmptyCavity.h::stamp(double f)` (port matrix
elements emitted into the MNA solver's Y).

---

#### Eq. 3.15 — Transmission line transfer admittance

```
Y₁₂ = Y₂₁ = -1 / (Z_g · sin(k_g · p))
```

Variables:
- Same as Eq. 3.14.
- The negative sign comes from the sign convention for port currents
  (both currents defined as entering their respective ports).

The off-diagonal terms describe how voltage at port 1 produces current
at port 2 and vice versa.

Implemented in: `TL_EmptyCavity.h::stamp(double f)`.

---

#### Eq. 3.16 — Empty-waveguide propagation constant

```
k_g = (2π/λ) · √(1 − (λ / 2a)²)
```

Variables:
- `λ` — free-space wavelength (m)
- `a` — broad-wall dimension of the rectangular waveguide (m)
- `2a` — cut-off wavelength of the TE₁₀ mode = `λ_c`

**Physical interpretation:** A rectangular waveguide supports
propagating waves only above the TE₁₀ cut-off frequency `f_c = c/(2a)`.
Below cut-off, `k_g` becomes imaginary and the wave is evanescent
(attenuates exponentially with distance). Above cut-off, the wave
propagates with a longer effective wavelength than free space (the
"guided wavelength"). The square root makes `k_g → 0` at cut-off
and `k_g → 2π/λ` as `f → ∞`.

Implemented in: `TL_EmptyCavity.h::propagationConstant(double f)`.

---

#### Eq. 3.17 — Empty-waveguide characteristic impedance

```
Z_g = 2π·f · μ₀ / k_g
```

Variables:
- `f` — operating frequency (Hz)
- `μ₀ = 4π × 10⁻⁷ H/m` — vacuum permeability
- `k_g` — from Eq. 3.16

`Z_g` is the wave impedance of the TE₁₀ mode — the ratio of transverse
electric field to transverse magnetic field for that mode. Diverges to
infinity at cut-off (where `k_g → 0`) and asymptotes to `Z₀ = 377 Ω`
far above cut-off.

Implemented in: `TL_EmptyCavity.h::characteristicImpedance(double f)`.

---

#### Eq. 3.18 — Single-section full Y matrix

```
        [ Y_src    0        0       0       ]
Y    =  [   0    Y_ap       0       0       ]
        [   0      0      Y₁₁ᴵᴵᴵ  Y₁₂ᴵᴵᴵ   ]
        [   0      0      Y₂₁ᴵᴵᴵ  Y₂₂ᴵᴵᴵ + Y₁₁ᴵⱽ ]
        (after merging the back TL segment via short-circuit BC)
```

Variables:
- `Y_src` from Eq. 3.12
- `Y_ap` from Eq. 3.13
- `Y₁₁ᴵᴵᴵ`, `Y₂₂ᴵᴵᴵ` etc. from Eqs. 3.14/3.15 applied to TL segment III (length `p`)
- Last entry includes Y₁₁ᴵⱽ from segment IV (length `d−p`) absorbed
  via the back-wall short-circuit boundary condition.

**Physical interpretation:** This is the full branch-admittance matrix
for one section. Block-diagonal in nature: source on its own, aperture
on its own, TL segments coupled as 2×2 sub-blocks. The MNA system
`A·Y·Aᵀ·U = −A·Y·V` then collapses this into the 2×2 node-admittance
system whose solution gives `U₂` directly.

Implemented in: implicit in `MNASolver` per-branch stamping; never
constructed as a single dense matrix.

---

### 7.4 — Dielectric model

#### Eq. 3.19 — Dielectric-loaded propagation constant

```
k_g = (2π·√ε_eff / λ) · √(1 − (λ / 2a·√ε_eff)²)
```

Variables:
- `ε_eff` — effective relative permittivity of the dielectric-loaded
  cross-section (dimensionless, ≥ 1)
- `λ` — free-space wavelength (m)
- `a` — broad-wall dimension (m)
- `2a·√ε_eff` — cut-off wavelength in the loaded waveguide (longer
  than the empty case because the cut-off frequency drops)

**Physical interpretation:** Filling part of the waveguide cross-section
with dielectric raises the average permittivity, slows the wave, and
LOWERS the cut-off frequency by factor `1/√ε_eff`. This shifts the
empty-waveguide resonance peaks down in frequency and changes their
spacing. Used when the enclosure floor is covered with a PCB substrate
(typical case: εr = 4.4 for FR-4, h = 1 mm, in an enclosure of height
b = 4 mm).

Implemented in: `TL_DielectricCavity.h::propagationConstant(double f)`.

---

#### Eq. 3.20 — Maxwell-Garnett effective permittivity

```
ε_eff  =  ( 2h·(ε_r − 1) + (ε_r + 2) ) / ( (ε_r + 2) − h·(ε_r − 1) )
```

Variables:
- `ε_r` — bulk relative permittivity of the dielectric filler
- `h` — fractional volume occupied by the dielectric within the
  waveguide cross-section (`0 ≤ h ≤ 1`)

The Maxwell-Garnett formula treats the structure as a host material
(air, ε=1) sparsely doped with dielectric inclusions. Most accurate
when the inclusion volume fraction is small.

Implemented in: `TL_DielectricCavity.h::effPermittivityMaxwellGarnett()`
(optional, NOT the default).

---

#### Eq. 3.21 — Lichtenecker logarithmic mixing rule (DEFAULT)

```
log₁₀(ε_eff)  =  (h/b) · log₁₀(ε_r)  +  ((b−h)/b) · log₁₀(1)
              =  (h/b) · log₁₀(ε_r)
```

Variables:
- `h` — thickness of the dielectric layer (m)
- `b` — total height of the waveguide cross-section (m)
- `ε_r` — relative permittivity of the dielectric

**Physical interpretation:** The Lichtenecker rule treats the cross-
section as a series of horizontal layers and takes the GEOMETRIC mean
of the layer permittivities, weighted by layer thickness. For a single
dielectric layer of height `h` plus an air gap of height `b−h`, it
reduces to `ε_eff = ε_r^(h/b)`. This is the DEFAULT formula used by
EMShieldDesigner because it matches PCB-loaded enclosures better than
Maxwell-Garnett at moderate fill ratios. The second term `log(1) = 0`
explicitly shows that the air portion contributes nothing.

Implemented in: `TL_DielectricCavity.h::effPermittivityLichtenecker()`
(default).

---

### 7.5 — Aperture with cover (removable lid)

#### Eq. 3.22 — Covered aperture impedance (air gap)

```
Z_ap = j·Z_0s · F_s · tan( k₀ · (l/2 + w/2 − 4τ) )
```

Variables:
- `Z_0s` — characteristic impedance of the equivalent coplanar
  strip transmission line of conductor width τ (Ω)
- `F_s` — correction factor from Eq. 3.23
- `k₀ = 2π/λ` — free-space wavenumber (rad/m)
- `l` — aperture length (m)
- `w` — aperture width (m)
- `τ` — gap thickness between the cover and the front wall (m)

**Physical interpretation:** When the front wall has a removable lid,
the seam between lid and main enclosure forms a slot of length
`l + w − 4τ` (the perimeter minus four corner gaps). This slot
behaves as a quarter-wave coplanar strip resonator. The model captures
the strong frequency dependence of the cover gap — at the resonance
frequency, `Z_ap → ∞` and the cover acts as a perfect shield; off
resonance, the gap leaks fields. The model is valid ONLY for an air
gap between lid and wall (no gasket/sealant).

Implemented in: `AP_SlotWithCover.h::stamp(double f)` when
`epsilon_r_gap == 1.0`.

---

#### Eq. 3.23 — Cover gap correction factor

```
F_s  =  √( 2τ·(l + w + 2τ) / (l·w) )
```

Variables:
- `τ`, `l`, `w` — same as Eq. 3.22.

Geometric correction relating the gap perimeter, gap thickness, and
aperture area. Approaches 1 for thick gaps and large apertures,
shrinks for thin gaps.

Implemented in: `AP_SlotWithCover.h::correctionFactor()`.

---

#### Eq. 3.24 — Covered aperture impedance (dielectric gap)

```
Z_ap = j · C_s · 1/(c·√(C'·C)) · tan( k₀ · √(C/C') · (l/2 + w/2 − 4τ) )
```

Variables:
- `c = 2.998 × 10⁸ m/s` — speed of light in vacuum
- `C` — capacitance per unit length of the equivalent coplanar
  strip line WITH dielectric in the gap (F/m)
- `C'` — capacitance per unit length WITHOUT dielectric (air gap
  only) (F/m)
- `C_s` — gap-corrected capacitance factor
- Other variables as in Eq. 3.22.

**Physical interpretation:** Generalisation of Eq. 3.22 to handle a
gasket/sealant/spacer with relative permittivity `ε_r > 1` filling
the gap. Increases the effective electrical length of the cover seam
by factor `√(C/C') ≈ √ε_r_gap`, shifting the cover's resonance to
LOWER frequency than the air-gap case. Use this when modelling
realistic enclosures with rubber gaskets or conductive elastomers.

Implemented in: `AP_SlotWithCover.h::stamp(double f)` when
`epsilon_r_gap > 1.0`.

---

### 7.6 — Multi-section models

#### Eq. 3.25 — 2-section cascade Y matrix (CASCADE topology, Fig 3.10b)

```
        [ Y_src   0        0          0          0          0       ]
        [   0   Y_ap¹      0          0          0          0       ]
        [   0     0      Y₁₁ᴵᴵᴵ     Y₁₂ᴵᴵᴵ      0          0       ]
Y    =  [   0     0      Y₂₁ᴵᴵᴵ   Y₂₂ᴵᴵᴵ+Y₁₁ᴵⱽ Y₁₂ᴵⱽ       0       ]
        [   0     0        0       Y₂₁ᴵⱽ      Y₂₂ᴵⱽ+Y_ap² Y₁₂ⱽ     ]
        [   0     0        0          0        Y₂₁ⱽ     Y₂₂ⱽ+Y₁₁ⱽᴵ ]
```

Variables:
- Block on the top-left: source + aperture #1 (same as single-section).
- Roman numerals III, IV — TL segments of section 1 (front: length `p₁`,
  back: length `d₁−p₁`).
- `Y_ap²` — aperture #2 admittance (separator between sections 1 and 2),
  computed from Eq. 3.13 with its own `l, w` dimensions.
- Roman numerals V, VI — TL segments of section 2 (front: length `p₂`,
  back: length `d₂−p₂`).

**Physical interpretation:** Two sections stacked depth-wise. The front
section's back-wall TL segment joins the inter-section aperture, then
the second section's front TL segment continues from there. Observation
points P₁ (between segments III and IV) and P₂ (between V and VI) are
both available in one matrix solve. For N cascaded sections, the
matrix has `3N+1` branches and `2N` independent nodes; structure
generalises by adding more (III, IV)-like pairs.

Implemented in: `CircuitGenerator.h::generateCascade(EnclosureConfig&)`.

---

#### Eq. 3.26 — 3-section star-branch Y matrix (STAR_BRANCH topology, Fig 3.11b)

```
        [ Y_src   0       0           0          0          0          0          0          0       ]
        [   0   Y_ap¹     0           0          0          0          0          0          0       ]
        [   0     0     Y₁₁ᴵᴵᴵ      Y₁₂ᴵᴵᴵ      0          0          0          0          0       ]
        [   0     0     Y₂₁ᴵᴵᴵ    Y₂₂ᴵᴵᴵ+Y₁₁ᴵⱽ Y₁₂ᴵⱽ       0          0          0          0       ]
Y    =  [   0     0       0       Y₂₁ᴵⱽ      Y₂₂ᴵⱽ+Y_ap² Y₁₂ⱽ        0          0          0       ]
        [   0     0       0           0       Y₂₁ⱽ      Y₂₂ⱽ+Y₁₁ⱽᴵ   0          0          0       ]
        [   0     0       0           0          0          0       Y₁₁ⱽᴵᴵ    Y₁₂ⱽᴵᴵ    Y_ap³     ]
        [   0     0       0           0          0          0       Y₂₁ⱽᴵᴵ   Y₂₂ⱽᴵᴵ+Y₁₁ⱽᴵᴵᴵ Y₁₂ⱽᴵᴵᴵ ]
        [   0     0       0           0          0          0          0       Y₂₁ⱽᴵᴵᴵ   Y₂₂ⱽᴵᴵᴵ  ]
```

Variables:
- III, IV, V, VI — TL segments of sections 1 and 2 (same structure as Eq. 3.25).
- VII, VIII — TL segments of section 3, hanging as a SIDE BRANCH off
  the inter-section junction node (NOT cascaded after section 2).
- `Y_ap³` — aperture between section 1's junction node and section 3.
- Block structure: section 3 occupies the bottom-right 3×3 quadrant,
  separated from sections 1+2 by the absence of any direct coupling
  between row 6 and row 7.

**Physical interpretation:** Section 1 is the trunk; sections 2 and 3
are side branches connecting via separate apertures to the same trunk
node. Models physical configurations like a main shielded compartment
with two smaller compartments branching off its rear wall (Fig 3.11a).
Three independent observation points (P₁, P₂, P₃), all solved
simultaneously. Branch count: 10 (3 TL pairs + 3 apertures + source +
3 back-wall opens... 3×3 + 1 = 10 indeed for N=3 star). Node count: 6.

Implemented in: `CircuitGenerator.h::generateStarBranch(EnclosureConfig&)`.

---

### 7.7 — Summary: which file implements what

This table is the reverse index of equations → implementation files,
already shown in Section 6's equation map. Reproduced here for quick
lookup:

| Eq | File | Function |
|---|---|---|
| 3.8 | `MNASolver.cpp` | `computeSE()` |
| 3.9 | `MNASolver.cpp` | `solve(f)` |
| 3.10 | `MNASolver.cpp` | `assembleA()` |
| 3.11 | `MNASolver.cpp` | `assembleV()` |
| 3.12 | `SRC_VoltageSource.h` | `stamp()` |
| 3.13 | `AP_SlotAperture.h` | `stamp(f)` |
| 3.14–3.15 | `TL_EmptyCavity.h` | `stamp(f)` (port matrix) |
| 3.16 | `TL_EmptyCavity.h` | `propagationConstant(f)` |
| 3.17 | `TL_EmptyCavity.h` | `characteristicImpedance(f)` |
| 3.18 | (implicit in MNA stamping) | — |
| 3.19 | `TL_DielectricCavity.h` | `propagationConstant(f)` |
| 3.20 | `TL_DielectricCavity.h` | `effPermittivityMaxwellGarnett()` |
| 3.21 | `TL_DielectricCavity.h` | `effPermittivityLichtenecker()` (default) |
| 3.22 | `AP_SlotWithCover.h` | `stamp(f)` air gap path |
| 3.23 | `AP_SlotWithCover.h` | `correctionFactor()` |
| 3.24 | `AP_SlotWithCover.h` | `stamp(f)` dielectric gap path |
| 3.25 | `CircuitGenerator.h` | `generateCascade(cfg)` |
| 3.26 | `CircuitGenerator.h` | `generateStarBranch(cfg)` |

---

## 8. Engineering Rules

These rules govern every code change. They are non-negotiable.

### ALWAYS

- **Work on ONE task at a time.** Wait for explicit user confirmation
  ("TASK X.Y — COMPLETE" or similar) before moving to the next.
- **Write production-quality Qt 6 C++ code.** No placeholders, no `TODO`
  comments, no stub functions.
- **Remove all AI-generated aesthetic traces:** no flat grey boxes, no
  generic icons, no default Qt widget styling left unstyled, no gradients,
  no drop shadows on element widgets, no underlines beneath titles.
- **Use Qt 6 best practices:** QSS stylesheets, custom QWidget subclasses,
  QPainter for custom drawing.
- **For every UI change, provide the complete modified file.** Never partial
  diffs unless the file exceeds 500 lines, in which case provide the exact
  changed blocks with surrounding context (anchor strings the user can
  Ctrl+F to locate the patch site).
- **Always validate balanced braces/parens before delivering code.** Run a
  token-aware delimiter check on every produced file.
- **After each task is confirmed working, explicitly state:**
  "TASK COMPLETE — ready to move to next task."
- **Centralise shared styling in `Styles.h`** under namespace `EMStyle` —
  single source of truth for all QSS helpers.
- **No `[=]` lambda captures** (C++20 hygiene). Use explicit `[this]`,
  `[this, fn]`, or named captures.

### NEVER

- Mix tasks from different windows.
- Add features not requested in the current task.
- Leave any widget without a QSS style rule.
- Use emoji or decorative characters in UI labels.

### WORKFLOW PROTOCOL — for every non-trivial task

1. **Plan first:** describe scope, file list, design decisions, what stays
   as-is. No code yet.
2. **Ask up to 3 targeted questions** (multiple-choice format) when
   decisions are ambiguous. Do not fabricate user intent.
3. **Wait for user approval** before writing code.
4. **Write the code;** validate brace/paren balance and hygiene before
   delivering.
5. **Deliver** with a clear summary, how-to-apply steps, and a verification
   checklist.
6. **Wait for user confirmation** before declaring the task complete.

### Communication style

- Be direct. No filler praise like "Great question!" or "Excellent!".
- Use tables and code blocks where they improve clarity over prose.
- Quote real file names, real line numbers, real function signatures —
  never invented ones.
- When uncertain, say so and ask. Don't guess on parameter values, paths,
  or conventions.

---

## 9. Design Tokens

The single source of truth for visual design lives in `Styles.h` under
namespace `EMStyle`. Helpers are accessed as `EMStyle::spinSS(color)`,
`EMStyle::lblSS()`, `EMStyle::brandStripQSS()`, etc.

### CBStyle palette (defined in `CircuitBuilderWindow.h`)

| Token | Hex | Use |
|---|---|---|
| `BG` | `#F6FBF9` | primary background |
| `SURFACE` | `#EBF4F0` | panel background |
| `SURFACE2` | `#DCEBE5` | nested panel background |
| `BORDER` | `#B4CDC3` | strong border |
| `BORDER_LT` | `#D2E4DE` | subtle border |
| `ACCENT` | `#0E64C8` | blue — aperture, primary action |
| `GREEN` | `#168240` | cavity, success, valid |
| `ORANGE` | `#B45A00` | source |
| `RED` | `#C31E1E` | obs.pt, error, invalid |
| `TEXT` | `#182420` | primary text |
| `TEXT_MUTED` | `#5A7369` | secondary text |
| `TEXT_DIM` | `#96AFA5` | tertiary text, hints |

### Element-type accents (Circuit Builder)

| Element | Accent |
|---|---|
| Source | ORANGE |
| Aperture / AP+Cover | ACCENT (blue) |
| Cavity / Diel.Cavity | GREEN |
| Obs.Pt | RED |

### Brand strips

- Window 1 (Quick Simulation): `"EMShieldQuickSim"`
- Window 2 (Circuit Builder): `"EMShieldBuilder"`

Both rendered via `EMStyle::brandStripText("QuickSim")` /
`EMStyle::brandStripText("Builder")` (parameterised suffix).

---

## 10. Project State

### Completed work

**Window 1 (Quick Simulation) — fully complete**
- Core MainWindow redesign (no QToolBar, left-panel button stack)
- Brand strip "EMShieldQuickSim" via `EMStyle::brandStripText("QuickSim")`
- Validity indicator (red/green dot above COMPUTE)
- MessageDialog integration replacing all QMessageBox
- Plot styling aligned with Window 2 (CBStyle palette)
- Three presets (single section, identical 2-section, identical 3-section, etc.)
- Preset reset on any user edit (combobox shows "Custom")
- Live validity reevaluation on parameter change
- Status bar showing N sections, branches, nodes, obs points, elapsed ms

**Window 2 (Circuit Builder) — complete with [T3.2-TEMPORARY] validator**
- Custom app pictogram, left-panel button layout (no QToolBar)
- Schematic element styling (no gradients, no glows, two-tone strokes)
- Stack layer info panel (right side, mirrors canvas elements)
- Mouse wheel zoom 50–300% + Ctrl+0 reset
- Topology validation (red/green dot + tooltip + full dialog)
- MessageDialog replaces all QMessageBox (no drop shadow — caused dual-effect crash)
- Drag-and-drop element placement, Arrange auto-layout
- COMPUTE wires MNA, runs frequency sweep, plots curves
- Export CSV with success/error dialogs

**Engine (include/core/) — fully validated**
- MNASolver: assembleA, assembleY, solve, computeSE
- All six branch types implemented and validated against dissertation Eqs.
- CircuitGenerator: CASCADE + STAR_BRANCH with verbose-print option
- EnclosureConfig + SectionConfig with `isValid(error_msg)` validation
- Matches FEM reference to within mean deviation 3.16–4.98 dB on four test
  structures (better than the dissertation's own 4.5–7.4 dB published values).

**Defence (21 May 2026) — completed successfully**

### In-flight items (currently in repo)

**[T3.2-TEMPORARY] — Circuit Builder validator relaxation**
- `validateCircuitCore()` in `CircuitBuilderWindow.h` drops three strict
  rules (nAp == nCav, ObsNotLast, PatternViolation) so users can build the
  dissertation Fig 3.10b topology by manually splitting cavities.
- All markers tagged `[T3.2-TEMPORARY]` for clean revert.
- ValidationCode enum retains the three dropped codes (just unused) so
  Task 3.3 (Option α) can re-enable them without re-editing the enum.
- This is a STOPGAP. Task 3.3 (P2 in the planned list below) is the proper
  fix and will revert these markers.

### Validation reference numbers

For internal sanity checking — these are the deviation values from the
post-defence presentation slides:

| Structure | Geometry | Mean dB | MAD dB | RSD % | Dissertation ref |
|---|---|---|---|---|---|
| 1 | 1-sect 10×4×10 mm | 3.16 | 2.28 | 163.4 | 7.4 dB (Fig 3.12a) |
| 2 | 2-sect 15+10 wide | 3.92 | 3.16 | 143.2 | 6.0 dB (Fig 3.19b) |
| 3 | 2-sect 10+5 narrow | 4.98 | 2.55 | 97.0 | 4.5 dB (Fig 3.19a) |
| 4 | 3-sect star-branch | 4.57 | 3.30 | 119.1 | 5.6 dB (Fig 3.21) |

---

## 11. Planned Tasks

Priority order: P1 → P2 → P3 → P4. Each must be planned before any code.

### P1 — Unit migration: Circuit Builder m → mm

**Status:** PLANNED, not yet started.

**Goal:** Circuit Builder uses metres for all dimensions and GHz for frequency.
Quick Simulation uses millimetres and MHz. This inconsistency confuses users
who switch between windows.

**Scope:**
- Change every `QDoubleSpinBox` in `BuilderPropertyPanel::showElement()` to
  millimetre ranges/steps/decimals. Affects ~12 spinboxes across all six
  element types.
- Update every default value in `struct ElementParams` (e.g.
  `double a{0.050}` becomes `double a_mm{50.0}`, or keep `a` in metres
  internally but display in mm — design decision required).
- Update property panel placeholder text in `BuilderPropertyPanel`
  constructor (the "CORRECT CIRCUIT" hint text mentions `0.150 m`).
- Update validity-indicator messages where they reference units.
- Engine bridge: at `runCompute()` time, multiply the displayed-mm values by
  `0.001` before passing to `MNASolver::stamp()` which expects SI metres.
- Frequency: change source defaults from GHz to MHz across panel, and
  multiply by `1e6` at runCompute time.

**Out of scope:**
- Quick Simulation window stays in mm/MHz (no change).
- Engine internal state stays in SI units (m, Hz). Engine sees ONLY metres.
- Validation reference plots (already produced, no need to regenerate).

**Decision needed before implementation:**
- Keep `ElementParams::a` etc. as internal metres and convert at UI boundary,
  OR rename fields to `a_mm` and convert at engine boundary? Recommendation:
  rename to `a_mm` for clarity, matches `SectionItemData::depth_mm` pattern
  already used in Window 1.

### P2 — Option α: Cavity internal observation offset

**Status:** PLANNED, not yet started. Requires P1 OR independent — pick.

**Goal:** Restore strict alternation rules in Circuit Builder by giving each
Cavity element an internal `obs_offset` field that splits the cavity into
two TL halves at MNA assembly time. Users build the natural pattern:

```
Source → Aperture → Cavity → Aperture → Cavity → Obs.Pt
```

Where each Cavity carries its own observation offset. The engine internally
splits each cavity into `TL_p` and `TL_{d-p}` during `runCompute()`.

**Scope:**
- Add `double obs_offset_mm` field to `ElementParams` (or `obs_offset`
  in metres if P1 not yet done).
- Add a "Has internal observation" checkbox + offset spinbox to the Cavity
  property panel.
- Modify `runCompute()` to detect when a Cavity has obs_offset > 0 and
  split it into two TL branches with an Obs.Pt-equivalent node between them.
- Revert the [T3.2-TEMPORARY] validator relaxation in the same commit.
  Restore `nAp == nCav`, `ObsNotLast`, `PatternViolation` checks.
- Remove the manual-split workaround instructions from any user-facing text.

**Out of scope:**
- Quick Simulation already supports this via `SectionConfig::obs_position` —
  no changes needed there.

### P3 — Internationalisation: RU/EN switching

**Status:** PLANNED, not yet started.

**Goal:** Application supports Russian and English. Language picker on
StartupWindow. Persistence via `QSettings`. Runtime switching (no restart).

**Scope:**
- Wrap every user-facing string in `tr(...)`. Most are in property panels,
  status bars, error messages, brand strips.
- Generate `.ts` files via `lupdate`.
- Translate `.ts` files manually (Russian).
- Compile to `.qm` via `lrelease`.
- Load appropriate `.qm` at startup via `QTranslator::load()` based on
  saved QSettings.
- Add language picker (Combobox or radio buttons) on StartupWindow.
- On language change, emit a signal that triggers `QApplication::installTranslator()`
  and re-translation of visible widgets (via `retranslateUi()` pattern).

**Files to modify:**
- Every `.h` and `.cpp` containing user-visible strings — many files.
- CMakeLists.txt: add `qt6_add_translations` or `qt_add_translations` call.
- New: `translations/em-shield-designer_ru.ts` and `..._en.ts`.
- StartupWindow.h: add language selector UI.
- Settings persistence: extend `QSettings` keys.

**Decisions needed:**
- Should the language picker be in the StartupWindow, or in a settings menu
  accessible from all windows? Recommendation: StartupWindow for now (one
  decision per app launch), revisit after.

### P4 — Validation completion + Release build

**Status:** PLANNED, not yet started. Lower priority post-defence.

**Goal:** Produce a deployable Release `.exe` with all dependencies bundled.

**Scope:**
- Make `add_definitions(-DDEBUG_APERTURE)` conditional: only in Debug builds.
  Use generator expression `target_compile_definitions(em-shield-designer
  PRIVATE $<$<CONFIG:Debug>:DEBUG_APERTURE>)`.
- Build Release configuration: `cmake --build build --config Release`.
- Run `windeployqt` to bundle Qt DLLs alongside the .exe.
  - Path: `C:\Qt\6.10.1\msvc2022_64\bin\windeployqt.exe`
  - Command: `windeployqt --release --no-translations build/Release/em-shield-designer.exe`
- Verify the deployed folder runs on a Windows machine WITHOUT Qt installed.
- (Optional, future) Create an installer with Inno Setup or NSIS.

**Out of scope for this task:**
- Code signing (requires certificate).
- App store distribution.

---

## 12. Known Issues and Trade-Offs

### Issue 1 — `DEBUG_APERTURE` in Release builds

`CMakeLists.txt` line `add_definitions(-DDEBUG_APERTURE)` is unconditional.
Every Release build still includes aperture debug `qDebug()` output.
Should be made `$<CONFIG:Debug>`-conditional during P4.

### Issue 2 — Two parallel build folders

`build/Desktop_Qt_6_10_1_MSVC2022_64bit-Debug/` (Qt Creator) and `build/Debug/`
(VS Code) coexist. The Qt Creator folder is ~300 MB of leftover artifacts.
User has explicitly asked NOT to delete it. Both build systems work
independently — they don't fight.

### Issue 3 — MSVC (home) vs MinGW (work) toolchain divergence

The application currently compiles cleanly on both. No runtime differences
have been observed on Qt+Eigen pure C++ code (no platform-specific code).
Should be retested if any low-level C library is added.

### Issue 4 — `external/eigen/` is vendored at version 5.0.0

Eigen 5.0.0 is the current vendored version. If Eigen 5.x major version
introduces breaking changes, the vendored copy isolates us from them.
Update is a deliberate task, not automatic.

### Issue 5 — QCustomPlot is vendored

`qcustomplot.h` and `qcustomplot.cpp` are at project root, not in a
separate directory. This makes them visible in IntelliSense's project tree.
Consider moving to `external/qcustomplot/` in a future cleanup.

### Issue 6 — `mainwindow.ui` is a Qt Designer file

Only `mainwindow` uses the .ui form-designer workflow. All other windows
are pure C++ QWidget subclasses. The .ui file should be opened in Qt
Creator if it needs visual edits — VS Code does not have a .ui editor.
Editing the generated XML by hand is possible but discouraged.

### Issue 7 — `.vscode/` is gitignored, must be recreated per machine

When syncing to the work PC, the four `.vscode/` files must be
recreated with MinGW paths. See Section 4.

---

## 13. Working with Claude Code in This Repo

### Invocation

Claude Code reads this file at session start. Verify by running:

```powershell
claude
```

The first turn should reference this file. If Claude Code asks
"what does this project do?", it has not read CLAUDE.md — close and
re-open in the correct project directory.

### Workflow protocol

Every non-trivial task follows the protocol from Section 7:

1. **Plan** — Claude proposes scope and design choices, asks clarifying
   questions if needed, lists files to touch. NO CODE YET.
2. **User approval** — user replies `approved`, or refines the plan.
3. **Code** — Claude writes the change, validates delimiter balance,
   delivers as either:
   - Full file (if ≤ 500 lines)
   - Anchor-bracketed patch blocks (if > 500 lines)
4. **User applies** — user pastes into Qt Creator / VS Code, rebuilds.
5. **User confirms** — user replies `TASK X.Y — COMPLETE` or describes
   the failure.
6. **Claude states** "TASK COMPLETE — ready to move to next task."

### What Claude Code should ALWAYS do

- Read `CLAUDE.md` first.
- Quote real file names and line numbers.
- Validate delimiter balance before delivery.
- Use explicit lambda captures (no `[=]`).
- Match the existing code style of the file being modified.
- Verify CMakeLists.txt list inclusion when adding new headers.

### What Claude Code should NEVER do

- Edit files in `external/eigen/` (vendored, treat as read-only).
- Edit files in `build/` (build output, always regenerated).
- Modify `.git/` directly.
- Add new third-party dependencies without explicit approval.
- Use `git push` or `git commit` without explicit instruction.
- Run commands that delete files outside the project root.
- Modify `.vscode/` files (per-machine; user manages those).

### Build verification commands

After any code change, Claude Code should suggest running these:

```powershell
# Configure (rare — only if CMakeLists.txt changed)
cmake -S . -B build -G "Ninja Multi-Config" -DCMAKE_PREFIX_PATH="C:/Qt/6.10.1/msvc2022_64"

# Build Debug
cmake --build build --config Debug --parallel

# Run
$env:PATH = "C:\Qt\6.10.1\msvc2022_64\bin;" + $env:PATH
.\build\Debug\em-shield-designer.exe
```

### When to stop and ask

Stop and ask the user when:
- A task touches more than three files.
- A change might invalidate previous validation work (e.g., engine
  numerical behaviour).
- The user's request is ambiguous about scope.
- A decision is needed about ABI/binary compatibility.
- A change might affect existing UI screenshots in the dissertation
  document.

### File-modification policy

- For files ≤ 500 lines: deliver the full modified file.
- For files > 500 lines: deliver anchor-bracketed patches with exact
  surrounding context that the user can Ctrl+F to locate.
- Always validate delimiter balance (curly, paren, bracket) before
  delivery.
- Mark every temporary modification with a tag like `[T3.2-TEMPORARY]`
  so it can be located and reverted via grep later.

---

## 14. Lessons Learned

These are real incidents from the project. Read them so we do not repeat
the mistakes.

### Lesson 1 — Topology bug isolation took weeks

**What happened:** Circuit Builder and Quick Simulation produced ~50–65 dB
different SE curves for the same physical structure. Initial hypothesis was
an engine bug. Many sessions were spent debugging the MNA solver.

**Root cause:** The engine was correct. Circuit Builder's UX did not allow
users to express the dissertation Fig 3.10b topology — specifically, the
"Obs.Pt as chain terminator" pattern caused the back-wall short-circuit
auto-add to be skipped, leaving a 1 GΩ Obs.Pt load as the only termination
(effectively open-circuit, not short).

**Resolution:** [T3.2-TEMPORARY] validator relaxation lets users manually
split cavities and place Obs.Pt mid-chain. Task 3.3 (Option α) will be the
permanent fix.

**Generalisable lesson:** When two implementations of the same physics
disagree, suspect TOPOLOGY before MATH. Code-traced node sequences
revealed the bug in 20 minutes after weeks of math-side debugging.

### Lesson 2 — Anton's Matlab files were red herrings

**What happened:** Supervisor's `MUP_2sek_gnu.m` and `MUP_3sek.m` were
treated as authoritative references for multi-section validation. Many
days were spent trying to match the C++ output to their plots.

**Root cause:** The filenames mislead. "2sek" in `MUP_2sek` does NOT mean
"2 sections" — it means "2 transmission-line SEGMENTS within ONE cavity".
Both files model SINGLE-section enclosures with non-canonical topology
(extra back-wall apertures and extra parallel TLs). They are not models
of the dissertation's multi-section structures.

**Resolution:** Dropped Matlab as reference. FEM data is the true ground
truth. C++ matches FEM to within mean deviation 3–5 dB, matching or
beating the dissertation's own published values.

**Generalisable lesson:** Filenames are not documentation. Read the
incidence matrix and the Y matrix structurally before trusting a
reference's labelling. Pattern-match the matrix size against the
number of physical components, not against the filename.

### Lesson 3 — `_comment` keys break c_cpp_properties.json schema

**What happened:** First version of `.vscode/c_cpp_properties.json` had
`"_comment": "..."` fields scattered throughout for documentation. The
C/C++ extension's schema validator rejected them as unknown properties,
producing warnings in the Problems panel.

**Resolution:** Use `//` line comments (JSONC) instead of fake key-value
comments. VS Code's general JSONC parser accepts them; the C/C++
extension's strict schema validator only allows known keys.

**Generalisable lesson:** Different parsers in the same project may
enforce different strictness. Stick to `//` comments for `.vscode/*.json`.

### Lesson 4 — Build folder location confusion (VS Code vs Qt Creator)

**What happened:** During VS Code setup, the `build/` folder contained both
VS Code's expected layout and Qt Creator's legacy
`Desktop_Qt_6_10_1_MSVC2022_64bit-Debug/` subfolder. It was unclear which
build was being run.

**Resolution:** Explicitly set `cmake.buildDirectory` in `.vscode/settings.json`
to `${workspaceFolder}/build` (the parent, not the Qt Creator subfolder).
Both build systems now coexist without interference.

**Generalisable lesson:** Always anchor `cmake.buildDirectory` to a precise
absolute path. Never rely on CMake Tools' auto-detection when multiple
build folders exist.

### Lesson 5 — Decimals must be set BEFORE setValue on QDoubleSpinBox

**What happened:** Property panel spinboxes silently rounded 0.005 mm to
0.01 mm, breaking user input precision. Diagnostic log showed `setValue(0.005)`
producing `value() == 0.01` AND `text() == "0.010000"`.

**Root cause:** `QDoubleSpinBox::setValue()` rounds to current `decimals()`
precision. Fresh spinbox has `decimals() == 2`. Calling `setValue(0.005)`
before `setDecimals(6)` silently rounds. Calling `setDecimals(6)` afterward
re-formats the already-lossy stored value.

**Resolution:** Always call `setDecimals(N)` BEFORE `setValue(v)`. Tagged
as `[BUGFIX-DECIMALS-ROUNDING]` in `BuilderPropertyPanel::addDouble()`.

**Generalisable lesson:** Qt widget setters can have hidden ordering
dependencies. When in doubt, set precision/range BEFORE value.

### Lesson 6 — Q_OBJECT widget destructor fires signals

**What happened:** Editing one section's parameters silently overwrote the
previously-edited section's data. Switching between sections caused stale
writes.

**Root cause:** `QFormLayout::removeRow()` triggers widget destruction.
Qt's destructor fires `valueChanged()` one final time as the widget tears
down. The lambdas connected to that signal then wrote to
`currentElement_->params`, which by then pointed at the NEXT element.

**Resolution:** `m_loading_` guard flag. Set to `true` during `showElement()`
form rebuild. Every callback lambda checks the flag and returns early if
true. Tagged as `[BUGFIX-PARAMS-RESET]` in `BuilderPropertyPanel`.

**Generalisable lesson:** Disconnect signals from widgets BEFORE deletion,
OR guard callbacks against firing during structural form rebuilds. Both
together are belt-and-braces.

---

## End

This document is maintained by the user (Salif Kante) with assistance from
Claude. Update it when:

- A planned task moves to "in progress".
- A planned task completes — move it from Section 10 to Section 9.
- A new lesson is learned that future Claude Code sessions should know.
- A build setup detail changes (Qt version upgrade, new dependency).

Last updated: post-defence, May 2026.
