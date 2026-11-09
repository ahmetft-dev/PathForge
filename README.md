# PathForge

**An open-source laboratory for racing engineering, vehicle dynamics, and motion planning.**

PathForge is an experimental, cross-platform engineering application being developed in **C++20** and **Qt 6**. Its goal is to make it easier to build tracks, explore racing lines, model vehicle motion, estimate lap times, and compare optimization methods in a reproducible environment.

Rather than becoming a racing game, PathForge is intended to be a workspace for **experimentation, visualization, and engineering analysis**. The project starts with motorsport applications and may later expand into robotics and autonomous racing.

> [!IMPORTANT]
> **Project status: pre-alpha.** Implemented so far: a Qt-free geometry library (paths, Catmull–Rom splines with an
> analytic-curvature variant, arc-length resampling, tangents/normals/curvature, constant and variable track widths,
> boundaries, global track validation with clearance metrics, circle / oval / S-curve presets), a Qt-free **editable
> track document** with derived geometry, a Qt-free I/O library (versioned JSON, **editable schema v2**, atomic saves),
> a CLI (`circle|oval|scurve`, `track validate|inspect`, JSON export), and a Qt 6 Studio that is now an **interactive 2D
> track editor** (create/select/move/insert/delete control points, per-point widths, undo/redo, background validation,
> open/save) with automated tests. Everything else described below is a development goal, not an implemented capability.
> There is no release and no vehicle, lap-time or optimisation code. See [docs/editor.md](docs/editor.md),
> [docs/geometry.md](docs/geometry.md), [docs/validation.md](docs/validation.md),
> [docs/track-format.md](docs/track-format.md) and [docs/curvature-study.md](docs/curvature-study.md) for methods,
> tolerances and limitations.

## Why PathForge?

A short line around a track is not necessarily a fast one. The fastest trajectory depends on curvature, braking, acceleration, grip, vehicle dynamics, and constraints. PathForge is designed to make those trade-offs visible and testable.

The project is guided by a few principles:

- **Correctness before presentation.** Numerical results should be checked against analytical cases and clearly documented assumptions.
- **Reproducible experiments.** Inputs, models, solver settings, and results should be recorded so experiments can be repeated.
- **Independent computation core.** Physics and optimization must work without a graphical interface.
- **Measured performance.** Runtime and memory claims should be based on reproducible benchmarks, not guesses.
- **Progressive complexity.** Start with simple, verifiable models and add fidelity only when it improves the research.

## Planned capabilities

| Area | Scope |
| --- | --- |
| **Track Studio** | 2D track editing, centerlines, boundaries, geometry validation, import/export |
| **Vehicle Lab** | Configurable vehicle parameters and progressively more detailed dynamics models |
| **Racing Laboratory** | Centerline baseline, shortest-path and minimum-curvature comparisons, later minimum-time optimization |
| **Lap-time analysis** | Curvature-aware speed limits, acceleration/braking profiles, estimated lap times |
| **Simulation Studio** | Future closed-loop vehicle simulation and trajectory-following controllers |
| **Experiment Workspace** | Parameter sweeps, telemetry overlays, repeatable comparisons, exportable results |
| **Headless tools** | Command-line experiments and benchmarks without launching Qt |
| **Robotics extensions** | Future path planning, obstacle scenarios, and autonomous racing research |

Only the geometry foundation listed in the status note above exists today; all other entries are **planned**.

## Engineering approach

PathForge will distinguish between three related problems:

1. **Path optimization:** Which geometric route should the vehicle take within the track boundaries?
2. **Speed-profile optimization:** How quickly can the vehicle travel along a given route while respecting its limits?
3. **Minimum-time trajectory optimization:** How should the route and motion profile be optimized together to reduce lap time?

These are different objectives. In particular, *shortest path*, *minimum curvature*, and *minimum lap time* are not interchangeable.

### Model fidelity

The simulation will grow in stages rather than starting with an overly complex vehicle model:

| Level | Planned model | Primary purpose |
| --- | --- | --- |
| 0 | Track geometry and kinematics | Validate paths, arc length, and curvature |
| 1 | Point-mass vehicle | Initial speed profiles and lap-time estimates |
| 2 | Kinematic bicycle | Steering and basic trajectory tracking |
| 3 | Dynamic bicycle | Lateral dynamics and yaw response |
| 4 | Extended vehicle dynamics | Aerodynamics, combined tire-force limits, and load transfer |
| 5 | Experimental research models | More advanced tire, energy, and environmental models |

**Scientific limitation:** Early lap-time estimates will rely on simplified physics and should not be interpreted as predictions of real Formula 1 vehicle performance. Higher model complexity also does not guarantee higher accuracy without suitable validation and parameters.

## Planned architecture

The simulation engine will remain independent of Qt so the same computation can run from both the desktop application and command-line tools.

```text
+------------------------------------------------------------+
|                         Applications                       |
|        PathForge Studio (Qt)  |  PathForge CLI              |
+------------------------------+-----------------------------+
                               |
+------------------------------------------------------------+
|                  Application / Experiment Layer            |
|         Scenarios  |  Runs  |  Metrics  |  Exports          |
+------------------------------------------------------------+
                               |
+------------------------------------------------------------+
|                       C++ Core Libraries                   |
|  Geometry  |  Vehicle  |  Optimization  |  Simulation       |
|  Common Types  |  Numerical Utilities  |  Data I/O         |
+------------------------------------------------------------+
```

A proposed directory structure (subject to change as development begins):

```text
PathForge/
├── apps/
│   ├── pathforge-studio/    # Qt desktop application
│   └── pathforge-cli/       # Headless experiment runner
├── libs/
│   ├── core/
│   ├── geometry/
│   ├── vehicle/
│   ├── optimization/
│   ├── simulation/
│   ├── experiments/
│   └── io/
├── extensions/
│   └── robotics/            # Later research extension
├── tests/
├── benchmarks/
├── examples/
├── datasets/
├── docs/
└── CMakeLists.txt
```

This layout is a target, **not a claim that every module already exists**.

## Technology stack

| Component | Planned choice |
| --- | --- |
| Core language | C++20 |
| Desktop interface | Qt 6 Widgets |
| Build system | CMake |
| Linear algebra | Eigen, where needed |
| Automated tests | GoogleTest / CTest |
| Data exchange | Versioned JSON (implemented, nlohmann/json) and CSV (planned) |
| CI | GitHub Actions |
| Target platforms | macOS (including Apple Silicon), Windows, Linux |

A numerical solver will be selected when its requirements are clear; one will not be added solely for architectural completeness.

## Roadmap

- [x] **Phase 0 — Foundation:** CMake project, Qt shell, test harness, CI configuration, coding conventions (CI results unverified until first run on GitHub).
- [x] **Phase 1A — Advanced track geometry:** splines, arc-length resampling, tangents/normals/curvature, boundaries, circle/oval/S-curve presets (all six GitHub Actions jobs passed on `84f9a09`).
- [x] **Phase 1B — Track validation & persistence:** global validation, clearance metrics, variable widths, JSON schema v1, file I/O, CLI and Studio integration, curvature study (all six GitHub Actions jobs passed on `b676669`: [run 42678312235](https://github.com/ahmetft-dev/PathForge/actions/runs/42678312235)). **Not yet:** interactive track editor (Phase 1C), area-overlap detection, C2 splines.
- [ ] **Phase 1C — Interactive track editor** *(implemented and tested locally on macOS and with GCC; GitHub Actions verification pending)*: editable document model, control-point editing, undo/redo, width editing, background validation with revision-tagged results, schema v2 persistence with legacy v1 import. **Not yet:** C2 centerline (Phase 2 prerequisite), area-overlap detection, multi-point selection, snapping.
- [ ] **Phase 2 — First lap simulation (`v0.1` goal):** Point-mass model, speed profile, braking constraints, estimated lap time, headless execution.
- [ ] **Phase 3 — Racing-line optimization:** Shortest path, minimum curvature, physical/geometric comparison.
- [ ] **Phase 4 — Experiments and telemetry:** Parameter sweeps, plots, export, regression and performance benchmarks.
- [ ] **Phase 5 — Closed-loop simulation:** Bicycle model, controllers, tracking error, replay.
- [ ] **Phase 6 — Advanced dynamics:** More detailed force models and minimum-time optimization.
- [ ] **Phase 7 — Robotics and autonomous racing:** Planning algorithms, obstacle scenarios, research integrations.
- [ ] **Phase 8 — Extensibility:** Stable APIs and optional Python/plugin integrations if justified.

The **first meaningful milestone** is intentionally limited: define a track and a simple vehicle, compute a physically constrained speed profile and estimated lap time, then verify the result with analytical tests. It should be possible to run the same calculation through both the desktop interface and CLI.

## Building and running

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default
./build/release/apps/pathforge-cli/pathforge-cli circle --radius 100 --points 360
./build/release/apps/pathforge-cli/pathforge-cli oval --length 300 --width 120 --points 512
./build/release/apps/pathforge-cli/pathforge-cli scurve --points 256
./build/release/apps/pathforge-cli/pathforge-cli oval --points 512 --output oval.json
./build/release/apps/pathforge-cli/pathforge-cli track validate oval.json
```

PathForge Studio is built automatically when Qt 6 is found (`./build/release/apps/pathforge-studio/pathforge-studio.app/Contents/MacOS/pathforge-studio` on macOS).
Mouse and keyboard controls are in [docs/editor.md](docs/editor.md). See [docs/building.md](docs/building.md) for options,
platform notes and verification status, and [docs/architecture.md](docs/architecture.md) for the design.

## Validation and benchmarking

Planned validation includes:

- Analytical geometry tests (for example, constant curvature on a circle).
- Known acceleration and braking-distance scenarios.
- Track-boundary and vehicle-constraint checks.
- Regression tests for numerical results within documented tolerances.
- Repeatability checks for experiments and solver settings.
- Benchmarks that record hardware, build configuration, workload, and run conditions.

Experimental results should report their **model assumptions and limitations**. A solver reporting a result is not sufficient evidence that a solution is physically valid or that optimization converged correctly.

## Open source and contributions

PathForge is a **non-commercial personal research project**: I am building it to learn, experiment, and share engineering work, not to sell a product. The intention is to publish the code openly and welcome technical contributions as the project matures.

PathForge is released under the [MIT License](LICENSE). Third-party dependencies (fetched at configure time if not installed): GoogleTest (BSD-3-Clause), nlohmann/json (MIT); the optional Studio uses Qt 6 (LGPLv3/GPL, dynamically linked). See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines. Note that the MIT license permits third-party commercial use even though this project itself has no commercial purpose.

At the planning stage, the most useful feedback concerns numerical methods, vehicle modeling assumptions, test design, and maintainable C++ architecture.

## Reference projects

Projects worth studying and comparing against include:

- [TUM — Global Racetrajectory Optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [fastest-lap](https://github.com/juanmanzanero/fastest-lap)
- [TUM Racetrack Database](https://github.com/TUMFTM/racetrack-database)
- [PacSim](https://github.com/PacSim/pacsim)

They are references for research and validation, not an indication that their code or datasets are included in PathForge. Any external data or dependencies will be reviewed for license compatibility before inclusion.

---

**PathForge is an ongoing engineering experiment.** The goal is not to ship as many features as possible, but to build a trustworthy foundation for exploring how paths, vehicle dynamics, optimization, and computation interact.
