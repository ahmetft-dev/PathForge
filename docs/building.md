# Building and testing

Requirements: a C++20 compiler, CMake ≥ 3.21. Optional: Qt 6 (Widgets) for PathForge Studio.
GoogleTest (v1.17.0) and nlohmann/json (v3.11.3, MIT) are used from the system if found, otherwise downloaded at
configure time as pinned, SHA-256-checked tarballs, so the first configure needs network access.

## Quick start

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default
```

Presets: `default` (Release), `debug` (Debug, warnings as errors), `headless` (no Studio).
If Qt is not found, Studio is skipped automatically (`-DPATHFORGE_BUILD_STUDIO=ON` makes it mandatory).
On macOS with Homebrew Qt you may need `-DCMAKE_PREFIX_PATH=$(brew --prefix qt)`.

## Running

```bash
./build/release/apps/pathforge-cli/pathforge-cli circle --radius 100 --points 360
./build/release/apps/pathforge-cli/pathforge-cli oval --length 300 --width 120 --points 512 --track-width 10
./build/release/apps/pathforge-cli/pathforge-cli scurve --length 300 --amplitude 30 --points 256
./build/release/apps/pathforge-studio/pathforge-studio.app/Contents/MacOS/pathforge-studio   # macOS
./build/release/apps/pathforge-studio/pathforge-studio                                       # Linux/Windows
```

`pathforge-studio --screenshot out.png` renders the window to a PNG and exits (useful as a smoke test;
set `QT_QPA_PLATFORM=offscreen` on headless machines).

## CMake options

| Option | Default | Meaning |
| --- | --- | --- |
| `PATHFORGE_BUILD_TESTS` | ON | Build unit tests |
| `PATHFORGE_BUILD_CLI` | ON | Build `pathforge-cli` |
| `PATHFORGE_BUILD_STUDIO` | AUTO | `AUTO` / `ON` / `OFF` |
| `PATHFORGE_BUILD_BENCHMARKS` | OFF | `validation_bench` and `curvature_study` (not run by CTest) |
| `PATHFORGE_WARNINGS_AS_ERRORS` | OFF | `-Werror` / `/WX` |

## Track files and validation from the command line

```bash
./build/release/apps/pathforge-cli/pathforge-cli oval --points 512 --output oval.json --name "My Oval"   # export
./build/release/apps/pathforge-cli/pathforge-cli track validate oval.json
./build/release/apps/pathforge-cli/pathforge-cli track inspect oval.json
```

Exit codes: `0` success / valid, `1` invalid or not fully checked (`track validate`), `2` usage error, `3` file could
not be read, parsed or written. `--output` refuses to overwrite unless `--force` is given. `track inspect` exits 0
whenever the file could be read (it reports the validation status instead).

## Benchmarks

```bash
cmake -S . -B build/bench -DCMAKE_BUILD_TYPE=Release -DPATHFORGE_BUILD_BENCHMARKS=ON -DPATHFORGE_BUILD_STUDIO=OFF
cmake --build build/bench
./build/bench/benchmarks/validation_bench 7
./build/bench/benchmarks/curvature_study
```
