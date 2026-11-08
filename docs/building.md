# Building and testing

Requirements: a C++20 compiler, CMake ≥ 3.21. Optional: Qt 6 (Widgets) for PathForge Studio.
GoogleTest is used from the system if found, otherwise downloaded (pinned release v1.17.0, SHA-256 checked)
at configure time, so the first configure needs network access.

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
| `PATHFORGE_WARNINGS_AS_ERRORS` | OFF | `-Werror` / `/WX` |
