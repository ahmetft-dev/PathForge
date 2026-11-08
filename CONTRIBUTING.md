# Contributing to PathForge

Thanks for your interest. PathForge is a small research project; the bar is **numerical correctness first**.

## Ground rules

- The computational libraries under `libs/` must never depend on Qt. Qt lives only in `apps/pathforge-studio`.
- Use SI units (meters, radians, seconds) and document coordinate frames.
- New numerical code needs tests with analytical references and a short comment justifying each tolerance.
  Do not loosen a tolerance just to make a test pass.
- Distinguish discretisation error from rounding error in tests and docs.
- Don't add dependencies or modules "for later".
- Never claim a feature or platform as working unless it was actually run.

## Workflow

1. Build and test locally (see [docs/building.md](docs/building.md)). Use the `debug` preset: warnings are errors.
2. Format with `clang-format` (config in `.clang-format`).
3. Use [Conventional Commits](https://www.conventionalcommits.org/) (`feat:`, `fix:`, `test:`, `docs:`, `build:`, `ci:`).
4. One meaningful, verified change per commit/PR.
