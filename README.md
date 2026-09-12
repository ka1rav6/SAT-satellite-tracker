# SAT — Satellite Adaptive Tracker

Self-Adaptive beacon Tracking for coarse alignment of mobile FSOC terminals
(SIH PS 26169 · Department of Space / ISRO).

## Quick start (developers)

```bash
just setup-vcpkg   # once — clones/bootstraps vcpkg at the pinned baseline
just build         # cmake configure + build
just test          # ctest
just run           # prints the version string (CP 0.1)
just clean
```

Without `just`:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/sat-tracker
```

## Layout

See `docs/SAT-DESIGN.md` §5 for the full repository layout and CMake target graph.
Design checkpoints start at **Stage 0 / CP 0.1** (this scaffolding).

## Docs

| File | Role |
|---|---|
| `docs/SAT-DESIGN.md` | Master design + checkpoint roadmap |
| `docs/SAT-ML.md` | ML / OpenCV companion spec |
| `AGENTS.md` | Rules for contributors and coding agents |
