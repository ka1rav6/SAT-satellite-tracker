
# AGENTS.md

Guidelines for AI agents (and human contributors) working in this repository.
Read this file before making any changes and follow every rule below.

---

## 1. Project overview

- **Name:** `SAT - Satillite Adaptive Tracker`
- **Goal:** Hackathon submission demonstrating software built with extreme accuracy and use. Need to get as much score as possible based on the scoring criteria
- **Language:** C++20 (`CMAKE_CXX_STANDARD 20`).

---

## 2. Non-negotiable rules

These rules apply without fail, in every change, no exceptions:

- As optimized code as possible.
- **Everything must work.** Every change must keep the project building and
  tests passing.
- **Maximize performance and accuracy** while following the design doc.

---

## 3. Design doc & roadmap

- Always follow the **design doc** and the **roadmap* (`docs/design.md`, `docs/roadmap.mp`).
- **Ask before making a new design choice** that is not already covered by the
  design doc. Do not silently deviate.

---

## 4. Commit workflow

- **Commit a lot.** Every single logical change and feature gets its own commit.
- Write **verbose** commit messages that explain *what* and *why*.
- Do not bundle unrelated changes into one commit.
- Follow the repo's existing commit message style.

---

## 5. Code quality

- **Comment a lot.** The codebase should be understandable by beginners.
  Explain *why* a piece of code exists, not just *what* it does.
- Keep the code readable and idiomatic C++.
- Match the existing code conventions of the surrounding files.

---

## 6. Testing

- **Create unit tests for everything.**
- Test **every feature separately** so failures are isolated and diagnosable.
- Any external testing framework like ctest works. Choose the best one according to you and stick to it.
- Always run the full test suite before finishing any work.

---

## 7. CI pipeline

- Maintain a proper **CI pipeline** that verifies the build is appropriate.
- The pipeline must build and run the full test suite on every change.
- Keep CI green at all times.

---

## 8. Development environment

- Provide an easily reproducible dev environment (via **Docker** only).
- The container must be usable for building, testing, and running the project.
- Document the recommended local build/test commands here (see below).

### Build & test commands

- Create a `Justfile` that has all the important commands as needed
- `just test`, `just run`, `just clean`, `just build`
- `just docker-setup`, and any other command you deem necessary


Maintain documention as well:
- all the features
- how to use
- how to customize
- for the developers (how each thing can be run/ built/ what are the commands) etc


Use as many 3rd party libraries as possible but make sure: 
- Their license states we can use them
- Everything remains fast
