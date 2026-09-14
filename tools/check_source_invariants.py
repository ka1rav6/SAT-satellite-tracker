#!/usr/bin/env python3
"""Static invariant checks over the C++ sources (design §2).

Several of the design's invariants are properties of the SOURCE rather than of
the running program, and CI is asked to enforce them (§5.1: "Add a CI check that
greps ... and fails if found").

A plain grep is not good enough, and the reason is worth stating because it
already bit us: this project's diagnostics talk ABOUT the forbidden constructs.
verify_repro.cpp prints

    "  * an unseeded generator or rand() (`just gate-no-rand`)"

to tell a user what to go and look for, and core/rng.hpp quotes INV-3 verbatim
in its header comment. A grep for `rand()` matches both and reports a violation
in the very code that exists to detect violations.

So this strips comments and string literals before searching. That is the whole
reason it is a script rather than three lines of shell.

Usage:  tools/check_source_invariants.py [--root DIR]
Exit:   0 if every check passes, 1 otherwise.
"""

import argparse
import pathlib
import re
import sys

# ---------------------------------------------------------------------------
# Comment and string stripping.
#
# Not a C++ parser, and it does not need to be. It handles the four things that
# actually occur in this codebase -- // comments, /* */ comments, "strings" with
# escapes, and 'chars' -- and it replaces them with spaces rather than deleting
# them so that LINE NUMBERS are preserved and an error can point at the right
# place. Raw string literals (R"(...)") are not used anywhere here; if they
# appear later, this needs extending, and the self-test below would catch a
# construct it mishandles.
# ---------------------------------------------------------------------------
def strip_comments_and_strings(text: str) -> str:
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""

        if c == "/" and nxt == "/":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
        elif c == "/" and nxt == "*":
            out.append("  ")
            i += 2
            while i < n and not (text[i] == "*" and i + 1 < n and text[i + 1] == "/"):
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            out.append("  ")
            i += 2
        elif c in ('"', "'"):
            quote = c
            out.append(" ")
            i += 1
            while i < n and text[i] != quote:
                if text[i] == "\\" and i + 1 < n:
                    out.append("  ")
                    i += 2
                    continue
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            out.append(" ")
            i += 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def strip_comments_only(text: str) -> str:
    """Comments out, string literals left intact.

    Used by checks that search for #include directives, where the path is a
    quoted string and must survive.
    """
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if c == "/" and nxt == "/":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
        elif c == "/" and nxt == "*":
            out.append("  ")
            i += 2
            while i < n and not (text[i] == "*" and i + 1 < n and text[i + 1] == "/"):
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            out.append("  ")
            i += 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def sources(root: pathlib.Path, subdirs):
    for sub in subdirs:
        d = root / sub
        if not d.is_dir():
            continue
        for ext in ("*.hpp", "*.cpp", "*.h", "*.cc"):
            yield from sorted(d.rglob(ext))


def check(root, name, rationale, subdirs, patterns, exclude=(), strip_strings=True):
    """Report every line in `subdirs` matching any of `patterns`.

    Comments are always stripped. String literals are stripped only when
    `strip_strings` is true, which is right for checks looking for CODE and
    WRONG for checks looking for #include directives -- an include path IS a
    quoted string, so stripping it blanks out the very thing being searched
    for. That mistake made the INV-1 check silently pass a real violation the
    first time this tool was tested against one.
    """
    hits = []
    rx = [re.compile(p) for p in patterns]
    for path in sources(root, subdirs):
        rel = path.relative_to(root).as_posix()
        if any(rel.startswith(e) for e in exclude):
            continue
        try:
            raw = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        stripped = (strip_comments_and_strings(raw) if strip_strings
                    else strip_comments_only(raw))
        raw_lines = raw.splitlines()
        for lineno, line in enumerate(stripped.splitlines(), start=1):
            for r in rx:
                if r.search(line):
                    original = raw_lines[lineno - 1].strip() if lineno <= len(raw_lines) else ""
                    hits.append((rel, lineno, original))
                    break
    return hits



# ---------------------------------------------------------------------------
# Third-party include boundaries.
#
# WHY THIS CHECK EXISTS, AND WHAT IT COST NOT TO HAVE IT
# ---------------------------------------------------------------------------
# cmake/modules.cmake links every third-party library PRIVATE to exactly one
# module: toml++ to sat_scenario, nlohmann/json to sat_metrics, Eigen to
# sat_tracking, OpenCV to sat_engine, ImGui to sat_gui. That is a deliberate
# boundary — "the ONLY configuration language", "OpenCV never in the hot loop"
# — and CMake enforces it for LINKING.
#
# It does not enforce it for INCLUDING, and on a developer machine nothing
# does. src/app/sweep.cpp included <nlohmann/json.hpp> while sat_app linked no
# JSON library at all. Because nlohmann-json3-dev is installed here, the header
# sits in /usr/include and every compiler found it regardless of the link
# graph: four local build configurations were green, including one that
# deliberately forced the FetchContent path, because a fetched copy does not
# remove the system one from the default include path.
#
# On CI, where that package is not installed, the header does not exist outside
# the targets that link it — and EVERY job on EVERY platform failed to build.
#
# A system-installed header masking a missing link edge is invisible to any
# amount of local building. A static check is the only cheap way to see it, and
# unlike the module graph there is no behavioural test available: the code
# compiles perfectly well on the machine where it is wrong.
#
# The table below mirrors modules.cmake by hand, which is duplication — but it
# is six lines, and it goes red the moment an include crosses the boundary,
# which is the moment it matters rather than the moment someone else builds it.
# ---------------------------------------------------------------------------
THIRD_PARTY = [
    # (human name, include regex, directories permitted to include it)
    ("toml++",         r'#\s*include\s*[<"]toml\+\+/',      ("src/scenario",)),
    ("nlohmann/json",  r'#\s*include\s*[<"]nlohmann/',        ("src/metrics",)),
    ("Eigen",          r'#\s*include\s*[<"]Eigen/',           ("src/tracking",)),
    ("OpenCV",         r'#\s*include\s*[<"]opencv2?/',        ("src/engine",)),
    ("Dear ImGui",     r'#\s*include\s*[<"](imgui|implot)',   ("src/gui", "src/third_party")),
    ("GLFW",           r'#\s*include\s*[<"]GLFW/',            ("src/gui",)),
]

ALL_MODULE_DIRS = (
    "src/core", "src/world", "src/scenario", "src/camera", "src/degrade",
    "src/perception", "src/ai", "src/tracking", "src/search", "src/plant",
    "src/control", "src/engine", "src/metrics", "src/gui", "src/app",
)


def check_third_party(root):
    """Every third-party include outside the module that links that library."""
    hits = []
    for name, pattern, allowed in THIRD_PARTY:
        rx = re.compile(pattern)
        for sub in ALL_MODULE_DIRS:
            if sub in allowed:
                continue
            for path in sources(root, (sub,)):
                rel = path.relative_to(root).as_posix()
                try:
                    raw = path.read_text(encoding="utf-8", errors="replace")
                except OSError:
                    continue
                # strip_comments_only: an include path IS a quoted string, so
                # stripping string literals would blank the thing being sought.
                stripped = strip_comments_only(raw)
                raw_lines = raw.splitlines()
                for lineno, line in enumerate(stripped.splitlines(), start=1):
                    if rx.search(line):
                        original = (raw_lines[lineno - 1].strip()
                                    if lineno <= len(raw_lines) else "")
                        hits.append((rel, lineno,
                                     f"{original}   <- {name} is linked only to "
                                     f"{', '.join(allowed)}"))
    return hits


# ---------------------------------------------------------------------------
# The checks themselves.
# ---------------------------------------------------------------------------
CHECKS = [
    dict(
        name="INV-1: no world access from the tracker side",
        rationale=(
            "perception, ai, tracking, search, control and plant must not be able to\n"
            "  read the simulator's ground truth. The linker enforces the library edge\n"
            "  and cmake/modules.cmake asserts it at configure time; this catches a\n"
            "  header-only leak that would reach neither."
        ),
        subdirs=["src/perception", "src/ai", "src/tracking",
                 "src/search", "src/control", "src/plant"],
        patterns=[r'#\s*include\s*"(world|camera|scenario)/'],
        # An include path is a quoted string: stripping string literals would
        # blank out exactly what this is looking for.
        strip_strings=False,
    ),
    dict(
        name="INV-3: no wall clock in the simulation path",
        rationale=(
            "core/profile.hpp is the single sanctioned exception -- timing never feeds\n"
            "  back into the simulation and is excluded from the fingerprint. src/app\n"
            "  and src/gui may read a clock; neither is in the simulation path."
        ),
        subdirs=["src"],
        patterns=[r'#\s*include\s*<chrono>'],
        exclude=("src/core/profile.", "src/app/", "src/gui/"),
    ),
    dict(
        name="INV-3: no rand() and no unseeded generators",
        rationale=(
            "every draw must name a Stream (core/rng.hpp). std::random_device and\n"
            "  rand() are unseeded by construction, and the std::*_distribution types\n"
            "  are not portable between standard library implementations."
        ),
        subdirs=["src"],
        patterns=[r'\brand\s*\(\s*\)', r'\bsrand\s*\(', r'\brandom_device\b'],
    ),
]


def self_test() -> bool:
    """Prove the stripper does what the checks depend on.

    A checker that silently stopped stripping would turn every check green, so
    this runs before them and is as much a part of the tool as they are.
    """
    cases = [
        ('int x = 1; // rand() here',              False, "line comment"),
        ('int x = 1; /* rand() here */',           False, "block comment"),
        ('puts("call rand() to fail");',           False, "string literal"),
        ('puts("escaped \\" rand() still inside");', False, "string with escape"),
        ("char c = '\\'';  // rand()",             False, "char literal + comment"),
        ('int x = rand();',                        True,  "real call"),
        ('  int y = 2; rand ();',                  True,  "real call, spaced"),
    ]
    rx = re.compile(r'\brand\s*\(\s*\)')
    ok = True
    for text, should_match, what in cases:
        got = bool(rx.search(strip_comments_and_strings(text)))
        if got != should_match:
            print(f"  SELF-TEST FAILED ({what}): expected "
                  f"{'a match' if should_match else 'no match'} in: {text}", file=sys.stderr)
            ok = False

    # An #include path must SURVIVE comment-only stripping, and must still be
    # hidden when it appears inside a comment. Getting this wrong made the
    # INV-1 check pass a genuine violation.
    inc = re.compile(r'#\s*include\s*"(world|camera|scenario)/')
    inc_cases = [
        ('#include "world/emitters.hpp"',        True,  "real include"),
        ('// see #include "world/emitters.hpp"', False, "include named in a comment"),
        ('/* #include "camera/splat.hpp" */',    False, "include in a block comment"),
        ('#include "core/units.hpp"',            False, "an allowed include"),
    ]
    for text, should_match, what in inc_cases:
        got = bool(inc.search(strip_comments_only(text)))
        if got != should_match:
            print(f"  SELF-TEST FAILED ({what}): expected "
                  f"{'a match' if should_match else 'no match'} in: {text}", file=sys.stderr)
            ok = False

    # Line numbers must survive stripping, or an error points at the wrong line.
    src = 'a\n/* multi\n   line\n   comment */\nrand();\n'
    stripped_lines = strip_comments_and_strings(src).splitlines()
    if len(stripped_lines) != len(src.splitlines()):
        print("  SELF-TEST FAILED: stripping changed the line count", file=sys.stderr)
        ok = False
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=pathlib.Path(__file__).resolve().parent.parent)
    args = ap.parse_args()
    root = pathlib.Path(args.root)

    print("Checking the comment/string stripper itself…")
    if not self_test():
        print("FAIL: the stripper is broken, so every check below would be "
              "meaningless.", file=sys.stderr)
        return 1
    print("  stripper ok.\n")

    failures = 0
    for c in CHECKS:
        hits = check(root, c["name"], c["rationale"], c["subdirs"],
                     c["patterns"], c.get("exclude", ()),
                     c.get("strip_strings", True))
        if hits:
            failures += 1
            print(f"VIOLATION — {c['name']}", file=sys.stderr)
            print(f"  {c['rationale']}", file=sys.stderr)
            for rel, lineno, text in hits:
                print(f"    {rel}:{lineno}: {text}", file=sys.stderr)
            print(file=sys.stderr)
        else:
            print(f"ok — {c['name']}")

    tp = check_third_party(root)
    name = "build: no third-party include outside the module that links it"
    if tp:
        failures += 1
        print(f"VIOLATION — {name}", file=sys.stderr)
        print("  cmake/modules.cmake links each third-party library PRIVATE to one\n"
              "  module. A system-installed copy of that header is on the default\n"
              "  include path here and is NOT on CI, so this builds locally and\n"
              "  fails everywhere else. Either move the code, or add the link edge\n"
              "  in modules.cmake and this table.", file=sys.stderr)
        for rel, lineno, text in tp:
            print(f"    {rel}:{lineno}: {text}", file=sys.stderr)
        print(file=sys.stderr)
    else:
        print(f"ok — {name}")

    if failures:
        print(f"\n{failures} invariant check(s) failed.", file=sys.stderr)
        return 1
    print("\nAll source invariants hold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
