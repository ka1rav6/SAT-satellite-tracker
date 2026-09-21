#!/usr/bin/env python3
"""Check that the documentation's own references resolve.

WHY THIS EXISTS
---------------
The docs are a deliverable, not a comment. README.md, ARCHITECTURE.md,
RESULTS.md, MANUAL.md and METRICS.md are what an evaluator reads, and every one
of them names things that live in the repository: `just` recipes, source paths,
scenario files, sections of each other. Those names rot silently — renaming a
recipe or splitting a test suite breaks a document and nothing goes red.

Three real defects were found the first two times this was run by hand:

  * MANUAL.md described `scenarios/adversarial/` as holding breaking cases.
    The directory contains a `.gitkeep`.
  * METRICS.md had TWO sections numbered 2.12, one of them a stale list still
    claiming handover was disabled.
  * `just test-control` pointed at `test_loop` after the Stage 10 cases moved
    to their own suite, so it printed one test case out of twenty.

None of those is a typo; each was a statement that had stopped being true. This
checks the mechanical half of that — the half a machine can check — so the
review effort goes on the half it cannot.

WHAT IT DOES NOT CHECK
----------------------
Whether a measured number is still the number the command prints. That needs
the command run, and several take minutes. Re-measuring is a human step before
a release, and the docs name the command beside every figure precisely so that
step is mechanical rather than archaeological.
"""

import glob
import json
import os
import re
import sys

DOCS = ("README.md", "QUICKSTART.md", "docs/GUIDE.md", "docs/DEMO.md",
        "docs/MANUAL.md", "docs/RESULTS.md",
        "docs/ARCHITECTURE.md", "docs/METRICS.md", "issues_till_now.md",
        # The Technical Report is a submitted deliverable, and it is the
        # document most likely to be read by someone who will never run a
        # command in it. It is checked like every other document precisely
        # because of that: it claimed `just dist` for a recipe named
        # `just package`, and nothing caught it until it was listed here.
        "docs/report/TECHNICAL_REPORT.md")
CROSS_REF_DOCS = ("docs/RESULTS.md", "docs/METRICS.md",
                  "docs/ARCHITECTURE.md", "docs/MANUAL.md")

# Path prefixes that must exist on disk when a document names one.
REPO_DIRS = ("src", "tests", "tools", "scenarios", "cmake")

# Headings of the form "## 3. Title" or "### 2.14 Title".
HEADING = re.compile(r"^#{2,3} (\d+(?:\.\d+)?)[.\s]", re.M)

# ---------------------------------------------------------------------------
# P3-4 — numbers, not just links.
#
# This file used to check that every path, recipe and cross-reference in the
# documentation resolved, and nothing at all about whether a quoted FIGURE was
# still the figure the system produces. That is the drift that actually
# misleads a reader: a dead link is obvious the moment it is clicked, while
# "16.94 px" silently becoming wrong looks exactly like "16.94 px" being right.
#
# A document anchors a number by marking it:
#
#     Measured with jitter on: **16.94 px** <!--@ tracking.steady.rms_px 0.05 -->
#
# The marker names a dotted path into `metrics` in docs/baseline/run.json and
# an absolute tolerance in the number's own units. The value checked is the
# last number appearing BEFORE the marker on that line.
#
# WHY A MARKER AND NOT A PATTERN. The first design scanned prose for known
# metric names and pulled the nearby number out. That is unmaintainable in both
# directions: it silently stops checking when someone rewords a sentence, and
# it matches the wrong number the moment a line mentions two. An explicit
# marker says precisely which figure is anchored to precisely which
# measurement, it is invisible in rendered Markdown, and a reader editing the
# sentence around it can see that the number is under test.
#
# Refresh the baseline with `just baseline`, and read the diff.
BASELINE = "docs/baseline/run.json"
#
# A trailing `%` means the document quotes a PERCENTAGE of a value run.json
# stores as a fraction — `target_loss_post_acq` is 0.0, and the table says
# "0.00 %". Without it every percentage in the documentation would have to be
# left unanchored, which is three of the graded rows.
NUMBER_MARK = re.compile(
    r"([-+]?\d[\d,]*(?:\.\d+)?)"     # the number being anchored
    # Units, bold markers and closing prose — but NO DIGITS, so the number
    # captured is the one NEAREST the marker. Without that exclusion the lazy
    # match ran to the leftmost number on the line and a table row read its own
    # row index as the measurement: "documents 16 for acquisition.in_fov_s".
    r"(?:[^<\n\d]*?)"
    r"<!--@\s*([A-Za-z0-9_.]+)\s+([\d.]+)\s*(%?)\s*-->")


def baseline_lookup(metrics, path):
    """Walk a dotted path into the metrics object. Returns None if absent."""
    node = metrics
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node if isinstance(node, (int, float)) else None


def check_numbers(problems):
    """Every marked number must still match the committed baseline."""
    if not os.path.exists(BASELINE):
        problems.append(f"{BASELINE}: missing; run `just baseline`")
        return 0

    try:
        metrics = json.load(open(BASELINE, encoding="utf-8"))["metrics"]
    except (ValueError, KeyError) as exc:
        problems.append(f"{BASELINE}: not a readable run.json ({exc})")
        return 0

    checked = 0
    for f in DOCS:
        if not os.path.exists(f):
            continue
        for ln, line in enumerate(open(f, encoding="utf-8"), 1):
            for quoted, path, tol, pct in NUMBER_MARK.findall(line):
                checked += 1
                actual = baseline_lookup(metrics, path)
                if actual is None:
                    problems.append(
                        f"{f}:{ln}: no metric '{path}' in {BASELINE}")
                    continue
                if pct:
                    actual *= 100.0
                want = float(quoted.replace(",", ""))
                if abs(want - actual) > float(tol):
                    problems.append(
                        f"{f}:{ln}: documents {want:g} for {path}, but the "
                        f"baseline says {actual:.6g} "
                        f"(tolerance {tol}) — re-measure, or `just baseline` "
                        f"if the change was intended")
    return checked


def code_spans(text):
    """Every inline `code` span and fenced block, concatenated.

    Recipe names are only checked inside code, because prose legitimately
    contains the word "just" — "not just the central figure" was reported as a
    missing recipe called `the` the first time this ran.
    """
    spans = re.findall(r"```.*?```", text, re.S)
    spans += re.findall(r"`[^`\n]+`", text)
    return "\n".join(spans)


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)

    if not os.path.exists("Justfile"):
        print("check_docs: no Justfile; run from the repository root")
        return 2

    recipes = set(re.findall(r"^([a-z0-9-]+)(?:\s+[^:]*)?:",
                             open("Justfile", encoding="utf-8").read(), re.M))

    headings = {}
    for f in CROSS_REF_DOCS:
        headings[os.path.basename(f)] = set(
            HEADING.findall(open(f, encoding="utf-8").read()))

    problems = []
    for f in DOCS:
        if not os.path.exists(f):
            problems.append(f"{f}: does not exist")
            continue
        text = open(f, encoding="utf-8").read()
        code = code_spans(text)
        base = os.path.dirname(f) or "."

        # 1. `just <recipe>` must name a recipe that exists.
        for name in sorted(set(re.findall(r"just ([a-z0-9-]+)", code))):
            if name not in recipes and name != "--list":
                problems.append(f"{f}: `just {name}` is not a recipe")

        # 2. Repository paths named in code must exist.
        prefix = "|".join(REPO_DIRS)
        for p in sorted(set(re.findall(
                rf"`((?:{prefix})/[A-Za-z0-9_./*-]+)`", text))):
            if "*" in p:
                if not glob.glob(p):
                    problems.append(f"{f}: nothing matches {p}")
            elif not os.path.exists(p):
                problems.append(f"{f}: {p} does not exist")

        # 3. Relative markdown links must resolve.
        for target in sorted(set(re.findall(r"\]\(([^)#]+)(?:#[^)]*)?\)", text))):
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            if not os.path.exists(os.path.join(base, target)):
                problems.append(f"{f}: dead link -> {target}")

        # 4. "RESULTS.md §9" must name a section that exists.
        for doc, sec in re.findall(
                r"`?(RESULTS\.md|METRICS\.md|ARCHITECTURE\.md|MANUAL\.md)`? §([\d.]+)",
                text):
            if sec not in headings[doc]:
                problems.append(f"{f}: refers to {doc} §{sec}, which has no such section")

        # 5. No document may number two sections the same. This is what found
        #    the two 2.12s, one of which was a stale "not yet implemented" list.
        nums = HEADING.findall(text)
        for dup in sorted({n for n in nums if nums.count(n) > 1}):
            problems.append(f"{f}: section {dup} appears more than once")

    numbers_checked = check_numbers(problems)

    if problems:
        print("DOC REFERENCES — problems found:\n")
        for p in problems:
            print(f"  {p}")
        print(f"\n{len(problems)} problem(s).")
        return 1

    print(f"ok — docs: every recipe, path, link and cross-reference in "
          f"{len(DOCS)} documents resolves, and {numbers_checked} marked "
          f"figure(s) still match {BASELINE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
