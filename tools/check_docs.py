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
import os
import re
import sys

DOCS = ("README.md", "docs/MANUAL.md", "docs/RESULTS.md",
        "docs/ARCHITECTURE.md", "docs/METRICS.md")
CROSS_REF_DOCS = ("docs/RESULTS.md", "docs/METRICS.md",
                  "docs/ARCHITECTURE.md", "docs/MANUAL.md")

# Path prefixes that must exist on disk when a document names one.
REPO_DIRS = ("src", "tests", "tools", "scenarios", "cmake")

# Headings of the form "## 3. Title" or "### 2.14 Title".
HEADING = re.compile(r"^#{2,3} (\d+(?:\.\d+)?)[.\s]", re.M)


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

    if problems:
        print("DOC REFERENCES — problems found:\n")
        for p in problems:
            print(f"  {p}")
        print(f"\n{len(problems)} problem(s).")
        return 1

    print(f"ok — docs: every recipe, path, link and cross-reference in "
          f"{len(DOCS)} documents resolves")
    return 0


if __name__ == "__main__":
    sys.exit(main())
