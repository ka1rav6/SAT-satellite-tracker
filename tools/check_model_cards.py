#!/usr/bin/env python3
"""Check that every model card's headline numbers match its committed artifact.

WHY THIS EXISTS
---------------
SAT-ML.md's rule ML-8 requires every model to ship a model card, and the card
is the document an evaluator reads to decide whether to believe the model. The
numbers on it are therefore load-bearing in exactly the way a README figure is
not: "45.7% better than constant-velocity" is the claim the whole ML section
rests on.

Those numbers are transcribed by hand from an evaluation run, and transcribed
numbers drift. One had already drifted when this was written: the MotionNet v1
card quoted

    CV +5 2428.825, MotionNet +5 1318.304, 45.7% / 53.9%, regime 0.921

while `models/motionnet_v1.eval.json` -- the artifact
`ml/evaluate_motion.py` actually wrote, and the only evidence in the
repository -- said

    CV +5 2440.694, MotionNet +5 1380.805, 43.4% / 49.5%, regime 0.9288

The SAT-ML 6.6 gate passes on either set, so no conclusion changed. That is
what makes this class of defect dangerous rather than harmless: nothing looks
wrong, and the first person to check a card against its artifact is an
evaluator who now has a reason to doubt every other number in the submission.
The audit's P3-4 built exactly this kind of checker for docs/; this is the same
gate for docs/models/.

WHAT IT CHECKS
--------------
For every `docs/models/<name>.md` with a matching `models/<name>.eval.json`:
every number in the card's SAT-ML 6.6 GATE TABLE appears in the artifact, to
the precision the card prints it at.

Only that table. A card carries several tables from several sources -- the
end-to-end ablation rows come from `tools/motion_ablate.py`, not from the
evaluation -- and a checker that reported those as violations would be switched
off within a week. It checks the section it can speak for.

WHAT IT DOES NOT CHECK
----------------------
Prose. A card can still describe a model it no longer matches, and the
"Fails when" section is a judgement no checker can make. This covers the
mechanical half so review effort goes on the half it cannot.
"""

import json
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The eval.json keys a gate table row may be quoting, in the order a card
# prints them. A card is free to omit any of these; it is not free to print one
# that disagrees.
TRACKED = (
    "cv_plus5_rmse",
    "cv_plus15_rmse",
    "motion_plus5_rmse",
    "motion_plus15_rmse",
    "regime_acc",
    "plus5_improvement",
    "plus15_improvement",
)


# The card section whose numbers come from eval.json. A model card carries
# several tables from several artifacts -- the end-to-end ablation rows come
# from tools/motion_ablate.py, not from the evaluation -- so checking every
# table against eval.json would report the ablation as a violation and the
# checker would be turned off within a week. It checks the one section it can
# actually speak for, and says so.
GATE_HEADING = re.compile(r"^#{1,6}\s+.*\b6\.6\b.*gate", re.IGNORECASE)
ANY_HEADING = re.compile(r"^#{1,6}\s")


def card_numbers(text):
    """Every number in the card's SAT-ML 6.6 gate table, with its line number.

    Restricted to the table rows inside that one section. Prose routinely
    cites a threshold ("20% better", "0.90") that is a requirement rather than
    a measurement, and flagging those would make the checker noise.
    """
    out = []
    in_gate = False
    for lineno, line in enumerate(text.splitlines(), 1):
        if ANY_HEADING.match(line):
            in_gate = bool(GATE_HEADING.match(line))
            continue
        if not in_gate:
            continue
        s = line.strip()
        if not s.startswith("|") or set(s) <= set("|-: "):
            continue
        for m in re.finditer(r"\d+\.\d+", s):
            out.append((lineno, m.group(0)))
    return out


def matches_any(token, values):
    """Is `token` one of `values`, read at the precision the card printed it?

    A card printing 43.4 must match 0.43425737945184817 as a percentage, and a
    card printing 2440.694 must match 2440.6943359375 rounded to three places.
    Comparing at the card's own precision is the whole point: it is what the
    card claims, and it is all the card claims.
    """
    places = len(token.split(".")[1])
    want = float(token)
    for v in values:
        for scaled in (v, v * 100.0):
            if round(scaled, places) == want:
                return True
    return False


def check_card(card_path, eval_path):
    with open(card_path, encoding="utf-8") as f:
        card = f.read()
    with open(eval_path, encoding="utf-8") as f:
        table = json.load(f).get("table", {})

    values = [float(table[k]) for k in TRACKED if k in table]
    if not values:
        return [f"{eval_path}: no recognised metric keys; nothing to check against"]

    problems = []
    for lineno, token in card_numbers(card):
        if not matches_any(token, values):
            problems.append(
                f"{card_path}:{lineno}: {token} is in the gate table but not in "
                f"{os.path.relpath(eval_path, REPO)}"
            )
    return problems


def main():
    cards_dir = os.path.join(REPO, "docs", "models")
    models_dir = os.path.join(REPO, "models")
    if not os.path.isdir(cards_dir):
        print("ok — model cards: docs/models/ does not exist, nothing to check")
        return 0

    problems, checked = [], 0
    for name in sorted(os.listdir(cards_dir)):
        if not name.endswith(".md"):
            continue
        stem = name[:-3]
        eval_path = os.path.join(models_dir, stem + ".eval.json")
        if not os.path.exists(eval_path):
            # A card with no eval artifact is not a failure. Several models are
            # specified before they are trained, and their cards are correctly
            # labelled PRELIMINARY; demanding an artifact would push back
            # against writing the card first, which SAT-ML wants.
            continue
        problems += check_card(os.path.join(cards_dir, name), eval_path)
        checked += 1

    if problems:
        print("VIOLATION — a model card quotes a number its artifact does not contain")
        print("  A transcribed number that no artifact supports is the defect that")
        print("  costs the most credibility for the least benefit. Re-run the")
        print("  evaluation, or correct the card to match the committed eval.json.")
        for p in problems:
            print("    " + p)
        return 1

    print(f"ok — model cards: every gate-table figure in {checked} card(s) "
          f"matches its committed eval.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
