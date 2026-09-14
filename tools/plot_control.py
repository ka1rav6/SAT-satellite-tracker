#!/usr/bin/env python3
"""Plot one or more control traces (metrics/trace_log.hpp) as a standalone SVG.

Why a script and not C++
------------------------
The traces this reads are DIAGNOSTICS. They are produced by asking for
`--trace`, they contain truth, and nothing that is graded depends on them. A
plot of a diagnostic is itself a diagnostic, so it does not need to be inside
the shipped binary, reproducible to the bit, or covered by INV-3. metrics/
report.cpp writes the SVG that ships with a run; this writes the SVG that
argues a checkpoint.

The output is deliberately dependency-free: no matplotlib, no numpy, just a
hand-written SVG. Adding a plotting dependency to make a picture for a report
is not a trade worth making, and a text SVG diffs and reviews.

Usage
-----
    tools/plot_control.py -o out.svg \
        --column err_px --title "CP 10.1 ..." \
        label=path/to/trace.csv [label=path ...]
"""

import argparse
import math
import os
import sys

# Series colours. Chosen to stay distinguishable in greyscale, since these end
# up in a printed report: they differ in lightness as well as in hue.
COLOURS = ["#1f4e79", "#c0504d", "#4f8a3d", "#7d5ba6", "#b07d18"]

WIDTH, HEIGHT = 900, 380
PAD_L, PAD_R, PAD_T, PAD_B = 68, 22, 46, 46


def read_trace(path):
    """Return (header_lines, list_of_row_dicts). Comment lines start with '#'."""
    header, cols, rows = [], None, []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.rstrip("\n")
            if line.startswith("#"):
                header.append(line)
                continue
            if cols is None:
                cols = line.split(",")
                continue
            parts = line.split(",")
            if len(parts) != len(cols):
                continue
            rows.append(dict(zip(cols, parts)))
    if cols is None:
        raise SystemExit(f"{path}: no header row")
    return header, rows


def series(rows, column):
    """(t, value) pairs, skipping blanks.

    Blanks are skipped rather than plotted as zero for the same reason
    trace_log.cpp writes them blank: a zero in an error column reads as perfect
    pointing, and a plot that draws it is lying in the most convincing possible
    way.
    """
    out = []
    for r in rows:
        v = r.get(column, "")
        if v == "":
            continue
        try:
            out.append((float(r["t"]), float(v)))
        except ValueError:
            continue
    return out


def nice_step(span, target_ticks=6):
    """A round axis step near span/target_ticks: 1, 2 or 5 times a power of 10."""
    if span <= 0:
        return 1.0
    raw = span / max(1, target_ticks)
    mag = 10.0 ** int(math.floor(math.log10(raw)))
    for m in (1.0, 2.0, 5.0, 10.0):
        if raw <= m * mag:
            return m * mag
    return 10.0 * mag


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def render(datasets, column, title, subtitle, ylabel, out_path):
    xs = [p[0] for _, pts in datasets for p in pts]
    ys = [p[1] for _, pts in datasets for p in pts]
    if not xs:
        raise SystemExit(f"nothing to plot: column '{column}' was empty in every input")

    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    if y1 - y0 < 1e-9:
        y0, y1 = y0 - 1.0, y1 + 1.0
    # A little headroom, and always include zero: an error plot whose baseline
    # is off-screen invites reading a constant offset as good pointing.
    pad = 0.08 * (y1 - y0)
    y0, y1 = y0 - pad, y1 + pad
    if y0 > 0:
        y0 = 0.0
    if y1 < 0:
        y1 = 0.0

    pw = WIDTH - PAD_L - PAD_R
    ph = HEIGHT - PAD_T - PAD_B

    def sx(v):
        return PAD_L + (v - x0) / (x1 - x0 or 1.0) * pw

    def sy(v):
        return PAD_T + ph - (v - y0) / (y1 - y0 or 1.0) * ph

    o = []
    o.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" '
             f'height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}" '
             f'font-family="DejaVu Sans, Helvetica, Arial, sans-serif">')
    o.append(f'<rect width="{WIDTH}" height="{HEIGHT}" fill="#ffffff"/>')
    o.append(f'<text x="{PAD_L}" y="22" font-size="15" font-weight="600" '
             f'fill="#1a1a1a">{esc(title)}</text>')
    if subtitle:
        o.append(f'<text x="{PAD_L}" y="38" font-size="11" fill="#666">'
                 f'{esc(subtitle)}</text>')

    # grid + axes
    ystep = nice_step(y1 - y0)
    t = math.ceil(y0 / ystep) * ystep
    while t <= y1 + 1e-9:
        yy = sy(t)
        colour = "#999" if abs(t) < 1e-9 else "#e4e4e4"
        o.append(f'<line x1="{PAD_L}" y1="{yy:.1f}" x2="{PAD_L+pw}" y2="{yy:.1f}" '
                 f'stroke="{colour}" stroke-width="1"/>')
        o.append(f'<text x="{PAD_L-8}" y="{yy+4:.1f}" font-size="10" fill="#555" '
                 f'text-anchor="end">{t:g}</text>')
        t += ystep

    xstep = nice_step(x1 - x0)
    t = math.ceil(x0 / xstep) * xstep
    while t <= x1 + 1e-9:
        xx = sx(t)
        o.append(f'<line x1="{xx:.1f}" y1="{PAD_T}" x2="{xx:.1f}" y2="{PAD_T+ph}" '
                 f'stroke="#f0f0f0" stroke-width="1"/>')
        o.append(f'<text x="{xx:.1f}" y="{PAD_T+ph+16}" font-size="10" fill="#555" '
                 f'text-anchor="middle">{t:g}</text>')
        t += xstep

    o.append(f'<rect x="{PAD_L}" y="{PAD_T}" width="{pw}" height="{ph}" '
             f'fill="none" stroke="#bbb"/>')
    o.append(f'<text x="{PAD_L+pw/2:.0f}" y="{HEIGHT-10}" font-size="11" '
             f'fill="#333" text-anchor="middle">time, s</text>')
    o.append(f'<text x="14" y="{PAD_T+ph/2:.0f}" font-size="11" fill="#333" '
             f'text-anchor="middle" transform="rotate(-90 14 {PAD_T+ph/2:.0f})">'
             f'{esc(ylabel)}</text>')

    for i, (label, pts) in enumerate(datasets):
        c = COLOURS[i % len(COLOURS)]
        d = " ".join(("M" if j == 0 else "L") + f"{sx(px):.2f},{sy(py):.2f}"
                     for j, (px, py) in enumerate(pts))
        o.append(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="1.5" '
                 f'stroke-linejoin="round"/>')
        ly = PAD_T + 14 + i * 16
        o.append(f'<line x1="{PAD_L+pw-150}" y1="{ly-4}" x2="{PAD_L+pw-128}" '
                 f'y2="{ly-4}" stroke="{c}" stroke-width="2.5"/>')
        o.append(f'<text x="{PAD_L+pw-122}" y="{ly}" font-size="11" fill="#333">'
                 f'{esc(label)}</text>')

    o.append("</svg>")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(o) + "\n")


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--column", default="err_px")
    ap.add_argument("--title", default="")
    ap.add_argument("--subtitle", default="")
    ap.add_argument("--ylabel", default="")
    ap.add_argument("inputs", nargs="+", metavar="LABEL=TRACE.csv")
    a = ap.parse_args(argv)

    datasets = []
    for spec in a.inputs:
        if "=" not in spec:
            raise SystemExit(f"expected LABEL=path, got '{spec}'")
        # rsplit, not split: labels are human text and routinely contain '='
        # ("k_ff = 0"), whereas paths almost never do. Splitting on the first
        # one truncated every label at its first equals sign.
        label, path = spec.rsplit("=", 1)
        _, rows = read_trace(path)
        pts = series(rows, a.column)
        if not pts:
            print(f"warning: '{a.column}' had no values in {path}", file=sys.stderr)
        datasets.append((label, pts))

    render(datasets, a.column, a.title or a.column, a.subtitle,
           a.ylabel or a.column, a.out)
    print(a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
