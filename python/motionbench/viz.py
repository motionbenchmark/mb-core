"""Dataset explorer: one self-contained HTML page for verifying a MotionBench dataset.

    motionbench viz data/synthetic-v0.1 -o explorer.html

The page draws every run on its arena (scrub, play, overlay trials or agents),
shows the Rust scorer's report, and re-checks the data independently in the
browser: it re-counts waypoints, deliveries and coverage from the raw poses and
flags any run where that recount disagrees with the scorer, plus physical
sanity checks (time gaps, implausible speed, leaving the arena, budget overrun).

Poses are embedded at full resolution as base64 integer arrays (ms, mm, mrad),
which is exactly the precision of the trace CSVs, so the recount sees the same
numbers the scorer saw.
"""

from __future__ import annotations

import base64
import csv
import json
from array import array
from pathlib import Path
from typing import Optional

from . import score_dir, tasks as list_tasks


def _pack(values, typecode: str) -> str:
    a = array(typecode, values)
    if a.itemsize > 1 and array("H", [1]).tobytes()[0] != 1:  # always little-endian on the wire
        a.byteswap()
    return base64.b64encode(a.tobytes()).decode("ascii")


def _load_run(d: Path) -> dict:
    t, x, y, th = [], [], [], []
    with open(d / "poses.csv") as f:
        for r in csv.DictReader(f):
            t.append(round(float(r["t_s"]) * 1000))
            x.append(round(float(r["x_m"]) * 1000))
            y.append(round(float(r["y_m"]) * 1000))
            th.append(round(float(r["theta_rad"]) * 1000))
    events = []
    if (d / "events.csv").exists():
        with open(d / "events.csv") as f:
            events = [[float(r["t_s"]), r["type"], r.get("object_id", ""), r.get("detail", "")] for r in csv.DictReader(f)]
    world = json.loads((d / "world_final.json").read_text()) if (d / "world_final.json").exists() else None
    m = json.loads((d / "manifest.json").read_text())
    return {
        "path": str(d),
        "notes": m.get("notes", ""),
        "capture": m.get("capture", ""),
        "n": len(t),
        "t": _pack(t, "i"),
        "x": _pack(x, "h"),
        "y": _pack(y, "h"),
        "th": _pack(th, "h"),
        "events": events,
        "world": world,
    }


def build(root: "str | Path", title: Optional[str] = None, tasks_dir=None, embed: bool = False) -> str:
    """Return the explorer HTML for every trace under ``root``.

    ``embed=True`` produces the variant for hosting inside another page (an iframe on
    the project site): light theme, no page header, no outer padding, and it posts
    its height to the parent so the frame can grow to fit.
    """
    root = Path(root)
    reports, errors = score_dir(root, tasks_dir)
    runs = []
    # Match each report to its directory by (task, seed, agent, trial).
    index = {}
    for d in sorted(p.parent for p in root.rglob("manifest.json")):
        m = json.loads((d / "manifest.json").read_text())
        index[(m["task"], m["seed"], m["agent"]["id"], m.get("trial", 0))] = d
    for r in reports:
        d = index.get((r["task"], r["seed"], r["agent_id"], r["trial"]))
        if d is None:
            continue
        run = _load_run(d)
        run["report"] = r
        runs.append(run)
    used = {r["report"]["task"] for r in runs}
    task_specs = [t for t in list_tasks(tasks_dir) if f"{t['id']}@{t['version']}" in used]
    synthetic = any("synthetic" in (r["capture"] + r["notes"]).lower() for r in runs)
    payload = {
        "dataset": title or root.name,
        "synthetic": synthetic,
        "tasks": task_specs,
        "runs": runs,
        "load_errors": errors,
    }
    data = json.dumps(payload, separators=(",", ":")).replace("</", "<\\/")
    html = TEMPLATE.replace("__TITLE__", f"{payload['dataset']} explorer").replace("__DATA__", data)
    return html.replace("__EMBED__", "true" if embed else "false")


def write(root, out: "str | Path", **kw) -> Path:
    """Write the explorer as a standalone document (with a doctype, so browsers use
    standards mode). ``build()`` alone returns a fragment for hosts that add their own."""
    out = Path(out)
    out.write_text("<!doctype html>\n" + build(root, **kw), encoding="utf-8")
    return out


TEMPLATE = r"""<meta charset="utf-8">
<title>__TITLE__</title>
<meta name="claude:viewport-fit-sides" content="cover">
<script>
  // Embed mode (motionbench viz --embed): light theme, host page supplies the header.
  window.MB_EMBED = __EMBED__;
  if (window.MB_EMBED) { document.documentElement.dataset.theme = "light"; document.documentElement.classList.add("embed"); }
</script>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wdth,wght@62..125,400..800&display=swap">
<style>
/* Layout: control rail | field canvas with scrubber | run inspector; checks + strip plot below. */
:root {
  --bg: #e3e6e1; --panel: #f4f5f2; --ink: #18212b; --ink-2: #47525e; --grid: #c3c9c1; --rule: #b4bbb2;
  --tile: #d3d8d1; --tile-line: #bcc3ba; --obstacle: #3b4652; --unseen: #eceee9; --frame: #18212b;
  --good: #1e7d4f; --warn: #a15c00; --bad: #b42335; --focus: #1f6fd1;
  --c-human: #d23f5b; --c-ai: #1f6fd1; --c-human_auton: #d98316; --c-product: #178a62; --c-hybrid: #8a4fd0; --c-baseline: #7c8590;
  --font: "Archivo", system-ui, -apple-system, "Segoe UI", sans-serif;
  --mono: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #141a20; --panel: #1c242c; --ink: #e6ebef; --ink-2: #a5b0ba; --grid: #34404b; --rule: #3a4651;
  --tile: #26303a; --tile-line: #313d48; --obstacle: #8796a5; --unseen: #12181e; --frame: #0d1115;
  --good: #4cc38a; --warn: #e3a447; --bad: #f0707f; --focus: #6aa6f5;
  --c-human: #f0607a; --c-ai: #5b9bf0; --c-human_auton: #e6a33a; --c-product: #3db88b; --c-hybrid: #b07fe8; --c-baseline: #95a0ab;
  color-scheme: dark; } }
:root[data-theme="dark"] {
  --bg: #141a20; --panel: #1c242c; --ink: #e6ebef; --ink-2: #a5b0ba; --grid: #34404b; --rule: #3a4651;
  --tile: #26303a; --tile-line: #313d48; --obstacle: #8796a5; --unseen: #12181e; --frame: #0d1115;
  --good: #4cc38a; --warn: #e3a447; --bad: #f0707f; --focus: #6aa6f5;
  --c-human: #f0607a; --c-ai: #5b9bf0; --c-human_auton: #e6a33a; --c-product: #3db88b; --c-hybrid: #b07fe8; --c-baseline: #95a0ab;
  color-scheme: dark; }
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink); font: 15px/1.45 var(--font);
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); padding-block: 18px 40px; }
h1 { font-size: 1.55rem; font-weight: 800; font-stretch: 120%; letter-spacing: -0.015em; margin: 0; text-wrap: balance; }
h2 { font-size: 1.05rem; font-weight: 720; font-stretch: 108%; margin: 0 0 10px; }
.tnum { font-variant-numeric: tabular-nums; }
.muted { color: var(--ink-2); }
header.top { display: flex; flex-wrap: wrap; gap: 8px 16px; align-items: baseline; margin-bottom: 14px; }
.badge { display: inline-block; font-size: 12px; font-weight: 650; padding: 2px 8px; border-radius: 999px; border: 1.5px solid currentColor; }
.badge.synthetic { color: var(--warn); }
.app { display: grid; grid-template-columns: 230px minmax(0, 1fr) 300px; gap: 16px; align-items: start; }
@media (max-width: 1040px) { .app { grid-template-columns: 220px minmax(0, 1fr); } .inspector { grid-column: 1 / -1; } }
@media (max-width: 700px) { .app { grid-template-columns: minmax(0, 1fr); } }
.card { background: var(--panel); border: 1px solid var(--grid); border-radius: 8px; padding: 14px; min-width: 0; }
label.f { display: block; font-size: 12.5px; font-weight: 650; color: var(--ink-2); margin: 10px 0 4px; }
label.f:first-child { margin-top: 0; }
select, button { font: inherit; color: var(--ink); }
select { width: 100%; padding: 6px 8px; border-radius: 6px; border: 1px solid var(--rule); background: var(--bg); }
button { cursor: pointer; border: 1.5px solid var(--ink); background: transparent; border-radius: 6px; padding: 5px 12px; font-weight: 650; }
button:hover { background: var(--bg); }
:focus-visible { outline: 3px solid var(--focus); outline-offset: 2px; }
.seg { display: flex; flex-direction: column; gap: 4px; }
.seg label { display: flex; gap: 8px; align-items: center; font-size: 14px; }
.legend { list-style: none; padding: 0; margin: 12px 0 0; display: grid; gap: 4px; font-size: 13px; }
.dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 7px; vertical-align: -0.05em; }
.fieldwrap { position: relative; }
canvas#field { width: 100%; height: auto; display: block; border-radius: 8px; background: var(--frame); touch-action: none; }
.scrub { display: flex; gap: 10px; align-items: center; margin-top: 10px; }
.scrub input { flex: 1; min-width: 0; accent-color: var(--ink); }
.scrub output { font-variant-numeric: tabular-nums; min-width: 8.5ch; text-align: right; font-size: 13.5px; }
dl.kv { display: grid; grid-template-columns: auto 1fr; gap: 4px 12px; margin: 0; font-size: 13.5px; }
dl.kv dt { color: var(--ink-2); }
dl.kv dd { margin: 0; text-align: right; font-variant-numeric: tabular-nums; }
.verdict { display: flex; flex-direction: column; gap: 2px; padding: 8px 10px; border-radius: 6px; margin: 10px 0; font-size: 13.5px; font-weight: 600; border: 1.5px solid; }
.verdict.ok { color: var(--good); } .verdict.bad { color: var(--bad); } .verdict.warn { color: var(--warn); }
.verdict span { color: var(--ink); font-weight: 450; }
.below { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); gap: 16px; margin-top: 16px; }
@media (max-width: 1000px) { .below { grid-template-columns: minmax(0, 1fr); } }
table.checks { width: 100%; border-collapse: collapse; font-size: 13.5px; }
table.checks td, table.checks th { padding: 6px 8px; border-bottom: 1px solid var(--grid); text-align: left; vertical-align: top; }
table.checks th { color: var(--ink-2); font-weight: 650; }
table.checks td.n { text-align: right; font-variant-numeric: tabular-nums; }
.chip { display: inline-block; min-width: 3.2em; text-align: center; font-size: 12px; font-weight: 700; padding: 1px 7px; border-radius: 999px; }
.chip.pass { background: color-mix(in srgb, var(--good) 18%, transparent); color: var(--good); }
.chip.fail { background: color-mix(in srgb, var(--bad) 18%, transparent); color: var(--bad); }
.chip.warn { background: color-mix(in srgb, var(--warn) 20%, transparent); color: var(--warn); }
.flagged { margin-top: 6px; display: flex; flex-wrap: wrap; gap: 4px; }
.flagged button { font-size: 12px; padding: 1px 7px; border-width: 1px; font-weight: 500; }
.scroll { overflow-x: auto; }
svg text { fill: var(--ink-2); font-family: var(--font); }
.hint { font-size: 12.5px; color: var(--ink-2); margin: 8px 0 0; }
@media (prefers-reduced-motion: reduce) { * { transition: none !important; } }
html.embed body { padding-block: 0 4px; padding-left: 0; padding-right: 0; background: transparent; }
html.embed header.top { display: none; }
</style>

<header class="top">
  <h1 id="title">Dataset explorer</h1>
  <span id="summary" class="muted tnum"></span>
  <span id="synth" class="badge synthetic" hidden>Synthetic data</span>
</header>

<div class="app">
  <aside class="card" aria-label="Choose runs">
    <label class="f" for="task">Task</label><select id="task"></select>
    <label class="f" for="seed">Seed</label><select id="seed"></select>
    <label class="f" for="agent">Agent</label><select id="agent"></select>
    <label class="f" for="trial">Trial</label><select id="trial"></select>
    <label class="f" id="ovl-l">Also draw</label>
    <div class="seg" role="radiogroup" aria-labelledby="ovl-l">
      <label><input type="radio" name="ovl" id="ovl-none" value="none" checked> Nothing else</label>
      <label><input type="radio" name="ovl" id="ovl-trials" value="trials"> This agent's other trials</label>
      <label><input type="radio" name="ovl" id="ovl-class" value="class"> Same class, trial 0</label>
      <label><input type="radio" name="ovl" id="ovl-all" value="all"> Every agent, trial 0</label>
    </div>
    <ul class="legend" id="legend"></ul>
  </aside>

  <section class="card" aria-label="Field">
    <div class="fieldwrap"><canvas id="field" width="800" height="800" role="img" aria-label="Top-down arena with the selected run"></canvas></div>
    <div class="scrub">
      <button id="play" type="button">Play</button>
      <input id="time" type="range" min="0" max="1000" value="1000" step="1" aria-label="Time">
      <output id="tout" for="time">0.0 s</output>
    </div>
    <p class="hint" id="hint"></p>
  </section>

  <section class="card inspector" aria-label="Selected run">
    <h2 id="runname">Run</h2>
    <div id="verdict"></div>
    <dl class="kv" id="kv"></dl>
    <p class="hint" id="warn"></p>
  </section>
</div>

<div class="below">
  <section class="card" aria-label="Dataset checks">
    <h2>Checks across every run</h2>
    <div class="scroll"><table class="checks" id="checks"></table></div>
    <p class="hint">Independent of the scorer: these are recomputed in this page from the raw poses. Click a flagged run to open it.</p>
  </section>
  <section class="card" aria-label="Score distribution">
    <h2 id="striptitle">Every run on this task</h2>
    <div class="scroll"><svg id="strip" width="100%" role="img" aria-label="Success score of every run by agent"></svg></div>
    <p class="hint">One dot per run. The dashed line is the median of human agents' averages. Click a dot to open that run.</p>
  </section>
</div>

<script id="mbdata" type="application/json">__DATA__</script>
<script>
"use strict";
const D = JSON.parse(document.getElementById("mbdata").textContent);
const $ = (id) => document.getElementById(id);
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const CLASS_LABEL = { human: "Human", ai: "AI-written", human_auton: "Team-written code", product: "Commercial robot", hybrid: "Human + assist", baseline: "Baseline" };
const CLASS_ORDER = ["human", "ai", "human_auton", "product", "hybrid", "baseline"];
const key = (t) => `${t.id}@${t.version}`;
const TASKS = Object.fromEntries(D.tasks.map((t) => [key(t), t]));

// ---------- decode poses (little-endian base64 int arrays: ms, mm, mm, mrad)
function unpack(b64, Ctor) {
  const bin = atob(b64); const buf = new ArrayBuffer(bin.length); const u8 = new Uint8Array(buf);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return new Ctor(buf);
}
for (const r of D.runs) {
  const t = unpack(r.t, Int32Array), x = unpack(r.x, Int16Array), y = unpack(r.y, Int16Array), th = unpack(r.th, Int16Array);
  r.T = Float64Array.from(t, (v) => v / 1000); r.X = Float64Array.from(x, (v) => v / 1000);
  r.Y = Float64Array.from(y, (v) => v / 1000); r.TH = Float64Array.from(th, (v) => v / 1000);
  r.rep = r.report; r.cls = r.rep.agent_class;
}

// ---------- independent recount (same rules as the spec, written separately from the Rust scorer)
const angDiff = (a, b) => { let d = (a - b) % (2 * Math.PI); if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI; return Math.abs(d); };
function inBudget(r, task) { const t0 = r.T[0]; let n = 0; while (n < r.n && r.T[n] - t0 <= task.budget.time_s + 1e-9) n++; return n; }
function recount(r) {
  const task = TASKS[r.rep.task]; const g = task.goal; const lay = task.layouts[r.rep.seed]; const n = inBudget(r, task);
  if (g.kind === "waypoints") {
    const hits = []; let k = 0;
    for (let i = 0; i < n && k < lay.waypoints.length; i++) {
      const w = lay.waypoints[k];
      if (Math.hypot(r.X[i] - w.x, r.Y[i] - w.y) <= g.pos_tol_m && angDiff(r.TH[i], w.theta) <= g.heading_tol_rad) { hits.push(i); k++; }
    }
    return { score: hits.length, hits };
  }
  if (g.kind === "delivery") {
    // each object's own target zone (e.g. sorting by colour), else the goal's zone
    const target = new Map(lay.objects.map((o) => [o.id, task.arena.zones[o.zone || g.zone]]));
    const ids = new Set(lay.objects.map((o) => o.id));
    const inZ = (o) => { const z = target.get(o.id); return !!z && o.x >= z.x && o.x <= z.x + z.w && o.y >= z.y && o.y <= z.y + z.h; };
    if (r.world) return { score: r.world.objects.filter((o) => ids.has(o.id) && inZ(o)).length };
    return { score: new Set(r.events.filter((e) => e[1] === "deliver" && ids.has(e[2])).map((e) => e[2])).size };
  }
  // coverage
  const cell = g.cell_m, rt = g.tool_width_m / 2, W = task.arena.width_m, H = task.arena.height_m;
  const nx = Math.ceil(W / cell), ny = Math.ceil(H / cell);
  const free = new Uint8Array(nx * ny), cov = new Uint16Array(nx * ny);
  let nFree = 0;
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const cx = (i + 0.5) * cell, cy = (j + 0.5) * cell;
    const f = cx <= W && cy <= H && !task.arena.obstacles.some((o) => cx >= o.x && cx <= o.x + o.w && cy >= o.y && cy <= o.y + o.h);
    free[j * nx + i] = f ? 1 : 0; nFree += f ? 1 : 0;
  }
  let nCov = 0;
  const stamp = (x, y) => {
    const i0 = Math.max(0, Math.floor((x - rt) / cell)), i1 = Math.min(nx, Math.ceil((x + rt) / cell));
    const j0 = Math.max(0, Math.floor((y - rt) / cell)), j1 = Math.min(ny, Math.ceil((y + rt) / cell));
    for (let j = j0; j < j1; j++) for (let i = i0; i < i1; i++) {
      const k = j * nx + i; if (!free[k]) continue;
      if (Math.hypot(x - (i + 0.5) * cell, y - (j + 0.5) * cell) <= rt) { if (!cov[k]) nCov++; if (cov[k] < 65535) cov[k]++; }
    }
  };
  stamp(r.X[0], r.Y[0]);
  for (let s = 1; s < n; s++) {
    const ax = r.X[s - 1], ay = r.Y[s - 1], bx = r.X[s], by = r.Y[s];
    const steps = Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay) / (cell / 2)));
    for (let q = 1; q <= steps; q++) stamp(ax + (bx - ax) * q / steps, ay + (by - ay) * q / steps);
  }
  return { score: 100 * nCov / Math.max(1, nFree), grid: { nx, ny, cell, cov, free } };
}

// ---------- sanity checks per run
const CHECKS = [
  { id: "recount", label: "Recount matches the scorer", sev: "fail" },
  { id: "time", label: "Time strictly increasing", sev: "fail" },
  { id: "gap", label: "No gap over 0.5 s between samples", sev: "warn" },
  { id: "speed", label: "Speed under 3 m/s", sev: "warn" },
  { id: "bounds", label: "Robot centre stays inside the arena", sev: "fail" },
  { id: "budget", label: "Run fits its time budget", sev: "warn" },
  { id: "warnings", label: "No scorer warnings", sev: "warn" },
];
function check(r) {
  const task = TASKS[r.rep.task]; const out = {};
  const rc = recount(r); r.recount = rc;
  const tol = task.goal.kind === "coverage" ? 0.05 : 1e-9;
  out.recount = Math.abs(rc.score - r.rep.success.score) <= tol;
  let mono = true, gap = 0, vmax = 0, oob = 0;
  for (let i = 1; i < r.n; i++) {
    const dt = r.T[i] - r.T[i - 1];
    if (dt <= 0) mono = false;
    gap = Math.max(gap, dt);
    if (dt > 0) vmax = Math.max(vmax, Math.hypot(r.X[i] - r.X[i - 1], r.Y[i] - r.Y[i - 1]) / dt);
  }
  for (let i = 0; i < r.n; i++) if (r.X[i] < -0.02 || r.Y[i] < -0.02 || r.X[i] > task.arena.width_m + 0.02 || r.Y[i] > task.arena.height_m + 0.02) oob++;
  out.time = mono; out.gap = gap <= 0.5; out.speed = vmax < 3; out.bounds = oob === 0;
  out.budget = r.T[r.n - 1] - r.T[0] <= task.budget.time_s + 1e-6;
  out.warnings = !(r.rep.warnings && r.rep.warnings.length);
  r.stats = { gap, vmax, oob };
  return out;
}
for (const r of D.runs) r.checks = check(r);

// ---------- selection state
const S = { task: null, seed: null, agent: null, trial: null, ovl: "none", t: Infinity };
const runsFor = (task, seed) => D.runs.filter((r) => r.rep.task === task && r.rep.seed === seed);
const current = () => D.runs.find((r) => r.rep.task === S.task && r.rep.seed === S.seed && r.rep.agent_id === S.agent && r.rep.trial === S.trial);
function opts(sel, items, value) {
  sel.innerHTML = ""; for (const [v, label] of items) { const o = document.createElement("option"); o.value = v; o.textContent = label; sel.appendChild(o); }
  sel.value = items.some(([v]) => v === value) ? value : (items[0] || [""])[0];
  return sel.value;
}
function fillControls() {
  S.task = opts($("task"), Object.keys(TASKS).map((k) => [k, `${TASKS[k].title} (${k})`]), S.task);
  const seeds = [...new Set(D.runs.filter((r) => r.rep.task === S.task).map((r) => r.rep.seed))].sort();
  S.seed = opts($("seed"), seeds.map((s) => [s, s]), S.seed);
  const rs = runsFor(S.task, S.seed);
  const agents = [...new Map(rs.map((r) => [r.rep.agent_id, r])).values()]
    .sort((a, b) => CLASS_ORDER.indexOf(a.cls) - CLASS_ORDER.indexOf(b.cls) || a.rep.agent_name.localeCompare(b.rep.agent_name));
  S.agent = opts($("agent"), agents.map((r) => [r.rep.agent_id, `${r.rep.agent_name} · ${CLASS_LABEL[r.cls]}`]), S.agent);
  const trials = rs.filter((r) => r.rep.agent_id === S.agent).map((r) => r.rep.trial).sort((a, b) => a - b);
  S.trial = Number(opts($("trial"), trials.map((t) => [String(t), `Trial ${t}`]), String(S.trial)));
}

// ---------- drawing
const cv = $("field"); const ctx = cv.getContext("2d");
let geo = null;
function resize() {
  const task = TASKS[S.task]; if (!task) return;
  const w = cv.clientWidth || 800; const pad = 22;
  const W = task.arena.width_m, H = task.arena.height_m; const sc = (w - 2 * pad) / W;
  const h = Math.round(H * sc + 2 * pad); const dpr = window.devicePixelRatio || 1;
  cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); cv.style.height = h + "px";
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  geo = { pad, sc, W, H, X: (x) => pad + x * sc, Y: (y) => pad + (H - y) * sc };
}
function pathTo(r, tEnd, n) {
  ctx.beginPath(); let started = false;
  for (let i = 0; i < n; i++) { if (r.T[i] - r.T[0] > tEnd) break; const px = geo.X(r.X[i]), py = geo.Y(r.Y[i]); started ? ctx.lineTo(px, py) : (ctx.moveTo(px, py), started = true); }
}
function idxAt(r, tRel) { let lo = 0, hi = r.n - 1; const t = r.T[0] + tRel; while (lo < hi) { const m = (lo + hi + 1) >> 1; if (r.T[m] <= t) lo = m; else hi = m - 1; } return lo; }
function draw() {
  const task = TASKS[S.task]; const r = current(); if (!task || !r || !geo) return;
  const { X, Y, sc, W, H } = geo; const lay = task.layouts[S.seed];
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = css("--tile"); ctx.fillRect(X(0), Y(H), W * sc, H * sc);
  const step = task.domain === "vex" ? 0.6096 : 0.5;
  ctx.strokeStyle = css("--tile-line"); ctx.lineWidth = 1;
  for (let gx = step; gx < W - 1e-6; gx += step) { ctx.beginPath(); ctx.moveTo(X(gx), Y(0)); ctx.lineTo(X(gx), Y(H)); ctx.stroke(); }
  for (let gy = step; gy < H - 1e-6; gy += step) { ctx.beginPath(); ctx.moveTo(X(0), Y(gy)); ctx.lineTo(X(W), Y(gy)); ctx.stroke(); }

  // coverage heat for the selected run
  if (task.goal.kind === "coverage" && r.recount.grid) {
    const g = r.recount.grid; const tEnd = Math.min(S.t, Infinity);
    const sub = tEnd < r.T[r.n - 1] - r.T[0] ? recountUpTo(r, tEnd) : g;
    const col = css(`--c-${r.cls}`);
    for (let j = 0; j < g.ny; j++) for (let i = 0; i < g.nx; i++) {
      const v = sub.cov[j * g.nx + i]; if (!v) continue;
      ctx.globalAlpha = Math.min(0.55, 0.18 + 0.12 * (v - 1));
      ctx.fillStyle = col; ctx.fillRect(X(i * g.cell), Y((j + 1) * g.cell), g.cell * sc + 0.5, g.cell * sc + 0.5);
    }
    ctx.globalAlpha = 1;
  }
  // Field captures mark floor the camera never saw as "not-in-view": drawn faint, not as a solid obstacle.
  let hatch = null;
  for (const o of task.arena.obstacles) {
    if (o.label === "not-in-view") {
      if (!hatch) {
        const pc = document.createElement("canvas"); pc.width = pc.height = 8;
        const g = pc.getContext("2d"); g.fillStyle = css("--unseen"); g.fillRect(0, 0, 8, 8);
        g.strokeStyle = css("--rule"); g.lineWidth = 1; g.beginPath(); g.moveTo(0, 8); g.lineTo(8, 0); g.stroke();
        hatch = ctx.createPattern(pc, "repeat");
      }
      ctx.fillStyle = hatch;
    } else ctx.fillStyle = css("--obstacle");
    ctx.fillRect(X(o.x), Y(o.y + o.h), o.w * sc + 0.5, o.h * sc + 0.5);
  }
  ctx.setLineDash([6, 4]); ctx.strokeStyle = css("--c-ai"); ctx.lineWidth = 2;
  const zoneNames = Object.keys(task.arena.zones || {});
  for (const z of Object.values(task.arena.zones || {})) ctx.strokeRect(X(z.x), Y(z.y + z.h), z.w * sc, z.h * sc);
  ctx.setLineDash([]);
  if (zoneNames.length > 1) {
    ctx.fillStyle = css("--ink"); ctx.font = "700 11px " + css("--font"); ctx.textAlign = "center"; ctx.textBaseline = "bottom";
    for (const [name, z] of Object.entries(task.arena.zones)) ctx.fillText(name + " zone", X(z.x + z.w / 2), Y(z.y + z.h) - 4);
  }

  // overlays
  const all = runsFor(S.task, S.seed);
  let others = [];
  if (S.ovl === "trials") others = all.filter((o) => o.rep.agent_id === S.agent && o !== r);
  if (S.ovl === "class") others = all.filter((o) => o.cls === r.cls && o.rep.trial === 0 && o !== r);
  if (S.ovl === "all") others = all.filter((o) => o.rep.trial === 0 && o !== r);
  ctx.lineWidth = 1.5; ctx.lineJoin = "round";
  for (const o of others) { ctx.globalAlpha = others.length > 8 ? 0.35 : 0.55; ctx.strokeStyle = css(`--c-${o.cls}`); pathTo(o, Infinity, o.n); ctx.stroke(); }
  ctx.globalAlpha = 1;

  // waypoints / objects
  if (lay.waypoints) lay.waypoints.forEach((w, i) => {
    const cx = X(w.x), cy = Y(w.y);
    ctx.beginPath(); ctx.arc(cx, cy, 10, 0, 7); ctx.fillStyle = css("--panel"); ctx.fill(); ctx.strokeStyle = css("--ink"); ctx.lineWidth = 1.5; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(w.theta) * 18, cy - Math.sin(w.theta) * 18); ctx.stroke();
    ctx.fillStyle = css("--ink"); ctx.font = "700 11.5px " + css("--font"); ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(String(i + 1), cx, cy + 0.5);
  });
  if (lay.objects) {
    const fin = new Map((r.world ? r.world.objects : []).map((o) => [o.id, o]));
    const sorted = zoneNames.length > 1;
    for (const o of lay.objects) {
      const objCol = sorted ? css("--ink") : css("--c-human_auton");
      ctx.beginPath(); ctx.arc(X(o.x), Y(o.y), 6, 0, 7); ctx.strokeStyle = objCol; ctx.lineWidth = 2; ctx.stroke();
      const f = fin.get(o.id);
      const ended = f && S.t >= r.T[r.n - 1] - r.T[0];
      if (ended) { ctx.beginPath(); ctx.arc(X(f.x), Y(f.y), 6, 0, 7); ctx.fillStyle = objCol; ctx.fill(); }
      if (sorted && o.zone) {
        // letter = target zone, drawn at the start ring and, once the run ends, on the final position
        ctx.font = "700 9px " + css("--font"); ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillStyle = objCol; ctx.fillText(o.zone[0].toUpperCase(), X(o.x), Y(o.y) - 13);
        if (ended) { ctx.fillStyle = css("--panel"); ctx.fillText(o.zone[0].toUpperCase(), X(f.x), Y(f.y) + 0.5); }
      }
    }
  }
  // start pose
  ctx.beginPath(); ctx.arc(X(lay.start.x), Y(lay.start.y), 4, 0, 7); ctx.fillStyle = css("--ink-2"); ctx.fill();

  // selected run
  const col = css(`--c-${r.cls}`);
  ctx.strokeStyle = col; ctx.lineWidth = 3; pathTo(r, S.t, r.n); ctx.stroke();
  if (r.recount.hits) for (const i of r.recount.hits) if (r.T[i] - r.T[0] <= S.t) {
    ctx.beginPath(); ctx.arc(X(r.X[i]), Y(r.Y[i]), 4, 0, 7); ctx.fillStyle = col; ctx.fill(); ctx.strokeStyle = css("--panel"); ctx.lineWidth = 1.5; ctx.stroke();
  }
  const k = idxAt(r, Math.min(S.t, r.T[r.n - 1] - r.T[0]));
  const rx = X(r.X[k]), ry = Y(r.Y[k]);
  ctx.beginPath(); ctx.arc(rx, ry, task.robot.radius_m * sc, 0, 7); ctx.globalAlpha = 0.14; ctx.fillStyle = col; ctx.fill(); ctx.globalAlpha = 1;
  ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.stroke();
  ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(rx + Math.cos(r.TH[k]) * task.robot.radius_m * sc, ry - Math.sin(r.TH[k]) * task.robot.radius_m * sc); ctx.stroke();
  // frame
  ctx.strokeStyle = css("--ink"); ctx.lineWidth = 2; ctx.strokeRect(X(0), Y(H), W * sc, H * sc);
}
let covCache = null;
function recountUpTo(r, tEnd) {
  const key = r.path + "|" + Math.round(tEnd * 4);
  if (covCache && covCache.key === key) return covCache.g;
  const n = idxAt(r, tEnd) + 1; const copy = Object.assign({}, r, { n });
  const g = recount(copy).grid; covCache = { key, g }; return g;
}

// ---------- inspector
const f2 = (v, d = 2) => (v === null || v === undefined ? "–" : Number(v).toFixed(d));
function inspect() {
  const r = current(); if (!r) return; const p = r.rep; const task = TASKS[p.task];
  $("runname").textContent = `${p.agent_name}, trial ${p.trial}`;
  const unit = task.goal.kind === "coverage" ? "%" : "";
  const ok = r.checks.recount;
  $("verdict").innerHTML = `<div class="verdict ${ok ? "ok" : "bad"}">${ok ? "Recount matches" : "Recount differs"} <span class="tnum">scorer ${f2(p.success.score, task.goal.kind === "coverage" ? 1 : 0)}${unit} · this page ${f2(r.recount.score, task.goal.kind === "coverage" ? 1 : 0)}${unit}</span></div>`;
  const rows = [
    ["Class", CLASS_LABEL[r.cls]],
    ["Success", `${f2(p.success.score, task.goal.kind === "coverage" ? 1 : 0)} / ${f2(p.success.max_score, 0)}${p.success.completed ? " (complete)" : ""}`],
    ["Time to goal", p.efficiency.time_to_goal_s == null ? "–" : `${f2(p.efficiency.time_to_goal_s, 1)} s`],
    ["Path", `${f2(p.efficiency.path_length_m)} m`],
    ["Path efficiency", f2(p.efficiency.path_efficiency)],
    ["Mean speed", `${f2(p.process.mean_speed_mps)} m/s`],
    ["Top speed (this page)", `${f2(r.stats.vmax)} m/s`],
    ["Hesitation", `${f2(p.process.hesitation_s, 1)} s`],
    ["Violations", String(p.safety.total)],
    ["Samples", `${r.n} over ${f2(r.T[r.n - 1] - r.T[0], 1)} s`],
    ["Largest gap", `${f2(r.stats.gap, 2)} s`],
  ];
  if (p.precision.mean_target_error_m != null) rows.splice(5, 0, ["Mean target error", `${f2(p.precision.mean_target_error_m * 100, 1)} cm`]);
  if (p.coverage) rows.splice(5, 0, ["Redundant sweep", `${f2(p.coverage.redundant_pct, 1)}%`]);
  $("kv").innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
  const warn = (p.warnings || []).join(" · ");
  $("warn").textContent = warn ? `Scorer warnings: ${warn}` : "";
  const dur = r.T[r.n - 1] - r.T[0];
  $("time").max = String(Math.round(dur * 100));
  if (!isFinite(S.t) || S.t > dur) S.t = dur;
  $("time").value = String(Math.round(S.t * 100));
  $("tout").textContent = `${S.t.toFixed(1)} s`;
  $("hint").textContent = task.goal.kind === "waypoints" ? "Filled dots on the path mark where this page counted each waypoint as reached."
    : task.goal.kind === "delivery" ? (Object.keys(task.arena.zones || {}).length > 1
      ? "Rings are starting object positions, lettered with their target zone; filled dots are final positions from world_final.json (shown at the end of the run). An object counts only inside its own zone."
      : "Rings are starting object positions; filled dots are final positions from world_final.json (shown at the end of the run).")
    : task.arena.obstacles.some((o) => o.label === "not-in-view")
    ? "Shaded cells are floor swept by the tool so far. Hatched cells were outside the camera's view and are left out of the score; dark cells are walls; plain cells are floor not yet swept."
    : "Shading shows swept floor; darker means swept more than once.";
}

// ---------- checks table
function renderChecks() {
  const rows = CHECKS.map((c) => {
    const bad = D.runs.filter((r) => !r.checks[c.id]);
    const chip = bad.length === 0 ? `<span class="chip pass">pass</span>` : `<span class="chip ${c.sev}">${bad.length}</span>`;
    const list = bad.slice(0, 24).map((r) => `<button type="button" data-run="${D.runs.indexOf(r)}">${r.rep.agent_id} ${r.rep.seed} t${r.rep.trial}</button>`).join("")
      + (bad.length > 24 ? `<span class="muted"> +${bad.length - 24} more</span>` : "");
    return `<tr><td>${chip}</td><td>${c.label}${bad.length ? `<div class="flagged">${list}</div>` : ""}</td><td class="n">${D.runs.length - bad.length}/${D.runs.length}</td></tr>`;
  });
  const errs = D.load_errors.length ? `<tr><td><span class="chip fail">${D.load_errors.length}</span></td><td>Traces the scorer could not load<div class="flagged muted">${D.load_errors.slice(0, 5).map((e) => e.error).join("<br>")}</div></td><td class="n"></td></tr>` : "";
  $("checks").innerHTML = `<thead><tr><th>Result</th><th>Check</th><th class="n">Passing</th></tr></thead><tbody>${rows.join("")}${errs}</tbody>`;
}
$("checks").addEventListener("click", (e) => { const b = e.target.closest("button[data-run]"); if (b) selectRun(D.runs[Number(b.dataset.run)]); });

// ---------- strip plot
function renderStrip() {
  const svg = $("strip"); const task = TASKS[S.task];
  const rs = D.runs.filter((r) => r.rep.task === S.task);
  const agents = [...new Map(rs.map((r) => [r.rep.agent_id, r])).values()]
    .map((r) => ({ id: r.rep.agent_id, name: r.rep.agent_name, cls: r.cls, mean: rs.filter((q) => q.rep.agent_id === r.rep.agent_id).reduce((s, q, _, a) => s + q.rep.success.score / a.length, 0) }))
    .sort((a, b) => b.mean - a.mean);
  const L = 190, P = 360, R = 16, rowH = 18, top = 8;
  const max = rs[0] ? rs[0].rep.success.max_score : 1;
  const x = (v) => L + (v / max) * P;
  const H = top + agents.length * rowH + 28;
  svg.setAttribute("viewBox", `0 0 ${L + P + R} ${H}`); svg.setAttribute("height", String(H));
  const humans = agents.filter((a) => a.cls === "human").map((a) => a.mean).sort((a, b) => a - b);
  const med = humans.length ? (humans.length % 2 ? humans[(humans.length - 1) / 2] : (humans[humans.length / 2 - 1] + humans[humans.length / 2]) / 2) : null;
  let s = "";
  const ticks = task.goal.kind === "coverage" ? [0, 25, 50, 75, 100] : Array.from({ length: Math.round(max) + 1 }, (_, i) => i);
  for (const v of ticks) { s += `<line x1="${x(v)}" x2="${x(v)}" y1="${top - 4}" y2="${H - 24}" stroke="var(--grid)"/><text x="${x(v)}" y="${H - 8}" text-anchor="middle" font-size="11">${task.goal.kind === "coverage" ? v + "%" : v}</text>`; }
  if (med !== null) s += `<line x1="${x(med)}" x2="${x(med)}" y1="${top - 4}" y2="${H - 24}" stroke="var(--ink)" stroke-dasharray="4 3"/>`;
  agents.forEach((a, i) => {
    const y = top + i * rowH + rowH / 2;
    const sel = a.id === S.agent;
    s += `<text x="${L - 10}" y="${y + 4}" text-anchor="end" font-size="11.5" style="fill:var(--${sel ? "ink" : "ink-2"});font-weight:${sel ? 700 : 400}">${a.name.length > 30 ? a.name.slice(0, 29) + "…" : a.name}</text>`;
    rs.filter((r) => r.rep.agent_id === a.id).forEach((r, j) => {
      const jitter = ((j % 5) - 2) * 2.2;
      s += `<circle cx="${x(r.rep.success.score)}" cy="${y + jitter}" r="${sel && r.rep.trial === S.trial && r.rep.seed === S.seed ? 5.5 : 3.6}" fill="var(--c-${r.cls})" stroke="var(--panel)" stroke-width="1.2" data-run="${D.runs.indexOf(r)}" style="cursor:pointer"><title>${r.rep.agent_name}, ${r.rep.seed} trial ${r.rep.trial}: ${r.rep.success.score.toFixed(task.goal.kind === "coverage" ? 1 : 0)}</title></circle>`;
    });
  });
  svg.innerHTML = s;
  $("striptitle").textContent = `Every run on ${S.task}`;
}
$("strip").addEventListener("click", (e) => { const c = e.target.closest("[data-run]"); if (c) selectRun(D.runs[Number(c.dataset.run)]); });

function renderLegend() {
  const present = CLASS_ORDER.filter((c) => D.runs.some((r) => r.cls === c && r.rep.task === S.task));
  $("legend").innerHTML = present.map((c) => `<li><span class="dot" style="background:var(--c-${c})"></span>${CLASS_LABEL[c]}</li>`).join("");
}

// ---------- wiring
function refresh(full) { fillControls(); if (full) { resize(); renderLegend(); } inspect(); draw(); renderStrip(); }
function selectRun(r, scroll = true) {
  S.task = r.rep.task; S.seed = r.rep.seed; S.agent = r.rep.agent_id; S.trial = r.rep.trial; S.t = Infinity; refresh(true);
  if (scroll) $("field").scrollIntoView({ block: "nearest" });
}
$("task").addEventListener("change", (e) => { S.task = e.target.value; S.t = Infinity; refresh(true); });
$("seed").addEventListener("change", (e) => { S.seed = e.target.value; S.t = Infinity; refresh(false); });
$("agent").addEventListener("change", (e) => { S.agent = e.target.value; S.t = Infinity; refresh(false); });
$("trial").addEventListener("change", (e) => { S.trial = Number(e.target.value); S.t = Infinity; refresh(false); });
document.querySelectorAll('input[name="ovl"]').forEach((el) => el.addEventListener("change", () => { S.ovl = el.value; draw(); }));
$("time").addEventListener("input", (e) => { S.t = Number(e.target.value) / 100; $("tout").textContent = `${S.t.toFixed(1)} s`; draw(); });
let raf = null;
$("play").addEventListener("click", () => {
  const r = current(); if (!r) return;
  if (raf) { cancelAnimationFrame(raf); raf = null; $("play").textContent = "Play"; return; }
  const dur = r.T[r.n - 1] - r.T[0]; if (S.t >= dur - 0.01) S.t = 0;
  const t0 = performance.now() - S.t * 1000; $("play").textContent = "Pause";
  const tick = (now) => {
    S.t = Math.min(dur, (now - t0) / 1000); $("time").value = String(Math.round(S.t * 100)); $("tout").textContent = `${S.t.toFixed(1)} s`; draw();
    if (S.t < dur) raf = requestAnimationFrame(tick); else { raf = null; $("play").textContent = "Play"; }
  };
  raf = requestAnimationFrame(tick);
});
window.addEventListener("resize", () => { resize(); draw(); });
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);
new MutationObserver(draw).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

// ---------- boot
$("title").textContent = D.dataset;
const nAgents = new Set(D.runs.map((r) => r.rep.agent_id)).size;
$("summary").textContent = `${D.runs.length} runs · ${Object.keys(TASKS).length} tasks · ${nAgents} agents`;
$("synth").hidden = !D.synthetic;
renderChecks();
const q = new URLSearchParams(location.search);
const want = (r) => (!q.get("task") || r.rep.task === q.get("task")) && (!q.get("seed") || r.rep.seed === q.get("seed"))
  && (!q.get("agent") || r.rep.agent_id === q.get("agent")) && (!q.get("trial") || r.rep.trial === Number(q.get("trial")));
let pool = D.runs.filter(want);
if (!q.get("task")) {
  // Default to the task with the most runs.
  const counts = {}; for (const r of D.runs) counts[r.rep.task] = (counts[r.rep.task] || 0) + 1;
  const top = Object.keys(counts).sort((a, b) => counts[b] - counts[a] || a.localeCompare(b))[0];
  pool = pool.filter((r) => r.rep.task === top);
}
const first = pool.find((r) => r.cls === "human") || pool[0] || D.runs[0];
if (first) selectRun(first, false);
if (window.MB_EMBED && window.parent !== window) {
  // Report content height (not scrollHeight, which never drops below the frame's own height).
  const post = () => {
    // Bottom of the last block, so the frame never feeds its own height back.
    const bottom = Math.max(...[...document.body.children].map((e) => e.getBoundingClientRect().bottom + window.scrollY));
    window.parent.postMessage({ type: "mb-explorer-height", height: Math.ceil(bottom) + 4 }, "*");
  };
  new ResizeObserver(post).observe(document.body);
  post();
}
</script>
"""
