"""Tap-to-calibrate: make the empty-floor picture of a video and a small offline
page where you tap the floor's corners, then save ``calib.json``.

    motionbench calibrate run.mov -o calib.html     # open calib.html, tap, download calib.json
    motionbench track run.mov --untagged --calib calib.json -o out/
"""

from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Optional

import numpy as np

from .stabilize import Stabilizer
from .video import _cv2, iter_frames, probe


def empty_floor(video: "str | Path", *, width: int = 540, samples: int = 90) -> np.ndarray:
    """Median of aligned frames across the whole video: moving things vanish."""
    cv2 = _cv2()
    info = probe(video)
    fps = max(0.5, min(10.0, samples / max(info.duration_s, 1e-3)))
    stab, acc = None, []
    for _, f in iter_frames(video, width=width, fps=fps):
        if stab is None:
            stab = Stabilizer(f)
            size = (f.shape[1], f.shape[0])
        acc.append(cv2.warpPerspective(f, stab.homography(f), size))
    if not acc:
        raise ValueError(f"no frames read from {video}")
    return np.median(np.stack(acc), axis=0).astype(np.uint8)


def write_calibration_page(video: "str | Path", out_html: "str | Path", *, width: int = 540,
                           png: Optional["str | Path"] = None, defaults: Optional[dict] = None) -> Path:
    cv2 = _cv2()
    bg = empty_floor(video, width=width)
    if png:
        cv2.imwrite(str(png), bg)
    ok, buf = cv2.imencode(".jpg", bg, [cv2.IMWRITE_JPEG_QUALITY, 88])
    data = base64.b64encode(buf.tobytes()).decode()
    h, w = bg.shape[:2]
    page = PAGE.replace("__IMG__", data).replace("__W__", str(w)).replace("__H__", str(h)) \
        .replace("__NAME__", Path(video).name).replace("__DEFAULTS__", json.dumps(defaults or {}))
    Path(out_html).write_text(page)
    return Path(out_html)


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>MotionBench calibration</title>
<style>
:root { --bg:#e3e6e1; --panel:#f4f5f2; --ink:#18212b; --ink2:#47525e; --red:#d23f5b; --blue:#1f6fd1; }
* { box-sizing: border-box; }
body { margin:0; background:var(--bg); color:var(--ink); font:15px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; }
main { max-width:1100px; margin:0 auto; padding:20px 16px 40px; display:grid; grid-template-columns:minmax(0,1fr) 340px; gap:20px; }
@media (max-width:820px){ main { grid-template-columns:1fr; } }
h1 { font-size:22px; margin:0 0 4px; } p { margin:6px 0; color:var(--ink2); }
.panel { background:var(--panel); border-radius:10px; padding:14px; }
canvas { max-width:100%; max-height:82vh; width:auto; height:auto; display:block; margin:0 auto; border-radius:8px; cursor:crosshair; touch-action:manipulation; }
label { display:block; margin:10px 0 2px; font-weight:600; font-size:13px; }
input, select { width:100%; font:inherit; padding:7px 9px; border:1.5px solid #b4bbb2; border-radius:6px; background:#fff; }
.row { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
button { font:inherit; font-weight:650; padding:9px 12px; border-radius:6px; border:2px solid var(--ink); background:transparent; color:var(--ink); cursor:pointer; margin:8px 6px 0 0; }
button.primary { background:var(--ink); color:var(--panel); }
.steps { margin:0; padding-left:18px; } .steps li { margin:3px 0; } .done { color:#1e7d4f; } .now { font-weight:700; }
textarea { width:100%; min-height:120px; font:12px ui-monospace,Menlo,monospace; border-radius:6px; border:1.5px solid #b4bbb2; padding:8px; }
.mode { display:flex; gap:6px; flex-wrap:wrap; } .mode button.on { background:var(--ink); color:var(--panel); }
</style></head><body>
<main>
  <section>
    <h1>Calibrate: __NAME__</h1>
    <p>This is the empty floor (the robot is removed by stacking every frame). Tap the four corners of a rectangle on the floor whose long sides run along the walls, in this order:</p>
    <ol class="steps" id="steps"><li>far-left</li><li>far-right</li><li>near-right</li><li>near-left</li></ol>
    <div class="mode" style="margin:10px 0">
      <button id="m-corners" class="on" type="button">Corners</button>
      <button id="m-outline" type="button">Floor outline (optional)</button>
      <button id="m-maxy" type="button">Ignore below (optional)</button>
      <button id="undo" type="button">Undo</button>
      <button id="clear" type="button">Clear</button>
    </div>
    <canvas id="c" width="__W__" height="__H__"></canvas>
    <p style="font-size:13px">Floor outline: tap around all the visible floor (including door nooks) if it is not just the rectangle. Ignore below: tap once to cut off the bottom of the picture (feet, a doormat).</p>
  </section>
  <aside class="panel">
    <label>Measured width, far-left to far-right (m)</label><input id="mw" type="number" step="0.01" placeholder="optional, e.g. 1.07">
    <label>Measured length, near-left to far-left (m)</label><input id="ml" type="number" step="0.01" placeholder="optional">
    <p style="font-size:13px">One tape measurement makes the size exact; two also fix the camera's lens.</p>
    <div class="row">
      <div><label>Robot diameter (m)</label><input id="rd" type="number" step="0.005" value="0.35"></div>
      <div><label>Robot height (m)</label><input id="rh" type="number" step="0.005" value="0.10"></div>
    </div>
    <label>Robot compared with the floor</label>
    <select id="pol"><option value="dark">darker</option><option value="light">lighter</option><option value="any">either</option></select>
    <button class="primary" id="dl" type="button">Download calib.json</button><button id="copy" type="button">Copy</button>
    <label>calib.json</label><textarea id="out" readonly></textarea>
  </aside>
</main>
<script>
const W = __W__, H = __H__, DEF = __DEFAULTS__;
const img = new Image(); img.src = "data:image/jpeg;base64,__IMG__";
const c = document.getElementById("c"), ctx = c.getContext("2d");
let mode = "corners", corners = [], outline = [], maxy = null;
const names = ["far-left", "far-right", "near-right", "near-left"];
const $ = (id) => document.getElementById(id);
for (const [k, id] of [["measured_width_m","mw"],["measured_length_m","ml"],["robot_diameter_m","rd"],["robot_height_m","rh"],["robot_polarity","pol"]]) if (DEF[k] != null) $(id).value = DEF[k];
function setMode(m) { mode = m; for (const b of ["corners","outline","maxy"]) $("m-" + b).classList.toggle("on", b === m); }
$("m-corners").onclick = () => setMode("corners"); $("m-outline").onclick = () => setMode("outline"); $("m-maxy").onclick = () => setMode("maxy");
$("undo").onclick = () => { if (mode === "outline" && outline.length) outline.pop(); else if (mode === "maxy") maxy = null; else corners.pop(); draw(); };
$("clear").onclick = () => { corners = []; outline = []; maxy = null; draw(); };
c.addEventListener("click", (e) => {
  const r = c.getBoundingClientRect(), x = (e.clientX - r.left) * W / r.width, y = (e.clientY - r.top) * H / r.height;
  const p = [Math.round(x * 10) / 10, Math.round(y * 10) / 10];
  if (mode === "corners") { if (corners.length < 4) corners.push(p); if (corners.length === 4) setMode("outline"); }
  else if (mode === "outline") outline.push(p); else maxy = p[1];
  draw();
});
for (const id of ["mw","ml","rd","rh","pol"]) $(id).addEventListener("input", draw);
function dot(p, col, label) { ctx.beginPath(); ctx.arc(p[0], p[1], 6, 0, 7); ctx.fillStyle = col; ctx.fill(); ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
  if (label) { ctx.font = "bold 13px system-ui"; ctx.lineWidth = 3; ctx.strokeStyle = "#fff"; ctx.strokeText(label, p[0] + 9, p[1] - 8); ctx.fillStyle = "#18212b"; ctx.fillText(label, p[0] + 9, p[1] - 8); } }
function draw() {
  ctx.drawImage(img, 0, 0, W, H);
  if (maxy != null) { ctx.fillStyle = "rgba(24,33,43,.45)"; ctx.fillRect(0, maxy, W, H - maxy); }
  if (outline.length) { ctx.beginPath(); outline.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.closePath(); ctx.fillStyle = "rgba(31,111,209,.15)"; ctx.fill(); ctx.strokeStyle = "#1f6fd1"; ctx.lineWidth = 2; ctx.stroke(); outline.forEach((p) => dot(p, "#1f6fd1")); }
  if (corners.length) { ctx.beginPath(); corners.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); if (corners.length === 4) ctx.closePath(); ctx.strokeStyle = "#d23f5b"; ctx.lineWidth = 2.5; ctx.stroke(); corners.forEach((p, i) => dot(p, "#d23f5b", names[i])); }
  [...$("steps").children].forEach((li, i) => { li.className = i < corners.length ? "done" : i === corners.length ? "now" : ""; });
  $("out").value = JSON.stringify(result(), null, 2);
}
function num(id) { const v = parseFloat($(id).value); return isFinite(v) && v > 0 ? v : null; }
function result() {
  return { frame_size: [W, H], floor_corners_px: corners, floor_polygon_px: outline.length >= 3 ? outline : null,
    measured_width_m: num("mw"), measured_length_m: num("ml"), robot_diameter_m: num("rd") || 0.35, robot_height_m: num("rh") || 0.10,
    robot_polarity: $("pol").value, max_y_px: maxy };
}
$("dl").onclick = () => { if (corners.length !== 4) { alert("Tap all four corners first."); return; }
  const a = document.createElement("a"); a.href = URL.createObjectURL(new Blob([JSON.stringify(result(), null, 2)], { type: "application/json" })); a.download = "calib.json"; a.click(); };
$("copy").onclick = () => navigator.clipboard && navigator.clipboard.writeText($("out").value);
img.onload = draw;
</script></body></html>
"""
