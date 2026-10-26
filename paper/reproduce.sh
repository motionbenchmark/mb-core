#!/usr/bin/env bash
# Reproduce every number and figure in the paper.
#
#   bash paper/reproduce.sh          # boards, verdict, index sensitivity, capture accuracy (~5 min)
#   bash paper/reproduce.sh --full   # also re-render and re-track the three simulated matches (~15 min)
#
# Writes paper/results/ and paper/figs/verdict-unseen.pdf, records the environment in
# paper/results/environment.txt, and checks outputs against paper/results/SHA256SUMS.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=paper/results
mkdir -p "$OUT"
R=games/aim-sort/routines

{
  echo "motionbench commit: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "motionbench: $(motionbench --version)"
  python3 -c "import sys, numpy, scipy, cv2; print('python', sys.version.split()[0]); print('numpy', numpy.__version__); print('scipy', scipy.__version__); print('opencv', cv2.__version__)"
  echo "ffmpeg: $(ffmpeg -version 2>/dev/null | head -1 || echo none)"
  echo "rustc: $(rustc --version 2>/dev/null || echo none)"
} > "$OUT/environment.txt"
cat "$OUT/environment.txt"

echo "== 1. synthetic boards (scorer)"
motionbench aggregate data/synthetic-v0.1 -o "$OUT/leaderboard-synthetic.json"
cmp -s "$OUT/leaderboard-synthetic.json" results/leaderboard-synthetic-v0.1.json \
  && echo "   identical to the published board" || echo "   WARNING: differs from results/leaderboard-synthetic-v0.1.json"

echo "== 2. AIM sorting verdict (public and 10 unseen layouts, 20 paired runs)"
motionbench game verdict games/aim-sort --layout dev-01 \
  --code $R/deadreckon_v1.py --code $R/vision_v2.py --code $R/vision_tags_v3.py --code $R/planner_v4.py \
  -o "$OUT/verdict" > "$OUT/verdict.txt"
cp "$OUT/verdict/verdict.json" "$OUT/aim-sort-verdict.json"
rm -rf "$OUT/verdict"

echo "== 2b. the unseen-layout runs, layout by layout"
python3 paper/analysis/per_layout.py

echo "== 3. index weight sensitivity"
python3 paper/analysis/index_sensitivity.py

echo "== 4. capture accuracy on rendered video"
python3 paper/analysis/capture_accuracy.py

echo "== 5. figures"
(cd paper && python3 make_figs.py results/aim-sort-verdict.json)

if [[ "${1:-}" == "--full" ]]; then
  echo "== 6. match tracking benchmark"
  for s in 1 2 3; do
    m=$(printf m%04d $s)
    motionbench match render data/match-sim-v0.1/$m -o "$OUT/match/$m.mp4" --fps 10 --seed $s > /dev/null
    motionbench match track "$OUT/match/$m.mp4" -o "$OUT/match/$m.tracks.csv" --fps 10 > /dev/null
    motionbench match eval data/match-sim-v0.1/$m "$OUT/match/$m.tracks.csv" --json > "$OUT/match/$m.eval.json"
  done
fi

echo "== checksums (deterministic outputs)"
(cd "$OUT" && sha256sum -c SHA256SUMS) || echo "   (capture accuracy can vary slightly with OpenCV/FFmpeg versions; the scorer and simulator outputs should match exactly)"
