"""Figures for the paper, from the simulated verdict (motionbench game verdict ... -o verdict/).

    python paper/make_figs.py verdict/verdict.json
"""
import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

v = json.load(open(sys.argv[1]))
P, U = v["public"], v["unseen"]
names = list(P["code"])
best = max(P["drivers"], key=lambda d: P["drivers"][d]["mean_points"])
labels = [n.replace(".py", "").replace("_", " ") for n in names] + ["typical simulated\ndriver", f"best simulated\ndriver ({P['drivers'][best]['name']})"]
pub = [P["code"][n]["mean_points"] for n in names] + [P["typical_student"]["mean_points"], P["drivers"][best]["mean_points"]]
uns = [U["code"][n]["mean_points"] for n in names] + [U["typical_student"]["mean_points"], U["drivers"][best]["mean_points"]]
fig, ax = plt.subplots(figsize=(6.4, 2.6), dpi=200)
x = range(len(labels))
w = 0.38
ax.bar([i - w / 2 for i in x], pub, w, label="public layout (dev-01)", color="#1F6FD1")
ax.bar([i + w / 2 for i in x], uns, w, label="10 unseen layouts (generated)", color="#D98316")
ax.set_xticks(list(x))
ax.set_xticklabels(labels, fontsize=7)
ax.set_ylabel("points (of 6)", fontsize=8)
ax.set_ylim(0, 6)
ax.axvline(3.5, color="#999", lw=0.8, ls=":")
ax.tick_params(axis="y", labelsize=7)
ax.spines[["top", "right"]].set_visible(False)
ax.legend(fontsize=7, frameon=False, loc="upper left")
fig.tight_layout()
fig.savefig("figs/verdict-unseen.pdf")
print("wrote figs/verdict-unseen.pdf")
