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
NICE = {"deadreckon_v1.py": "dead\nreckoning", "vision_v2.py": "vision", "vision_tags_v3.py": "vision\n+ tags", "planner_v4.py": "planner"}
labels = [NICE.get(n, n.replace(".py", "")) for n in names] + ["typical\nsim. driver", f"best sim. driver\n({P['drivers'][best]['name']})"]
pub = [P["code"][n]["mean_points"] for n in names] + [P["typical_student"]["mean_points"], P["drivers"][best]["mean_points"]]
uns = [U["code"][n]["mean_points"] for n in names] + [U["typical_student"]["mean_points"], U["drivers"][best]["mean_points"]]
fig, ax = plt.subplots(figsize=(6.5, 2.9), dpi=200)
x = range(len(labels))
w = 0.38
ax.bar([i - w / 2 for i in x], pub, w, label="public layout (dev-01)", color="#1F6FD1")
ax.bar([i + w / 2 for i in x], uns, w, label="10 unseen layouts (generated)", color="#D98316")
ax.set_xticks(list(x))
ax.set_xticklabels(labels, fontsize=9)
ax.set_ylabel("mean points (of 6)", fontsize=9)
ax.set_ylim(0, 6)
ax.axvline(3.5, color="#999", lw=0.8, ls=":")
ax.tick_params(axis="y", labelsize=9)
ax.spines[["top", "right"]].set_visible(False)
ax.legend(fontsize=9, frameon=False, loc="upper left", ncol=2)
for i, (a, b) in enumerate(zip(pub, uns)):
    ax.text(i - w / 2, a + 0.08, f"{a:.2f}", ha="center", fontsize=7)
    ax.text(i + w / 2, b + 0.08, f"{b:.2f}", ha="center", fontsize=7)
fig.tight_layout()
fig.savefig("figs/verdict-unseen.pdf")
print("wrote figs/verdict-unseen.pdf")
