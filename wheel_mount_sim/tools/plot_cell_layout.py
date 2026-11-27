"""Plan view of the accumulation end (left side; the right side is its mirror) for the records.
Run: python3 tools/plot_cell_layout.py  -> docs/troubleshooting/media/cell/accum_end_plan.png"""
import sys
from math import cos, radians, sin
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle, Polygon, Rectangle

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import cell_layout as L  # noqa: E402
from process import FASTEN_BASE_X, MOUNT_BASE_X, MOUNT_HUB_X, ROBOT_BASE_Y, LINE_Y  # noqa: E402

from matplotlib import font_manager
for f in font_manager.findSystemFonts():
    if "NotoSansCJK" in f and f.endswith((".ttc", ".otf")):
        font_manager.fontManager.addfont(f)
plt.rcParams["font.family"] = ["Noto Sans CJK JP", "Noto Sans CJK KR", "DejaVu Sans"]
fig, ax = plt.subplots(figsize=(11, 7.5), dpi=130)
y0 = LINE_Y
ax.add_patch(Rectangle((L.FRAME_END_X, y0-.335), 2.6-L.FRAME_END_X, .67, color="#d9dde1", zorder=0))
ax.add_patch(Rectangle((-.05, y0-.335), 2.65, .67, color="#c3c8cd", zorder=0))
ax.text(1.9, y0+.40, "기존 벨트 (끝 x −0.05)", ha="center", fontsize=9, color="#555")
ax.text(-.65, y0+.40, "연장 구간", ha="center", fontsize=9, color="#555")
for x0 in (L.GUIDE_START_X,):
    for e in (-1, 1):
        ax.plot([x0, 2.6], [y0+e*.349]*2, color="#6b7a8a", lw=3)
ax.annotate("측면 가이드 끝\n(조가 지나갈 자리)", (L.GUIDE_START_X, y0-.349), (.35, y0-.95),
            fontsize=8, arrowprops=dict(arrowstyle="->", color="#555"))
for (c, yaw, s) in L.stop_plates(1):
    a = radians(yaw)
    dx, dy = s[0]/2*cos(a), s[0]/2*sin(a)
    ax.plot([c[0]-dx, c[0]+dx], [c[1]-dy, c[1]+dy], color="#e8e2c8", lw=6, solid_capstyle="butt", zorder=3)
    ax.plot([c[0]-dx, c[0]+dx], [c[1]-dy, c[1]+dy], color="#8a8060", lw=1, zorder=4)
ax.add_patch(Rectangle((L.FRAME_END_X, y0-.32), .03, .64, color="#555", zorder=3))
ax.text(L.APEX_X-.08, y0+.02, "V 끝 스토퍼\n(UHMW, 고정)", ha="right", va="center", fontsize=8)
for i, x in enumerate((L.PICK_X, L.PICK_X+2*L.TIRE_R, L.PICK_X+4*L.TIRE_R)):
    ax.add_patch(Circle((x, y0), L.TIRE_R, fc="#2b2b2b" if i == 0 else "#555", ec="k", alpha=.85 if i else .95, zorder=5))
    ax.add_patch(Circle((x, y0), .203, fc="#9aa3ab", ec="#666", zorder=6))
    ax.text(x, y0, ["집는 타이어", "대기 1", "대기 2"][i], ha="center", va="center", fontsize=8, zorder=7)
for jaw in (-18, 0, 18):
    a = radians(90+jaw)
    for s in (-1, 1):
        ax.plot([L.PICK_X+s*.30*cos(a), L.PICK_X+s*.36*cos(a)], [y0+s*.30*sin(a), y0+s*.36*sin(a)],
                color="#1f5fbf" if jaw == 0 else "#8fb0e0", lw=4 if jaw == 0 else 2, zorder=8)
ax.text(L.PICK_X-.55, y0-.50, "조 닫힘 방향: 흐름에 수직 ±18°", color="#1f5fbf", fontsize=8)
ax.annotate("", (L.PICK_X+1.6, y0-.2), (L.PICK_X+2.3, y0-.2), arrowprops=dict(arrowstyle="->", lw=2, color="#2a7"))
ax.text(L.PICK_X+1.95, y0+.42, "흐름 0.32 m/s ←", ha="center", fontsize=8, color="#2a7")
# old pick zone
ax.add_patch(Rectangle((.27-.33, y0-.33), .66, .66, fill=False, ls="--", ec="#c33", lw=1.2, zorder=2))
ax.text(.27, y0-.42, "옛 집기 구역 (제거)", color="#c33", fontsize=8, ha="center", va="top")
# robots
bx, by = MOUNT_BASE_X, ROBOT_BASE_Y
ax.add_patch(Rectangle((bx-.3, by-.3), .6, .6, fc="#f0b429", ec="k", zorder=5))
ax.text(bx, by, "장착로봇\nRS080N", ha="center", va="center", fontsize=8, zorder=6)
ax.add_patch(Rectangle((FASTEN_BASE_X-.3, by-.3), .6, .6, fc="#bbb", ec="k", zorder=5))
ax.text(FASTEN_BASE_X, by, "체결로봇", ha="center", va="center", fontsize=8, zorder=6)
ax.plot([bx, bx], [by, by+1.0], color="#c33", lw=2, ls=":")
ax.text(bx+.05, by+.55, "J1 하드스톱\n(±180°, 바로 뒤)", color="#c33", fontsize=8)
ax.add_patch(Arc((bx, by), 1.5, 1.5, theta1=101, theta2=322, color="#1f5fbf", lw=2.2))
ax.annotate("", (bx+.75*cos(radians(322)), by+.75*sin(radians(322))),
            (bx+.75*cos(radians(330)), by+.75*sin(radians(330))), arrowprops=dict(arrowstyle="<-", color="#1f5fbf", lw=2))
ax.text(bx-1.5, by-.95, "−x 쪽으로 221° 회전\n(체결로봇 반대쪽)", color="#1f5fbf", fontsize=8, ha="left")
ax.plot([bx, L.PICK_X], [by, y0], color="#1f5fbf", lw=1, ls="--")
ax.text(L.PICK_X+.05, y0-.88, "1.39 m", fontsize=8, color="#1f5fbf", ha="right")
# car + hub
ax.add_patch(Rectangle((MOUNT_HUB_X-1.36-.95, -.10), 4.56, .90, fc="#eef2f6", ec="#889", zorder=0))
ax.add_patch(Rectangle((MOUNT_HUB_X-.316, .69), .632, .205, fc="#2b2b2b", zorder=4))
ax.plot(MOUNT_HUB_X, .79, "o", color="#f0b429", zorder=6)
ax.text(MOUNT_HUB_X, .58, f"허브 x {MOUNT_HUB_X:.2f} (항상 같은 자리)", ha="center", fontsize=8)
ax.text(MOUNT_HUB_X+.9, .2, "차 (위치 A, 앞이 +x)", fontsize=8, color="#667")
ax.set_xlim(-1.95, 2.7)
ax.set_ylim(.0, 4.5)
ax.set_aspect("equal")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title(f"벨트 끝 적재식 공급 (왼쪽, 오른쪽은 대칭) — 집는 점 x {L.PICK_X:.2f}, 스토퍼 꼭짓점 x {L.APEX_X:.3f}", fontsize=11)
ax.grid(alpha=.25)
out = HERE/"docs/troubleshooting/media/cell/accum_end_plan.png"
fig.tight_layout()
fig.savefig(out)
print(out)
