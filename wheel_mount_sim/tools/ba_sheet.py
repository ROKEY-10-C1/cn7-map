"""Before/after contact sheet from two replay GIFs (mount_control --replay-video): frames at given sim times.
python3 ba_sheet.py BEFORE.gif AFTER.gif T0 "t1,t2,.." OUT.png "before label" "after label" """
import sys
from PIL import Image, ImageDraw, ImageSequence

a, b, t0, ts, out, la, lb = sys.argv[1:8]
t0, ts = float(t0), [float(v) for v in ts.split(",")]
FPS = 7.5                                    # replay_video writes every 2nd frame of 15 fps into the GIF


def frames(p):
    return [f.convert("RGB") for f in ImageSequence.Iterator(Image.open(p))]


fa, fb = frames(a), frames(b)
w, h = 384, 216
g = Image.new("RGB", (w*len(ts), 2*h+44), "white")
d = ImageDraw.Draw(g)
for i, t in enumerate(ts):
    k = int(round((t-t0)*FPS))
    for row, fr in enumerate((fa, fb)):
        g.paste(fr[min(k, len(fr)-1)].resize((w, h)), (i*w, 22+row*(h+22)))
    d.text((i*w+6, 4), f"{t:.1f} s", fill="black")
d.text((w*len(ts)-260, 4), la, fill="black")
d.text((6, 22+h+4), lb, fill="black")
g.save(out)
print(out)
