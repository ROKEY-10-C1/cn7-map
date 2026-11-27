"""Build the presentation media from output/scenarios/: for each record a 'before (failure) / after (fix)'
key-frame strip with captions, and short GIFs of both. Writes docs/troubleshooting/media/sim/.
Run after tools/render_scenarios.sh:  python3 wheel_mount_sim/tools/make_media.py
"""
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parents[1]
SRC = HERE/"output/scenarios"
DST = HERE/"docs/troubleshooting/media/sim"
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
GIF_FPS = 7.5          # the run GIFs keep every 2nd frame of 15 fps

RECORDS = [
    dict(key="A_flat_grip", title="A. 누운 바퀴 집기: 슈 각도", cam="pick", window=(6.0, 11.0),
         before=("shoe45", "슈 바닥 ±45°: 조이면 바퀴가 V 사이로 빠져나감"),
         after=("nominal", "슈 수평 아래 10°: 들어 올려도 미끄러짐 0.02 mm")),
    dict(key="B_stand_up", title="B. 바퀴 세우기: 슈 각도", cam="wide", window=(11.0, 17.0),
         before=("shoe20", "슈 수평 아래 20°: 세우는 도중 바퀴가 빠져 떨어짐"),
         after=("nominal", "수평 아래 10°: 세운 뒤 허브 앞 도착, 기울기 0.01°")),
    dict(key="C_jaw_sync", title="C. 두 조 동기화", cam="hub", window=(16.5, 19.5),
         before=("nosync", "두 조 따로 구동: 정렬 후에도 바퀴 중심 21 mm 어긋남"),
         after=("nominal", "랙·피니언 동기(미믹 관절): 정렬 오차 0.06 → 0.03 mm")),
    dict(key="D_transfer_path", title="D. 허브까지 이동 경로", cam="wide", window=(11.0, 15.5),
         before=("joint_transfer", "관절 공간 한 번에 이동: 휠하우스 쪽을 지나며 바퀴 27° 비틀림"),
         after=("nominal", "허브 축 바깥 경유점 → 축 따라 직선 진입: 충돌 0")),
    dict(key="E_vision_align", title="E. 허브 앞 비전 정렬", cam="hub", window=(16.5, 20.5),
         before=("novision", "보정 없음: 구멍-스터드 어긋남 0.35~0.48 mm"),
         after=("nominal", "비전 보정 2회: 0.03 mm, 기울기 0.009°, 위상 0.0°")),
    dict(key="F_insertion", title="F. 삽입: 위치 제어 vs 힘 제어", cam="hub", window=(23.5, 29.0),
         before=("position_insert", "위치 제어: 간격 9.6 mm에서 3.5° 기울어 걸림"),
         after=("nominal", "힘 제어(F/T 145 N): 안착 판정 도달, 직후 기울기 6° — 진행 중")),
]


def frames(name, cam, window, step=1):
    g = Image.open(SRC/name/f"{cam}.gif")
    out = []
    for i in range(int(window[0]*GIF_FPS), min(int(window[1]*GIF_FPS), g.n_frames), step):
        g.seek(i)
        out.append(g.convert("RGB").copy())
    return out


def label(im, text, colour):
    im = im.copy()
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT, 15)
    d.rectangle((0, 0, im.width, 26), fill=(0, 0, 0))
    d.text((8, 3), text, font=f, fill=colour)
    return im


def strip(rec):
    rows = []
    for which, colour in (("before", (255, 110, 90)), ("after", (120, 230, 140))):
        name, text = rec[which]
        fr = frames(name, rec["cam"], rec["window"])
        pick = [fr[int(i*(len(fr)-1)/4)] for i in range(5)] if fr else []
        row = Image.new("RGB", (480*5, 270))
        for k, im in enumerate(pick):
            row.paste(im.resize((480, 270)), (k*480, 0))
        rows.append(label(row, ("실패  " if which == "before" else "수정 후  ")+text, colour))
    head = Image.new("RGB", (480*5, 40), (24, 24, 28))
    ImageDraw.Draw(head).text((10, 5), rec["title"], font=ImageFont.truetype(FONT, 22), fill=(255, 255, 255))
    out = Image.new("RGB", (480*5, 40+270*2))
    out.paste(head, (0, 0))
    out.paste(rows[0], (0, 40))
    out.paste(rows[1], (0, 310))
    out.save(DST/f"{rec['key']}.png")


def gif(rec, which):
    name, text = rec[which]
    fr = frames(name, rec["cam"], rec["window"])
    if not fr:
        return
    fr = [label(im.resize((400, 225)), text[:34], (255, 110, 90) if which == "before" else (120, 230, 140)) for im in fr]
    fr[0].save(DST/f"{rec['key']}_{which}.gif", save_all=True, append_images=fr[1:], duration=int(1000/GIF_FPS), loop=0,
               optimize=True)


def main():
    DST.mkdir(parents=True, exist_ok=True)
    for rec in RECORDS:
        strip(rec)
        gif(rec, "before")
        gif(rec, "after")
    summary = {n: json.load(open(SRC/n/"summary.json")) for n in
               {r[w][0] for r in RECORDS for w in ("before", "after")}}
    (DST/"scenario_events.json").write_text(json.dumps(
        {n: dict(result=s["result"], events=s["events"], peak_forces_N=s["peak_forces_N"]) for n, s in summary.items()},
        indent=1, ensure_ascii=False))
    print("written", DST)


if __name__ == "__main__":
    main()
