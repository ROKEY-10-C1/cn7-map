"""Hand calculation: H2515 + SCHUNK PZN-plus 80-2 centre-bore mandrel handling the CN7 wheel.

Every input is listed with its source (catalogue / model / estimate). No simulation: static and
quasi-static load cases with a dynamic factor, compared with the catalogue limits.
Run: python3 map/wheel_mount_sim/analysis/check_mandrel_loads.py (history: H2515 + centre-bore mandrel design)
"""
import sys
from math import atan, cos, degrees, pi, radians, tan
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"wheel_cell"))
from physical_data import WHEEL_MASS, wheel_inertia  # noqa: E402
from process import CN7_R16  # noqa: E402

G = 9.81
# ---- wheel (model / OEM) --------------------------------------------------------------------
M_WHEEL = WHEEL_MASS                       # 17.1 kg: rim 9.1 (wheel-size.com) + tire 8.0 (Continental)
I_T, _, I_AX = wheel_inertia(CN7_R16)      # transverse, axial kg m^2 (model estimate)
BORE_R = .0671/2                           # OEM
WEB = (.046, .060)                         # hub face side / outboard face of the web (wheel z, model)
PILOT_CLEAR = (.0671-.06694)/2             # radial, model
PILOT_LEN = .012
# ---- gripper (SCHUNK catalogue, ID 303411) --------------------------------------------------
F_OPEN = 2490.                             # N, sum of 3 jaws, I.D. gripping uses the opening force
MX, MY, MZ, FZ = 60., 95., 55., 1500.      # per base jaw, static
FINGER_MAX_L, FINGER_MAX_M = .105, .6
# ---- tool stack (design, see MOUNT_GRIPPER_SPEC.md) -----------------------------------------
GRIP_Z = (WEB[1]-.006+WEB[1])/2            # centre of the 6 mm nose contact, wheel z
SHOULDER_R = .040                          # shoulder contact radius on the web front
FINGER_P = .028                            # base-jaw reference to the force point (estimate)
FINGER_M = .06
FLANGE_Z = .1393                           # robot flange, wheel z
TOOL = {  # kg, wheel-z of the part CoG, radial offset
    "PZN-plus 80-2": (.79, .0997, 0.), "fingers": (.18, .068, 0.), "adapter": (.32, .1318, 0.),
    "D405+bracket": (.21, .165, .09), "valve+fittings": (.15, .12, 0.)}
# ---- robot (Doosan manual 3.5.0) -------------------------------------------------------------
DIAGRAM = ((25., .100), (20., .150), (15., .250))   # payload kg -> max CoG distance along flange normal
PRESS_LIMIT = 145.                                  # N, cell insertion force limit (calibration.json)
# ---- assumptions ----------------------------------------------------------------------------
DYN = 1.5          # 1 g + 0.5 g robot acceleration
SF = 2.0           # SCHUNK's own safety factor for force-fit gripping
MUS = (.1, .3, .5)  # SCHUNK catalogue basis / steel on painted alloy (est.) / serrated insert (est.)
J6_ALPHA = radians(180)/.2   # 180 deg/s reached in 0.2 s


def limit_z(mass):
    pts = sorted(DIAGRAM)
    if mass <= pts[0][0]:
        return pts[0][1]
    for (m0, z0), (m1, z1) in zip(pts, pts[1:]):
        if m0 <= mass <= m1:
            return z0+(z1-z0)*(mass-m0)/(m1-m0)
    return 0.


def row(name, need, have, unit, note=""):
    ok = have >= need
    sf = have/need if need else float("inf")
    print(f"  {'OK ' if ok else 'NG '} {name:<44} need {need:8.1f} {unit:<4} have {have:8.1f} {unit:<4} SF {sf:5.2f}  {note}")
    return ok


def main():
    tool_m = sum(m for m, _, _ in TOOL.values())
    tool_z = sum(m*z for m, z, _ in TOOL.values())/tool_m
    total = M_WHEEL+tool_m
    cog = (M_WHEEL*FLANGE_Z+tool_m*(FLANGE_Z-tool_z))/total       # from the flange, along its normal
    xy = sum(m*r for m, _, r in TOOL.values())/total
    W = M_WHEEL*G
    print(f"wheel {M_WHEEL:.2f} kg (I axial {I_AX:.3f}, transverse {I_T:.3f} kg m2), tool {tool_m:.2f} kg")
    print(f"payload {total:.2f} kg, CoG {cog*1000:.0f} mm along the flange normal, {xy*1000:.1f} mm radial\n")

    print("1) Robot H2515 payload diagram")
    lim = limit_z(total)
    row("CoG distance allowed at this mass (mm)", cog*1000, lim*1000, "mm")
    i_flange = I_T+M_WHEEL*FLANGE_Z**2
    print(f"     inertia about the wrist (transverse) {i_flange:.2f} kg m2, about J6 {I_AX:.2f} kg m2"
          " - H2515 limits not published (H2017: J5 4.5, J6 2.0)\n")

    print("2) Jaw forces when the wheel axis is horizontal (mounting pose)")
    n_top = (F_OPEN+2*W*DYN)/3
    row("top jaw radial force vs catalogue moment", n_top*FINGER_P, MX, "Nm", f"N_top {n_top:.0f} N x P {FINGER_P*1000:.0f} mm")
    lever = WEB[1]-GRIP_Z+(GRIP_Z-0.)                      # CoG (wheel centre, z 0) to the grip plane
    pitch = W*DYN*(GRIP_Z-0.)
    need_ax = pitch/(BORE_R+SHOULDER_R)
    print(f"     pitch moment {pitch:.1f} Nm (CoG {GRIP_Z*1000:.0f} mm inboard of the grip), "
          f"top jaw must hold {need_ax:.0f} N axially")
    for mu in MUS:
        row(f"top jaw axial friction, mu {mu}", need_ax*SF, mu*n_top, "N")
    print()

    print("3) Picking the wheel lying flat (axis vertical, wheel hangs below the jaws)")
    for mu in MUS:
        row(f"axial friction of 3 jaws, mu {mu}", W*DYN*SF, mu*F_OPEN, "N")
    row("axial load per jaw vs Fz", W*DYN/3, FZ, "N")
    print()

    print("4) Turning the wheel about its axis with J6 (phase alignment)")
    t_need = I_AX*J6_ALPHA
    for mu in MUS:
        row(f"friction torque, mu {mu}", t_need*SF, mu*F_OPEN*BORE_R, "Nm", f"alpha {degrees(J6_ALPHA):.0f} deg/s2")
    print()

    print("5) Pushing onto the hub")
    row("press force per jaw vs Fz", PRESS_LIMIT/3, FZ, "N")
    row("finger mass", FINGER_M, FINGER_MAX_M, "kg")
    row("finger length", FINGER_P*1000, FINGER_MAX_L*1000, "mm")
    print()

    print("6) After release at d = 6 mm (wheel hangs on the studs, pilot 6 mm into the bore)")
    engage = PILOT_LEN-.006
    tilt = degrees(atan(2*PILOT_CLEAR/engage))
    print(f"     lean limited by the pilot: <= {tilt:.2f} deg (0.08 mm clearance over {engage*1000:.0f} mm)")

    print()
    print("7) Same cases with a pull-down nose: 45 deg flank on the bore's outboard edge chamfer (estimate)")
    mu = .3

    pre = F_OPEN*tan(radians(45)-atan(mu))      # axial clamp of the web against the shoulder
    print(f"     clamp preload {pre:.0f} N (wedge with mu {mu})")
    row("pitch moment, mounting pose", pitch*SF, pre*SHOULDER_R, "Nm")
    row("wheel hanging (flat pick): radial force to hold wedge", W*DYN*SF, F_OPEN, "N")
    n_flank = F_OPEN/cos(radians(45))
    t_have = mu*pre*SHOULDER_R+mu*n_flank*BORE_R
    row("J6 phase torque", t_need*SF, t_have, "Nm")

    print()
    print("8) No chamfer assumed: 5 support pads on the web front between the lug holes (r 70 mm),")
    print("   fixed to the gripper body, plus slower J6 phase moves (450 deg/s2)")
    pad_r = .070
    need_pad = pitch/(BORE_R+pad_r)
    for mu in (.3, .5):
        row(f"pitch moment, top jaw friction, mu {mu}", need_pad*SF, mu*n_top, "N")
        row(f"flat pick, 3 jaws, mu {mu}", W*DYN*SF, mu*F_OPEN, "N")
        row(f"J6 phase torque at 450 deg/s2, mu {mu}", I_AX*J6_ALPHA/2*SF, mu*F_OPEN*BORE_R, "Nm")

    print()
    print("9) Industry-standard alternative: claws on the tire tread (Mitsubishi US4942659, Honda US9757828,")
    print("   Quatro US5125298). Lightest plausible tool, same H2515 diagram")
    mu_pad = .35                                   # calibration.json pad_tire_friction (SCHUNK HKI range)
    need_force = W*DYN*SF/mu_pad                   # sum of claw normal forces, axis horizontal
    print(f"     claws must press the tread with >= {need_force:.0f} N in total (mu {mu_pad})")
    claw_tool = {"3-jaw long-stroke gripper (>=1.4 kN, ~15 mm/jaw), est.": (6.0, .055),
                 "3 L-claws reaching r 0.33 m, est.": (1.8, -.06), "adapter + camera": (.7, .11)}
    plate_z = CN7_R16.width/2+.012                 # gripper face just outboard of the sidewall bulge
    tm = sum(m for m, _ in claw_tool.values())
    flange = plate_z+.12
    tz = sum(m*(plate_z+z) for m, z in claw_tool.values())/tm
    tot = M_WHEEL+tm
    cg = (M_WHEEL*flange+tm*(flange-tz))/tot
    print(f"     tool {tm:.1f} kg, payload {tot:.1f} kg, CoG {cg*1000:.0f} mm")
    row("H2515 rated payload", tot, 25., "kg")
    row("H2515 CoG distance at this mass", cg*1000, limit_z(min(tot, 25.))*1000, "mm")


if __name__ == "__main__":
    main()
