"""Physical parameters of the wheel-mount simulation, each with its source.

status:
  catalogue  - manufacturer data sheet
  oem        - Hyundai OEM wheel data confirmed by several sellers
  literature - measured values from a published study or standard practice
  derived    - computed here from catalogue/literature values (formula given)
  model      - geometry inherited from the CN7 cell model (map/wheel_cell), not yet measured
  estimate   - no data found; value chosen with the reason given and covered by a sensitivity sweep
All lengths in metres, masses in kg, forces in N.
"""
from math import pi, radians

P = {}


def p(name, value, unit, status, source):
    P[name] = dict(value=value, unit=unit, status=status, source=source)
    return value


# ---------------------------------------------------------------- scene
TABLE_TOP = p("table_top_z", .50, "m", "model", "cell conveyor belt height BELT_Z (map/wheel_cell/process.py)")
HUB_Z = p("hub_z", .94, "m", "model", "cell hub height with the car on the hanger (process.HUB_Z)")

# ---------------------------------------------------------------- wheel (CN7 16x6.5J ET46)
TIRE_R = p("tire_radius", .31595, "m", "derived", "205/55 R16: 16*25.4/2 + 0.205*0.55")
TIRE_HALF_W = p("tire_half_width_bulge", .1029, "m", "model", "widest sidewall of the cell tire mesh")
RIM_R = p("rim_bead_radius", .2032, "m", "derived", "16 in bead seat diameter / 2")
RIM_W = p("rim_width", .1651, "m", "oem", "6.5J")
OFFSET = p("offset_ET", .046, "m", "oem", "ET46 (52910-AB200/AA230)")
BORE = p("centre_bore", .0671, "m", "oem", "67.1 mm")
PCD = p("pcd", .1143, "m", "oem", "5 x 114.3")
LUG_HOLE = p("lug_hole_diameter", .0134, "m", "model", "cell value; no public CN7 drawing found")
BORE_CHAMFER = p("bore_hub_side_chamfer", .0015, "m", "estimate",
                 "hub-side bore edge is chamfered/relieved to clear the pilot root (wheel-fitment guides); 1.5 x 45 deg assumed")
WEB_T = p("mounting_pad_thickness", .014, "m", "model", "cell value; worst case for bore length")
RIM_MASS = p("rim_mass", 9.1, "kg", "literature", "wheel-size.com listing for 6.5Jx16 ET46 CN7 (single source)")
TIRE_MASS = p("tire_mass", 8.0, "kg", "catalogue", "Continental ProContact RX 205/55R16 (cell physical_data)")

# ---------------------------------------------------------------- tire compliance (rigid-ring model)
TIRE_K_RADIAL = p("tire_k_radial", 180e3, "N/m", "literature",
                  "average radial stiffness of passenger tyres, Eksploatacja i Niezawodnosc 2014 (archive.ein.org.pl 2014-01-11)")
TIRE_K_CIRC = p("tire_k_circumferential", 80e3, "N/m", "literature", "same study, circumferential")
TIRE_K_LAT = p("tire_k_lateral", 65e3, "N/m", "literature", "same study, lateral")
TIRE_K_TORSION = p("tire_k_torsion", TIRE_K_CIRC*TIRE_R**2, "N m/rad", "derived",
                   "k_circ * R^2 = 7.99 kN m/rad = 139 N m/deg (study reports ~150, consistent)")
TIRE_K_TILT = p("tire_k_tilt", TIRE_K_LAT*TIRE_R**2/2, "N m/rad", "derived",
                "lateral springs spread round the bead: k_lat * R^2 / 2")
TIRE_ZETA = p("tire_damping_ratio", .08, "-", "estimate",
              "passenger tyre structural damping is a few per cent of critical; 0.05-0.10 swept")

# ---------------------------------------------------------------- hub corner (car lifted, wheel at full droop)
HUB_MASS = p("hub_mass", 6.5, "kg", "estimate", "hub flange + rotor hat + inner race; cell value")
BEARING_DRAG = p("bearing_drag", .75, "N m", "literature",
                 "hub-unit design limit <=0.75 N m with two seals (US patent 'wheel hub assembly having low friction')")
BEARING_PLAY = p("bearing_radial_play", 0., "m", "literature", "generation-3 hub units are preloaded (no radial play)")
CURB = 1265.
CORNER_SPRUNG = CURB*.61/2-40.
RIDE_F = 1.3
WHEEL_RATE = p("suspension_wheel_rate", (2*pi*RIDE_F)**2*CORNER_SPRUNG, "N/m", "derived",
               "(2 pi f)^2 m: f 1.3 Hz (passenger car front ride frequency 1.0-1.5 Hz), m = 1265 kg * 0.61 / 2 - 40 kg unsprung")
DROOP_PRELOAD = p("rebound_stop_preload", 300., "N", "estimate",
                  "corner hangs on the rebound stop; only an upward push above this moves the hub")
PILOT = p("hub_pilot_diameter", .06694, "m", "model", "cell value, 0.08 mm radial clearance in the bore")
PILOT_LEN = p("hub_pilot_length", .012, "m", "model", "cell value")
STUD_D = p("stud_diameter", .012, "m", "oem", "M12 x 1.5")
STUD_PROJ = p("stud_projection", .030, "m", "model", "cell value")
FLANGE_R = p("hub_flange_radius", .069, "m", "literature", "aftermarket CN7-family hub flange OD ~138 mm (Bunnings listing)")
ROTOR = p("front_rotor_radius_thickness", (.140, .025), "m", "model", "cell value (280 x 25 mm ventilated)")
ARCH_R = p("wheelhouse_arch_radius", .365, "m", "model", "cell wheelhouse liner datum from the body mesh")
ARCH_SPAN = p("wheelhouse_arch_span_deg", (-9.9, 189.9), "deg", "model", "cell wheelhouse liner")
ARCH_Z = p("wheelhouse_axial_extent", (-.119, .113), "m", "model", "liner across the wheel plane")

# ---------------------------------------------------------------- friction
MU_SHOE_TREAD = p("mu_shoe_tread", (.6, .5), "static,dynamic", "estimate",
                  "polyurethane on rubber: industry range 0.2-2.5, higher for soft grades (Gallagher, Mearthane); "
                  "SCHUNK pad basis 0.3-0.4 as the low case. Swept 0.35-0.9")
MU_TREAD_TABLE = p("mu_tread_steel", (.7, .55), "static,dynamic", "literature",
                   "rubber on dry steel 0.6-0.8 (Deladi, rubber-metal static friction thesis, U Twente)")
MU_ALU_STEEL = p("mu_alloy_steel", (.61, .47), "static,dynamic", "literature",
                 "aluminium on mild steel, dry (Engineering ToolBox friction table); painted/zinc surfaces swept down to 0.3")

# ---------------------------------------------------------------- gripper SCHUNK PHL-W 63-080-S (ID 308273)
GRIP = dict(stroke=p("grip_stroke_per_jaw", .080, "m", "catalogue", "SCHUNK PHL-W 63-080-S"),
            force=p("grip_closing_force_sum", 4630., "N", "catalogue", "sum of both jaws"),
            spring=p("grip_spring_force_min", 1530., "N", "catalogue", "force maintenance on air loss"),
            mass=p("grip_mass", 18.7, "kg", "catalogue", "SCHUNK"),
            size=p("grip_size_xyz", (.541, .164, .1287), "m", "catalogue", "length X, width Y, height Z"),
            t_close=p("grip_closing_time", 1.09, "s", "catalogue", "base jaws only"),
            t_open=p("grip_opening_time", 1.74, "s", "catalogue", "base jaws only"),
            finger_max=p("grip_finger_max", (.72, 12.), "m,kg", "catalogue", "length, weight per finger"),
            moments=p("grip_jaw_moments", (180., 1080., 417., 10000.), "Nm,Nm,Nm,N", "catalogue", "Mx My Mz Fz per jaw"))
SHOE_ANGLE = p("shoe_angle_from_bottom", radians(80.), "rad", "derived",
               "parallel jaws squeezing the tread below its centre push the wheel out of the jaws unless "
               "mu >= tan(theta), theta = angle below horizontal (same condition for the flat pick). SF 3 at mu 0.6 "
               "-> theta <= 11.3 deg -> 10 deg below horizontal. The shoes (r <= 334 mm) stay inside the arch radius "
               "365 mm; fingers pass under the arch end (-9.9 deg). Tested: 45 deg squeezed out on the table (dbg5), "
               "20 deg rose 27 mm in the jaws while standing the wheel up and hit the caliper (dbg8/9)")
SHOE_ARC = p("shoe_arc", radians(18.), "rad", "estimate", "PU shoe, 70 Shore A, 10 mm thick; top edge stays under horizontal+1 deg")
ALU_DENSITY = p("aluminium_density", 2700., "kg/m3", "literature", "6082 aluminium")

# ---------------------------------------------------------------- robot Kawasaki RS080N
ROBOT_PAYLOAD = p("robot_payload", 80., "kg", "catalogue", "Kawasaki RS080N")
ROBOT_WRIST = p("robot_wrist_moment", (336., 336., 194.), "Nm", "catalogue", "JT4/JT5/JT6 allowable moment (Kawasaki RS080N page)")
ROBOT_URDF = p("robot_urdf", "isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/Kawasaki/rs080n",
               "-", "catalogue", "Isaac Sim 5.1 Kawasaki RS080N URDF + Lula description (joint limits/speeds)")

# ---------------------------------------------------------------- solver
PHYSICS_HZ = p("physics_hz", 240, "Hz", "model", "cell value; 480 Hz tried (dbg44): no better at the pilot, twice the cost")
CONTACT_OFFSET = p("hub_pin_contact_offset", .0008, "m", "model", "cell value on the hub studs (validated insertion bench)")
FACE_CONTACT_OFFSET = p("hub_face_contact_offset", .0002, "m", "derived",
                        "~5 x travel per step at 10 mm/s; 0.8 mm brought hundreds of pad triangles into contact at once "
                        "and the seat blew up (dbg40)")
STUD_CONTACT_OFFSET = p("stud_contact_offset", .0003, "m", "derived",
                        "> 3 x travel per step at 20 mm/s (0.083 mm) and stud+pad offsets (0.345 mm) < stud-hole clearance 0.7 mm; "
                        "0.8 mm created false axial contacts at the hole edges (sim dbg27)")
PILOT_CONTACT_OFFSET = p("pilot_contact_offset", .0008, "m", "derived",
                         ">= 2.5 x travel per step of the cell insertion bench (76 mm/s at 240 Hz = 0.32 mm); 0.2 mm let the "
                         "pilot jam at 8.9 mm on that bench. 0.02 mm tunnelled on release (dbg47). False edge contacts at "
                         "0.8 mm came from the native cylinder (dbg39/45), not from this polygon prism")
PAD_CONTACT_OFFSET = p("pad_contact_offset", .000045, "m", "model", "cell value on the wheel pad: below the 0.08 mm pilot clearance")


def table():
    rows = ["| 항목 | 값 | 단위 | 근거 종류 | 출처·계산 |", "|---|---|---|---|---|"]
    for k, v in P.items():
        val = v["value"]
        if isinstance(val, float):
            val = f"{val:.6g}"
        rows.append(f"| {k} | {val} | {v['unit']} | {v['status']} | {v['source']} |")
    return "\n".join(rows)


if __name__ == "__main__":
    print(table())

# ---------------------------------------------------------------- insertion control (robot force control option)
PRESS_FORCE = p("press_force_setpoint", 145., "N", "model", "cell press force limit (calibration.json press_force_limit)")
LATERAL_COMPLIANCE = p("lateral_compliance", 25e3, "N/m", "catalogue",
                       "SCHUNK AGE-F 080 XY compensation spring rate 25 N/mm (+-5 mm); emulated by force control "
                       "because the gripper+wheel (~40 kg) exceeds the unit's 32 kg rating")
SEAT_FORCE_RATIO = p("seat_force_ratio", .9, "-", "estimate", "seated when the axial force holds >= 90 % of the set point ...")
SEAT_STILL = p("seat_still", (.3, .00002), "s, m", "estimate", "... and the wheel moves < 0.02 mm in 0.3 s (force-displacement window)")
RELEASE_FORCE = p("release_hold_force", 20., "N", "estimate", "press force is ramped down to this before the jaws open")
MOUNTED = p("mounted_criteria", (.002, 2.), "m, deg", "derived",
            "after release the wheel hangs on pilot + studs: gap < 2 mm keeps the pilot (12 mm) engaged, lean < 2 deg; "
            "the fastening robot's nuts then draw it flat (11/26 release test: 1.45 deg lean)")
