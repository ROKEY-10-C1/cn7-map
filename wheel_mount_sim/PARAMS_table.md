| 항목 | 값 | 단위 | 근거 종류 | 출처·계산 |
|---|---|---|---|---|
| table_top_z | 0.5 | m | model | cell conveyor belt height BELT_Z (map/wheel_cell/process.py) |
| hub_z | 0.94 | m | model | cell hub height with the car on the hanger (process.HUB_Z) |
| tire_radius | 0.31595 | m | derived | 205/55 R16: 16*25.4/2 + 0.205*0.55 |
| tire_half_width_bulge | 0.1029 | m | model | widest sidewall of the cell tire mesh |
| rim_bead_radius | 0.2032 | m | derived | 16 in bead seat diameter / 2 |
| rim_width | 0.1651 | m | oem | 6.5J |
| offset_ET | 0.046 | m | oem | ET46 (52910-AB200/AA230) |
| centre_bore | 0.0671 | m | oem | 67.1 mm |
| pcd | 0.1143 | m | oem | 5 x 114.3 |
| lug_hole_diameter | 0.0134 | m | model | cell value; no public CN7 drawing found |
| bore_hub_side_chamfer | 0.0015 | m | estimate | hub-side bore edge is chamfered/relieved to clear the pilot root (wheel-fitment guides); 1.5 x 45 deg assumed |
| mounting_pad_thickness | 0.014 | m | model | cell value; worst case for bore length |
| rim_mass | 9.1 | kg | literature | wheel-size.com listing for 6.5Jx16 ET46 CN7 (single source) |
| tire_mass | 8 | kg | catalogue | Continental ProContact RX 205/55R16 (cell physical_data) |
| tire_k_radial | 180000 | N/m | literature | average radial stiffness of passenger tyres, Eksploatacja i Niezawodnosc 2014 (archive.ein.org.pl 2014-01-11) |
| tire_k_circumferential | 80000 | N/m | literature | same study, circumferential |
| tire_k_lateral | 65000 | N/m | literature | same study, lateral |
| tire_k_torsion | 7985.95 | N m/rad | derived | k_circ * R^2 = 7.99 kN m/rad = 139 N m/deg (study reports ~150, consistent) |
| tire_k_tilt | 3244.29 | N m/rad | derived | lateral springs spread round the bead: k_lat * R^2 / 2 |
| tire_damping_ratio | 0.08 | - | estimate | passenger tyre structural damping is a few per cent of critical; 0.05-0.10 swept |
| hub_mass | 6.5 | kg | estimate | hub flange + rotor hat + inner race; cell value |
| bearing_drag | 0.75 | N m | literature | hub-unit design limit <=0.75 N m with two seals (US patent 'wheel hub assembly having low friction') |
| bearing_radial_play | 0 | m | literature | generation-3 hub units are preloaded (no radial play) |
| suspension_wheel_rate | 23072.9 | N/m | derived | (2 pi f)^2 m: f 1.3 Hz (passenger car front ride frequency 1.0-1.5 Hz), m = 1265 kg * 0.61 / 2 - 40 kg unsprung |
| rebound_stop_preload | 300 | N | estimate | corner hangs on the rebound stop; only an upward push above this moves the hub |
| hub_pilot_diameter | 0.06694 | m | model | cell value, 0.08 mm radial clearance in the bore |
| hub_pilot_length | 0.012 | m | model | cell value |
| stud_diameter | 0.012 | m | oem | M12 x 1.5 |
| stud_projection | 0.03 | m | model | cell value |
| hub_flange_radius | 0.069 | m | literature | aftermarket CN7-family hub flange OD ~138 mm (Bunnings listing) |
| front_rotor_radius_thickness | (0.14, 0.025) | m | model | cell value (280 x 25 mm ventilated) |
| wheelhouse_arch_radius | 0.365 | m | model | cell wheelhouse liner datum from the body mesh |
| wheelhouse_arch_span_deg | (-9.9, 189.9) | deg | model | cell wheelhouse liner |
| wheelhouse_axial_extent | (-0.119, 0.113) | m | model | liner across the wheel plane |
| mu_shoe_tread | (0.6, 0.5) | static,dynamic | estimate | polyurethane on rubber: industry range 0.2-2.5, higher for soft grades (Gallagher, Mearthane); SCHUNK pad basis 0.3-0.4 as the low case. Swept 0.35-0.9 |
| mu_tread_steel | (0.7, 0.55) | static,dynamic | literature | rubber on dry steel 0.6-0.8 (Deladi, rubber-metal static friction thesis, U Twente) |
| mu_alloy_steel | (0.61, 0.47) | static,dynamic | literature | aluminium on mild steel, dry (Engineering ToolBox friction table); painted/zinc surfaces swept down to 0.3 |
| grip_stroke_per_jaw | 0.08 | m | catalogue | SCHUNK PHL-W 63-080-S |
| grip_closing_force_sum | 4630 | N | catalogue | sum of both jaws |
| grip_spring_force_min | 1530 | N | catalogue | force maintenance on air loss |
| grip_mass | 18.7 | kg | catalogue | SCHUNK |
| grip_size_xyz | (0.541, 0.164, 0.1287) | m | catalogue | length X, width Y, height Z |
| grip_closing_time | 1.09 | s | catalogue | base jaws only |
| grip_opening_time | 1.74 | s | catalogue | base jaws only |
| grip_finger_max | (0.72, 12.0) | m,kg | catalogue | length, weight per finger |
| grip_jaw_moments | (180.0, 1080.0, 417.0, 10000.0) | Nm,Nm,Nm,N | catalogue | Mx My Mz Fz per jaw |
| shoe_angle_from_bottom | 1.39626 | rad | derived | parallel jaws squeezing the tread below its centre push the wheel out of the jaws unless mu >= tan(theta), theta = angle below horizontal (same condition for the flat pick). SF 3 at mu 0.6 -> theta <= 11.3 deg -> 10 deg below horizontal. The shoes (r <= 334 mm) stay inside the arch radius 365 mm; fingers pass under the arch end (-9.9 deg). Tested: 45 deg squeezed out on the table (dbg5), 20 deg rose 27 mm in the jaws while standing the wheel up and hit the caliper (dbg8/9) |
| shoe_arc | 0.314159 | rad | estimate | PU shoe, 70 Shore A, 10 mm thick; top edge stays under horizontal+1 deg |
| aluminium_density | 2700 | kg/m3 | literature | 6082 aluminium |
| robot_payload | 80 | kg | catalogue | Kawasaki RS080N |
| robot_wrist_moment | (336.0, 336.0, 194.0) | Nm | catalogue | JT4/JT5/JT6 allowable moment (Kawasaki RS080N page) |
| robot_urdf | isaacsim/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/Kawasaki/rs080n | - | catalogue | Isaac Sim 5.1 Kawasaki RS080N URDF + Lula description (joint limits/speeds) |
| physics_hz | 240 | Hz | model | cell value; 480 Hz tried (dbg44): no better at the pilot, twice the cost |
| hub_pin_contact_offset | 0.0008 | m | model | cell value on the hub studs (validated insertion bench) |
| hub_face_contact_offset | 0.0002 | m | derived | ~5 x travel per step at 10 mm/s; 0.8 mm brought hundreds of pad triangles into contact at once and the seat blew up (dbg40) |
| stud_contact_offset | 0.0003 | m | derived | > 3 x travel per step at 20 mm/s (0.083 mm) and stud+pad offsets (0.345 mm) < stud-hole clearance 0.7 mm; 0.8 mm created false axial contacts at the hole edges (sim dbg27) |
| pilot_contact_offset | 2e-05 | m | derived | pilot+pad offsets (0.065 mm) < polygon bore-pilot clearance (0.072 mm): no contact until the pilot really touches; 0.8 mm (cell value) made the lead-in rim kick the wheel at the chamfer edge (dbg39) |
| pad_contact_offset | 4.5e-05 | m | model | cell value on the wheel pad: below the 0.08 mm pilot clearance |
