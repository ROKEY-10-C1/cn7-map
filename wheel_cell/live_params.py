"""Constants of the live cell: paths, rate, replay timing, terminal phases."""

from math import degrees


from fitment import LUG_HOLE, PCD, STUD_DIAMETER
from process import schedule
from cell_config import CONFIG


ROOT = "/World/Live"
REFERENCE = "/World/Orders/CN7_001"
HZ = CONFIG["physics"]["hz"]
HOLE_ANGLE = degrees((LUG_HOLE-STUD_DIAMETER)/PCD)  # wheel turn the stud-hole clearance allows (~0.7 deg)
DOCK_READY = 20.                  # live AMR runs the 7 m gate -> dock leg and lifts by ~15 s
SOURCE_POSE = 30.5                # seconds on the mechanical preview stage
MIN_RAIL_SPACING = 1.55           # meters between carriage origins, including braking clearance
_FIRST_ORDER = next(schedule(False))
FRONT_SEAT = _FIRST_ORDER[5]-SOURCE_POSE   # task seconds at which the replay seats the front wheels
REAR_SEAT = _FIRST_ORDER[6]-SOURCE_POSE
GUIDE_LEAD = 2.2                  # replay: mount tool reaches the wheel-side standby before seating
GUIDE_EXIT = 1.35                 # replay: mount tool is back at its safe pose after seating
GUIDE_RETREAT = 2.0               # live: joint-space withdrawal from standby to the safe pose
TERMINAL_PHASES = frozenset(("pick_ready","feed_failed","interlocked","interference","dock_lost"))
# A lost dock can shift the car; no station may continue after it.
INTERLOCK_FAULTS = frozenset(("dock_lost",))
