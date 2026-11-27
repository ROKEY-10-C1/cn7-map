"""Deterministic JIS schedule in metres, seconds and radians."""
from dataclasses import dataclass
from math import atan, ceil, cos, hypot, pi, sin

from fitment import WHEELBASE

FPS = 24
HUB_Z = 0.94
# Five-spindle nutrunner: one pick of all five nuts, one simultaneous run-down per wheel.
NUT_PICKUP = .90
NUT_ENGAGE = 2.45
NUT_DRIVE = .75
NUT_FINAL = .60                    # synchronized final-torque step after run-down
NUT_CYCLE = NUT_ENGAGE+NUT_DRIVE+NUT_FINAL+.70
AXLE = WHEELBASE/2
INDEX_TRAVEL = 6.0  # seconds to translate the wheelbase between dock positions
STATIONS = {"FL": (AXLE, 1), "FR": (AXLE, -1),
            "RL": (-AXLE, 1), "RR": (-AXLE, -1)}
LINE_Y = 3.80
RAIL_Y = 1.57
FEED = (7.72, 0.)
SPLIT_X = 6.85
REJECT_X = 4.25
REJECT_GATE_HALF = .365
REJECT_CRATE_Y = LINE_Y+1.05
BELT_Z = .50
SPEED = .30
AMR_GATE_X = -7.0
# AMR legs timed to KUKA KMP 1500 limits (crab 0.56 m/s, straight 1.0 m/s) with ~10 % margin,
# plus the 0.05 m/s lift: buffer -> gate (crab), gate -> dock (straight), lift 0.12 m.
AMR_AT_GATE = 20.                 # lateral transfer from the outer buffer
AMR_DOCKED = 8.                    # order start (at the gate) -> docked
AMR_LIFTED = AMR_DOCKED+3.
AMR_OUT_GATE = 10.                 # dock -> gate after departure; gate -> outbound buffer follows
AMR_OUT_BUFFER = AMR_AT_GATE
# RS080N bases on each side of the central dock. The fastening arm sits between axles.
MOUNT_BASE_X = 2.60
FASTEN_BASE_X = 0.
ROBOT_BASE_Y = 2.50
# Keep 1.55 m clear beside the control button (0.21 m) and buffer posts (0.03 m).
CELL_Y = 5.25
AMR_BAY_Y = CELL_Y + .21 + 1.55 + .03 + 2.65
# Vehicle buffers: three slots per side, cars parked across the bay (yaw -90 inbound, +90 outbound).
# The cars rest on four stands at the body ends so the AMR+hanger (rails +/-1.80 m) can crab in under
# the car from the AMR lane; slots are served in order (inbound 1->2->3, outbound 3->2->1).
BUFFER_SLOT_X = (-3.12, 0., 3.12)
BUFFER_STAND_X = 2.05             # car-frame stand position along the body, clear of the hanger rails
BUFFER_STAND_Y = .68
# Car height as the hanger/lift offset (0 = docked). Entry pads clear the parked body by 5 cm and the
# carried body clears the stands by 5 cm when passing an empty slot.
# Entry -0.16 (was -0.12): the live buffer probe found the hanger's side locating block 2 mm into the
# parked car's sill at -0.12 and its pads 5 mm under the wheel-arch edges at -0.14. At -0.16 with the
# locating blocks retracted (60 mm) the pads pass 2.5 cm under the arch edges.
AMR_LIFT_ENTRY = -.16
BUFFER_CAR_Z = -.09
AMR_LIFT_CARRY = -.04
MOUNT_PARK = 3.15
FASTEN_PARK = -3.15


@dataclass(frozen=True)
class Wheel:
    width: float
    aspect: float
    inches: int
    rim_inches: float

    @property
    def inner(self):
        return self.inches * 0.0254 / 2

    @property
    def radius(self):
        return self.inner + self.width * self.aspect

    @property
    def rim_width(self):
        return self.rim_inches*.0254

    @property
    def name(self):
        return f"{round(self.width * 1000)}/{round(self.aspect * 100)} R{self.inches}"


CN7_R16 = Wheel(.205, .55, 16, 6.5)
WHEELS = (CN7_R16, Wheel(.225, .45, 17, 7.), Wheel(.195, .65, 15, 6.), CN7_R16)


def bend(side):
    # Keep the existing powered splitter and its exit tangent. Two circular bends
    # and a lateral straight join it to the outboard feed at x=4.71.
    points = [(SPLIT_X-2.14*t, 1.19*t*t*(3-2*t)) for t in (i/16 for i in range(9))]
    x, y = points[-1]
    angle = atan(1.5*1.19/2.14)
    radius = 1.07/(2-sin(angle))
    n = ceil(radius*(pi/2-angle)/.13)
    for i in range(1, n+1):
        a = angle+(pi/2-angle)*i/n
        points.append((x-radius*(sin(a)-sin(angle)), y+radius*(cos(angle)-cos(a))))
    x, y = points[-1]
    span = LINE_Y-radius-y
    n = ceil(span/.13)
    points.extend((x, y+span*i/n) for i in range(1,n+1))
    n = ceil(radius*pi/2/.13)
    for i in range(1,n+1):
        a = pi/2*(1-i/n)
        points.append((x-radius*(1-sin(a)), LINE_Y-radius+radius*cos(a)))
    return tuple((px, side*py) for px, py in points)


def route(x, side):
    return (FEED, *bend(side), (x, side*LINE_Y))


def motion(points, start, speed=SPEED):
    time = start
    yield time, points[0]
    for a, b in zip(points, points[1:]):
        time += hypot(b[0] - a[0], b[1] - a[1]) / speed
        yield time, b


def schedule(inject, mixed_sizes=False):
    start = 0.
    for number, wheel in enumerate(WHEELS if mixed_sizes else (CN7_R16,)*4, 1):
        def delivery(name, launch):
            return list(motion(route(AXLE, STATIONS[name][1]), launch))

        deliveries = {name: delivery(name, start+4.2*i) for i,name in enumerate(("FL","FR"))}
        reject = None
        if inject and number == 1:
            original = deliveries["FL"]
            reject_start = max(original[-1][0], deliveries["FR"][-1][0])+1.
            reject = original + list(motion(((AXLE,LINE_Y),(REJECT_X,LINE_Y),
                                              (REJECT_X,REJECT_CRATE_Y)), reject_start))
            deliveries["FL"] = delivery("FL", reject[-1][0]+1.)
        front_seat = max(path[-1][0] for path in deliveries.values())+7.
        front_tighten = front_seat+5.
        # Both front supports are withdrawn before the first clamps open.
        index_start = front_tighten+NUT_CYCLE+4.
        for i,name in enumerate(("RL","RR")):
            deliveries[name] = delivery(name, front_seat+4.2*i)
        rear_ready = max(deliveries[name][-1][0] for name in ("RL","RR"))
        rear_seat = max(rear_ready, index_start+INDEX_TRAVEL+2.)+7.
        rear_tighten = rear_seat+5.
        departure = rear_tighten+NUT_CYCLE+8.
        end = departure+AMR_OUT_GATE+AMR_OUT_BUFFER
        yield number, wheel, start, deliveries, reject, front_seat, rear_seat, rear_tighten, front_tighten, departure, end
        start = end+1.
