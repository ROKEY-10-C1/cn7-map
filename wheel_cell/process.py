"""Deterministic JIS schedule in metres, seconds and radians."""
from dataclasses import dataclass
from math import hypot

from fitment import WHEELBASE
from layout import LAYOUT
from physical_data import AMR_LIFT_SPEED, AMR_SPEED_Y

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
REJECT_X = 4.25
REJECT_GATE_HALF = .365
REJECT_CRATE_Y = LINE_Y+1.05
BELT_Z = .50
SPEED = .30
# Aisle 0.5 m west of the old -7.0 so a car carried at yaw 0 clears the cell corner signal posts
# (x -4.61) by 0.46 m and the AMR guard (x -4.52) by 0.63 m (2026-11-27).
AMR_GATE_X = -7.5
# AMR legs timed to KUKA KMP 1500 limits (crab 0.56 m/s, straight 1.0 m/s) with ~10 % margin,
# plus the 0.05 m/s lift: buffer -> gate (crab), gate -> dock (straight), lift 0.12 m.
AMR_AT_GATE = 20.                 # lateral transfer from the outer buffer
AMR_GATE_LIFT = -.12
AMR_DOCKED = 8.                    # order start (at the gate) -> docked
AMR_LIFTED = AMR_DOCKED+3.
DOCK_X, INDEX_X, FINAL_X, EXIT_X = LAYOUT.stops
AMR_OUT_GATE = max(6., (EXIT_X-FINAL_X)/.8)  # stop C -> forward exit
# Forward exit -> east return aisle (straight), then crab -y to the outbound lane head as on the
# west aisle (AMR_AT_GATE). The car no longer backs through the cell to the gate.
RETURN_X = LAYOUT.return_x
AMR_TO_RETURN = max(6., (RETURN_X-EXIT_X)/.8)
AMR_OUT_BUFFER = AMR_AT_GATE
MOUNT_HUB_X = DOCK_X+AXLE
FASTEN_HUB_X = MOUNT_HUB_X+WHEELBASE
MOUNT_BASE_X = MOUNT_HUB_X-1.24
FASTEN_BASE_X = MOUNT_HUB_X+WHEELBASE/2
# Tyre stop on the feed belt, behind the mount arm: the arm turns round to it. 0.64 m from the hub
# line toward the base side keeps >= 20 deg joint margin (mirror of the x 1.86-2.16 scan, 2026-11-27).
PICK_X = MOUNT_HUB_X-.64
FEED_START_X = PICK_X-.32          # belt end just behind the stopped tyre (radius 0.316)
# Mount arm timing relative to the seat time (cell_motion.mount_keys): jaws close on the flat wheel,
# arm parked again after hanging it.
GRIP_AT = -5.3
MOUNT_CLEAR = 4.2
# The rear wheel waits upstream of the stop (0.27 m tread gap, room for the jaws) until the front wheel is lifted.
QUEUE_GAP = .90
ROBOT_BASE_Y = 2.50
# Keep 1.55 m clear beside the control button (0.21 m) and buffer posts (0.03 m).
CELL_Y = 5.25
AMR_BAY_Y = CELL_Y + .21 + 1.55 + .03 + 2.65
# Vehicle buffers: three slots per side, cars parked across the bay (yaw -90 inbound, +90 outbound).
# The cars rest on four stands at the body ends so the AMR+hanger (rails +/-1.80 m) can crab in under
# the car from the AMR lane; slots are served in order. Inbound 1->2->3 from the west aisle;
# outbound cars arrive from the east return aisle and fill the far slot first, 1->2->3.
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
BUFFER_CRAB_SPEED = .9 * AMR_SPEED_Y
BUFFER_TURN_TIME = 6.              # 90 degrees at 15 degrees/s
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
WHEELS = (CN7_R16, Wheel(.225, .45, 17, 7.), Wheel(.195, .65, 15, 6.))


def inbound_pickup(slot_x, start=0.):
    """Carrier keys (seconds, X, Y, yaw, lift) from the inbound aisle to the gate."""
    lane = (AMR_GATE_X, AMR_BAY_Y)
    slot = (slot_x, AMR_BAY_Y)
    crab = (slot_x-AMR_GATE_X)/BUFFER_CRAB_SPEED
    arrived = start+crab
    contact = arrived+(BUFFER_CAR_Z-AMR_LIFT_ENTRY)/AMR_LIFT_SPEED
    lifted = contact+(AMR_LIFT_CARRY-BUFFER_CAR_Z)/AMR_LIFT_SPEED
    returned = lifted+crab
    turned = returned+BUFFER_TURN_TIME
    at_gate = turned+AMR_AT_GATE
    return ((start,*lane,-90.,AMR_LIFT_ENTRY), (arrived,*slot,-90.,AMR_LIFT_ENTRY),
            (contact,*slot,-90.,BUFFER_CAR_Z), (lifted,*slot,-90.,AMR_LIFT_CARRY),
            (returned,*lane,-90.,AMR_LIFT_CARRY), (turned,*lane,0.,AMR_LIFT_CARRY),
            (at_gate,AMR_GATE_X,0.,0.,AMR_LIFT_CARRY),
            (at_gate+(AMR_LIFT_CARRY-AMR_GATE_LIFT)/AMR_LIFT_SPEED,AMR_GATE_X,0.,0.,AMR_GATE_LIFT))


def outbound_dropoff(slot_x, lane_head):
    """Carrier keys from the east aisle to the stands and back out empty."""
    lane = (RETURN_X, -AMR_BAY_Y)
    slot = (slot_x, -AMR_BAY_Y)
    crab = (RETURN_X-slot_x)/BUFFER_CRAB_SPEED
    turned = lane_head+BUFFER_TURN_TIME
    arrived = turned+crab
    parked = arrived+(AMR_LIFT_CARRY-BUFFER_CAR_Z)/AMR_LIFT_SPEED
    lowered = parked+(BUFFER_CAR_Z-AMR_LIFT_ENTRY)/AMR_LIFT_SPEED
    return ((lane_head,*lane,0.,AMR_LIFT_CARRY), (turned,*lane,90.,AMR_LIFT_CARRY),
            (arrived,*slot,90.,AMR_LIFT_CARRY), (parked,*slot,90.,BUFFER_CAR_Z),
            (lowered,*slot,90.,AMR_LIFT_ENTRY), (lowered+crab,*lane,90.,AMR_LIFT_ENTRY))


def belt_height(x):
    portion = (x-LAYOUT.ramp_bottom_x)/(LAYOUT.ramp_top_x-LAYOUT.ramp_bottom_x)
    return BELT_Z + LAYOUT.floor_rise * max(0., min(1., portion))


def route(x, side):
    y = side*LINE_Y
    return ((LAYOUT.supply_x, y), (LAYOUT.ramp_top_x, y), (LAYOUT.ramp_bottom_x, y), (x, y))


def motion(points, start, speed=SPEED):
    time = start
    yield time, points[0]
    for a, b in zip(points, points[1:]):
        time += hypot(b[0]-a[0], b[1]-a[1], belt_height(b[0])-belt_height(a[0])) / speed
        yield time, b


def schedule(inject, mixed_sizes=False, *, start=0.):
    for number, wheel in enumerate(WHEELS if mixed_sizes else (CN7_R16,)*len(BUFFER_SLOT_X), 1):
        def delivery(name, launch):
            return list(motion(route(PICK_X, STATIONS[name][1]), launch))

        deliveries = {name: delivery(name, start+4.2*i) for i,name in enumerate(("FL","FR"))}
        reject = None
        if inject and number == 1:
            original = deliveries["FL"]
            reject_start = max(original[-1][0], deliveries["FR"][-1][0])+1.
            reject = original + list(motion(((PICK_X,LINE_Y),(REJECT_X,LINE_Y),
                                              (REJECT_X,REJECT_CRATE_Y)), reject_start))
            deliveries["FL"] = delivery("FL", reject[-1][0]+1.)
        front_seat = max(path[-1][0] for path in deliveries.values())+7.
        # Stop A: front wheels hung, mount arms parked -> the car moves on to stop B with the front
        # wheels hung on the studs (retention during the move is handled outside this model).
        index_start = front_seat+MOUNT_CLEAR
        redock = index_start+INDEX_TRAVEL
        # Stop B: fasten the front (hub MOUNT_HUB_X+WHEELBASE) while the rear is mounted at MOUNT_HUB_X.
        front_tighten = redock+2.
        # Rear wheels follow the front ones on the trunk (4.2 s apart), wait at the queue spot and
        # move onto the stop once the front wheel of their side has been lifted (0.7 s after the grip).
        last_launch = max(deliveries[name][0][0] for name in ("FL","FR"))
        for i,name in enumerate(("RL","RR")):
            side = STATIONS[name][1]
            queued = list(motion(route(PICK_X+QUEUE_GAP, side), last_launch+4.2*(i+1)))
            release = max(queued[-1][0], front_seat+GRIP_AT+.8)
            deliveries[name] = queued+list(motion(((PICK_X+QUEUE_GAP, side*LINE_Y), (PICK_X, side*LINE_Y)), release))
        rear_ready = max(deliveries[name][-1][0] for name in ("RL","RR"))
        # Wheel on the stop before the grip (+0.5 s), arm leaves its park 8.5 s ahead (it parked
        # MOUNT_CLEAR after the front seat), car docked 2 s before the 2.6 s approach.
        rear_seat = max(rear_ready-GRIP_AT+.5, index_start+8.5, redock+2.+2.6)
        # Rear fastening once the mount arm has left the hub (parked 4.2 s after the seat, head at the
        # hub 1.94 s after the start) and the fasten arm is parked from the front (2.5 s after the
        # cycle; it leaves its park 5 s before the next start).
        second_index = max(rear_seat+MOUNT_CLEAR, front_tighten+NUT_CYCLE+2.5)
        final_dock = second_index+INDEX_TRAVEL
        # Fasten leaves PARK 5 s before tighten; it starts after the second stop.
        rear_tighten = final_dock+5.5
        departure = rear_tighten+NUT_CYCLE+8.
        end = departure+AMR_OUT_GATE          # forward exit reached: the cell is free for the next car
        lane_head = end+AMR_TO_RETURN+AMR_OUT_BUFFER
        yield number, wheel, start, deliveries, reject, front_seat, rear_seat, rear_tighten, front_tighten, second_index, final_dock, departure, end, lane_head
        start = end+1.
