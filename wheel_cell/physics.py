"""Live four-corner PhysX cell, method 1: flat tires on the station conveyors stop in a V-stop
pick zone for the mount robot's roller/centre-pin gripper; the car is docked on the AMR with
hubs on their bearings and suspension. Robot control is not part of the cell yet.

Entry point that keeps `from physics import ...` working. The code lives in:
    live_params.py  constants (paths, rate, replay timing, phases)
    sim_util.py     USD/PhysX building helpers
    wheel.py        wheel mounting-face colliders
    corner.py       hub on bearing, suspension, tolerances, ring tire, camera sensor
    handling.py     pick zone (V-stop, photo-eyes, centering arms, lift bars)
    gripper.py      roller / centre-pin gripper on the mount robot flange
    station.py      wheel station: pick zone + wheel + measurement
    live_robot.py   mount and fasten robots (old rail layout, RS080N port pending)
    carrier.py      AMR carrier and dock clamps
    cell.py         the cell: build and step everything
"""
from live_params import *  # noqa: F401,F403
from sim_util import *  # noqa: F401,F403
from process import STATIONS  # noqa: F401
from carrier import Carrier  # noqa: F401
from cell import Cell  # noqa: F401
from live_robot import Robot  # noqa: F401
from station import Station, make_station  # noqa: F401
from wheel import collision_web, merge_convex  # noqa: F401
