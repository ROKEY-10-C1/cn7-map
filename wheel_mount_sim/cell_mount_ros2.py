"""Phase 2: the mount cell driven over ROS 2 (phase 1 = cell_mount_run.py, unchanged).

This process is the robot-controller and plant side: Isaac Sim + PhysX at 240 Hz, the two mount robots' joint
servo, F/T force loop and gripper (mount_control.MountSide steps), and a fake AMR. It does not decide the order
of work: the sequencer node (ros2_nodes/mount_sequencer.py, system ROS 2 Jazzy) does, over the topics in
ros2_config.py:
    /Parking (in, from the fake AMR here or from outside), /Mount (out, from the sequencer),
    /mount_robot/<side>/cmd  "<wheel>:<step>"  ->  /mount_robot/<side>/state  "<wheel>:<step>:done|fail:<r>|busy"

Fake AMR (assumption: the AMR team owns the real transfer): publishes the 1st /Parking "success" shortly after
start (car stands at dock A); when FL and FR are reported on /Mount it moves the car +wheelbase to dock B
(kinematic, 6 s quintic) and publishes /Parking "success" again.

Run with the launcher (bundled Jazzy rclpy for Isaac's Python 3.11):
    ./wheel_mount_sim/cell_mount_ros2.sh [--no-video] [--out DIR] [--wait-parking] [--no-fake-amr]
and the sequencer in a second terminal:
    source /opt/ros/jazzy/setup.bash && python3 wheel_mount_sim/ros2_nodes/mount_sequencer.py
"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import ros2_config as C  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=42)
ap.add_argument("--no-video", action="store_true")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--out", default=None)
ap.add_argument("--basic", action="store_true")
ap.add_argument("--wait-parking", action="store_true", help="fake AMR does not send the 1st /Parking (send it by hand)")
ap.add_argument("--no-fake-amr", action="store_true", help="no fake AMR at all: /Parking from outside, no index move")
ap.add_argument("--index-on-fail", action="store_true", help="fake AMR indexes even if a front wheel failed")
ap.add_argument("--t-max", type=float, default=300., help="simulated seconds")
ap.add_argument("--hold", type=float, default=3., help="simulated seconds to keep running after the last /Mount")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1600 if args.gui else 960, "height": 900 if args.gui else 540,
                     "renderer": "RayTracedLighting", "anti_aliasing": 1, "multi_gpu": False})
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")              # puts the bundled rclpy on sys.path
app.update()
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import mount_control as MC  # noqa: E402

env = MC.CellEnv(app, args.seed, gui=args.gui, video=not args.no_video, out=args.out)
prof = MC.Profile(args.basic)
stations = {s.name: s for s in env.cell.stations}
front = env.stations(front=True)
units = {}
for lab, info in env.cell.mount_arms.items():
    side = lab.lower()
    units[side] = MC.MountSide(env, lab, info, front[info["side"]], prof)
    units[side].phase = "idle"
for u in units.values():
    u.command()

rclpy.init()
node = rclpy.create_node("mount_robot_controller")
ex = SingleThreadedExecutor()
ex.add_node(node)
pub_state = {side: node.create_publisher(String, C.ROBOT_STATE.format(side=side), 10) for side in units}
pub_parking = node.create_publisher(String, C.TOPIC_PARKING, 10)
mount_log, parking_log = [], []


def say(pub, text):
    pub.publish(String(data=text))
    print("ROS", pub.topic_name, text, flush=True)


def step_program(side, wheel, step):
    """One sequencer step on one robot; answers on /mount_robot/<side>/state when finished."""
    u = units[side]
    try:
        if step == "prepare":
            yield from u.prepare()
        elif step == "wait_tire":
            yield from u.wait_tire(timeout=60.)
        elif step == "pick":
            u.measure_pick()
            yield from u.pick()
        elif step == "transfer":
            yield from u.transfer()
        elif step == "align":
            yield from u.align()
        elif step == "insert":
            yield from u.insert()
        elif step == "release":
            yield from u.release_retract()
        elif step == "home":
            yield from u.go_home()
        elif step == "judge":
            yield from u.judge()
            if u.result != "mounted":
                raise MC.StopSide(f"wheel not on the hub after release ({u.result})", "release")
        else:
            raise MC.StopSide(f"unknown step {step}", "timeout")
        u.phase = "idle" if step == "judge" else u.phase
        say(pub_state[side], f"{wheel}:{step}:done")
    except MC.StopSide as exc:
        u.result = f"stopped: {exc}"
        u.fail_reason = exc.reason
        u.results[wheel] = u.result
        print("ERROR", u.label, exc, flush=True)
        u.event("failed", reason=exc.reason, message=str(exc), step=step)
        say(pub_state[side], f"{wheel}:{step}:{C.MOUNT_FAIL}:{exc.reason}")


def on_cmd(side):
    def cb(msg):
        try:
            wheel, step = msg.data.strip().split(":")[:2]
        except ValueError:
            print("ROS bad cmd", side, msg.data, flush=True)
            return
        u = units[side]
        if u.busy():
            say(pub_state[side], f"{wheel}:{step}:busy")
            return
        if wheel not in stations or stations[wheel].side != C.SIDES[side]:
            say(pub_state[side], f"{wheel}:{step}:{C.MOUNT_FAIL}:ik")
            return
        if u.station.name != wheel:
            u.station = stations[wheel]
            u.result, u.fail_reason = "incomplete", None
        env.log_event(dict(t=round(env.t, 3), side=u.label, event="cmd", wheel=wheel, step=step))
        u.load(step_program(side, wheel, step))
    return cb


for side in units:
    node.create_subscription(String, C.ROBOT_CMD.format(side=side), on_cmd(side), 10)


# ---------------------------------------------------------------- fake AMR
class FakeAMR:
    def __init__(self):
        self.reported = {}
        self.prog = None
        self.first_sent = False
        self.indexed = False
        self.held = False

    def parking(self, where):
        say(pub_parking, C.PARKING_OK)
        parking_log.append(dict(t=round(env.t, 3), where=where))
        env.log_event(dict(t=round(env.t, 3), side="AMR", event="parking", where=where))

    def on_mount(self, msg):
        parts = msg.data.split(":")
        mount_log.append(dict(t=round(env.t, 3), msg=msg.data))
        env.log_event(dict(t=round(env.t, 3), side="Cell", event="mount_msg", msg=msg.data))
        self.reported[parts[0]] = parts[1] if len(parts) > 1 else ""

    def tick(self):
        if args.no_fake_amr:
            return
        if not self.first_sent and not args.wait_parking and env.t >= C.FAKE_AMR_FIRST_PARKING_S:
            self.first_sent = True
            self.parking("A")
        if self.prog is None and not self.indexed and all(w in self.reported for w in C.FAKE_AMR_INDEX_ON):
            ok = all(self.reported[w] == C.MOUNT_OK for w in C.FAKE_AMR_INDEX_ON)
            if not ok and not args.index_on_fail:
                if not self.held:
                    self.held = True
                    env.log_event(dict(t=round(env.t, 3), side="AMR", event="index_held", reported=self.reported))
                return
            if any(u.busy() for u in units.values()):
                return
            self.indexed = True
            env.log_event(dict(t=round(env.t, 3), side="AMR", event="index_start"))
            self.prog = env.car_move(MC.process.WHEELBASE)
        if self.prog is not None:
            try:
                next(self.prog)
            except StopIteration:
                self.prog = None
                env.log_event(dict(t=round(env.t, 3), side="AMR", event="index_done", car_dx_m=round(env.car_dx, 4)))
                self.parking("B")


amr = FakeAMR()
node.create_subscription(String, C.TOPIC_MOUNT, amr.on_mount, 10)

# ---------------------------------------------------------------- loop
env.warmup()
print("READY mount_robot_controller", {s: C.ROBOT_CMD.format(side=s) for s in units}, flush=True)
t_last = None
wall0 = time.perf_counter()
try:
    while env.t < args.t_max:
        if env.step_i % 4 == 0:
            ex.spin_once(timeout_sec=0)
        env.tick([amr.tick]+[u.tick for u in units.values()])
        if len(amr.reported) >= 4 and t_last is None:
            t_last = env.t
        if t_last is not None and env.t > t_last+args.hold:
            break
except KeyboardInterrupt:
    print("interrupted", flush=True)
except Exception as exc:
    print("ERROR", exc, flush=True)
    import traceback
    traceback.print_exc()

env.finish(dict(wheels={k: v for u in units.values() for k, v in u.results.items()}, mount_msgs=mount_log,
                parking=parking_log, wall_s=round(time.perf_counter()-wall0, 1)))
node.destroy_node()
rclpy.shutdown()
if args.gui:
    while app.is_running():
        app.update()
env.cell.close()
app.close()
