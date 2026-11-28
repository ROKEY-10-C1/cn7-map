"""Mount-cell sequencer, ROS 2 node (system ROS 2 Jazzy, Python 3.12).

Order of work for the two mount robots: on each /Parking "success" it takes the wheel pair for that stop
(1st: FL/FR at dock A, 2nd: RL/RR at dock B), sends each robot the mount steps one at a time
(/mount_robot/<side>/cmd "<wheel>:<step>"), waits for "<wheel>:<step>:done" and publishes the result per wheel
on /Mount: "<wheel>:success" or "<wheel>:fail:<reason>". A step that does not answer in time is a
"fail:timeout". Names and formats: ../ros2_config.py (provisional).

    source /opt/ros/jazzy/setup.bash && python3 wheel_mount_sim/ros2_nodes/mount_sequencer.py
"""
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import ros2_config as C  # noqa: E402


class Job:
    def __init__(self, side, wheel):
        self.side, self.wheel = side, wheel
        self.i = 0
        self.sent_at = None
        self.retry_at = None
        self.done = False

    @property
    def step(self):
        return C.STEPS[self.i]


class Sequencer(Node):
    def __init__(self):
        super().__init__("mount_sequencer")
        self.parked = 0
        self.jobs = {}
        self.cmd = {s: self.create_publisher(String, C.ROBOT_CMD.format(side=s), 10) for s in C.SIDES}
        self.mount = self.create_publisher(String, C.TOPIC_MOUNT, 10)
        for s in C.SIDES:
            self.create_subscription(String, C.ROBOT_STATE.format(side=s), self.on_state(s), 10)
        self.create_subscription(String, C.TOPIC_PARKING, self.on_parking, 10)
        self.create_timer(.2, self.on_timer)
        self.get_logger().info(f"waiting for {C.TOPIC_PARKING} '{C.PARKING_OK}'")

    def on_parking(self, msg):
        if msg.data.strip() != C.PARKING_OK:
            self.get_logger().warn(f"{C.TOPIC_PARKING} '{msg.data}' ignored")
            return
        if any(not j.done for j in self.jobs.values()):
            self.get_logger().warn("parking while robots still busy: ignored")
            return
        self.parked += 1
        pair = C.WHEELS.get(self.parked)
        if pair is None:
            self.get_logger().warn(f"parking #{self.parked}: no wheels left")
            return
        self.get_logger().info(f"parking #{self.parked}: mount {pair}")
        self.jobs = {side: Job(side, wheel) for side, wheel in pair.items()}
        for job in self.jobs.values():
            self.send(job)

    def send(self, job):
        text = f"{job.wheel}:{job.step}"
        self.cmd[job.side].publish(String(data=text))
        job.sent_at, job.retry_at = time.monotonic(), None
        self.get_logger().info(f"{job.side} <- {text}")

    def report(self, job, ok, reason=""):
        job.done = True
        text = f"{job.wheel}:{C.MOUNT_OK}" if ok else f"{job.wheel}:{C.MOUNT_FAIL}:{reason}"
        self.mount.publish(String(data=text))
        self.get_logger().info(f"{C.TOPIC_MOUNT} {text}")

    def on_state(self, side):
        def cb(msg):
            job = self.jobs.get(side)
            parts = msg.data.strip().split(":")
            if job is None or job.done or len(parts) < 3 or parts[0] != job.wheel or parts[1] != job.step:
                return
            status = parts[2]
            if status == "busy":
                job.retry_at = time.monotonic()+.5
            elif status == "done":
                if job.i == len(C.STEPS)-1:
                    self.report(job, True)
                else:
                    job.i += 1
                    self.send(job)
            elif status == C.MOUNT_FAIL:
                reason = parts[3] if len(parts) > 3 and parts[3] in C.FAIL_REASONS else "timeout"
                self.report(job, False, reason)
        return cb

    def on_timer(self):
        now = time.monotonic()
        for job in self.jobs.values():
            if job.done:
                continue
            if job.retry_at is not None and now >= job.retry_at:
                self.send(job)
            limit = C.STEP_TIMEOUT_S.get(job.step, C.STEP_TIMEOUT_S["default"])
            if job.sent_at is not None and now-job.sent_at > limit:
                self.get_logger().error(f"{job.side} {job.wheel}:{job.step} no answer in {limit:.0f} s")
                self.report(job, False, "timeout")


def main():
    rclpy.init()
    node = Sequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
