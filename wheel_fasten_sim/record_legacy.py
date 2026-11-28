"""Improvement-before reenactment for recording the original upright/PARK routing."""
import sys

from isaacsim import SimulationApp


def main():
    app = SimulationApp({"headless": "--headless" in sys.argv, "multi_gpu": False})
    try:
        return record(app)
    finally:
        app.close()


def record(app):
    from unittest.mock import patch

    import numpy as np

    import main as runner

    sys.path.insert(0, str(runner.HERE.parent / "wheel_cell"))
    import fastening

    class LegacyController(fastening.FastenController):
        def __init__(self, *args, **kwargs):
            kwargs["return_to_park"] = True
            super().__init__(*args, **kwargs)

        def solve(self, obs, position, rotation, fallback=False):
            base = obs.base
            q = self.arm.solve(np.zeros(3), base.rotation.T @ (position-base.xyz),
                               side=self.task.side, previous=self.q, fallback=fallback,
                               rotation=base.rotation.T @ rotation)
            if q is None:
                raise RuntimeError("Pose IK failed on the current branch")
            return q

        @staticmethod
        def upright(yaw):
            pose = np.zeros(6)
            pose[0] = yaw
            return pose

        def plan_motion(self, obs, goal, *, transit=False, start=None, allow_detour=True):
            # 촬영용 경유 순서만 바꾼다. 공통 루프와 이송 중 통과 영역 검사는 그대로 사용한다.
            if self.state == "SETTLE":
                return [self.upright(self.park[0]), self.upright(goal[0]), goal]
            if self.state == "RELOAD_PLAN" and start is None:
                return [self.upright(0.), self.upright(goal[0]), goal]
            if self.state == "RELOAD_CHECK":
                return [goal, self.upright(goal[0]), self.upright(0.), self.park]
            return [goal]

        def control(self, obs):
            if self.state != "RETRACT":
                return super().control(obs)
            outward = obs.hub.rotation[:, 2]
            seats = obs.seats[self.mapping]
            clearance = float((obs.tcp.xyz-seats.mean(axis=0)) @ outward)
            if clearance < .09:
                self.feed(obs, obs.tips[self.mapping]+outward*self.config.stand_off)
                return
            self.metrics["retract_clearance_mm"] = clearance*1000
            self.queue_motion([self.upright(self.approach[0]), self.upright(self.park[0]), self.park])
            self.enter("PARK")

        def update_reload(self, dt):
            if self.state == "RELOAD_CLEAR":
                if not self.move(dt) or not self.parked(self.task.observe()):
                    return
            super().update_reload(dt)

    print("RECORDING: legacy upright/PARK route reenactment", flush=True)
    with patch.object(fastening, "FastenController", LegacyController):
        return runner.run(app, "--headless" in sys.argv, "full")


if __name__ == "__main__":
    raise SystemExit(main())
