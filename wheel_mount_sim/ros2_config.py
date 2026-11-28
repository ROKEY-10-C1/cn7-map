"""Phase-2 ROS 2 interface of the mount cell: names and message formats in one place (PROVISIONAL, to be agreed
with the AMR / fastening / integration teams). Plain Python, no ROS import: read by the Isaac Sim side
(cell_mount_ros2.py) and by the ROS 2 nodes (ros2_nodes/).

All messages are std_msgs/String.

Cell signals
  /Parking   in   "success"                    car parked at a mount position: 1st = dock A (front wheels),
                                               2nd = dock B after the index move (rear wheels)
  /Mount     out  "<wheel>:success"            one per wheel, wheel = FL|FR|RL|RR
                  "<wheel>:fail:<reason>"      reason = pick|ik|jam|release|timeout|feed

Robot controller <-> sequencer (one pair per mount robot, side = left|right)
  /mount_robot/<side>/cmd     sequencer -> controller   "<wheel>:<step>"            e.g. "FL:pick"
  /mount_robot/<side>/state   controller -> sequencer   "<wheel>:<step>:done"
                                                         "<wheel>:<step>:fail:<reason>"
                                                         "<wheel>:<step>:busy"  (answer to a cmd while busy)
"""

TOPIC_PARKING = "/Parking"
TOPIC_MOUNT = "/Mount"
PARKING_OK = "success"
MOUNT_OK = "success"
MOUNT_FAIL = "fail"
FAIL_REASONS = ("pick", "ik", "jam", "release", "timeout", "feed")

ROBOT_CMD = "/mount_robot/{side}/cmd"
ROBOT_STATE = "/mount_robot/{side}/state"
SIDES = {"left": +1, "right": -1}           # robot side -> station side (cell y)
WHEELS = {                                 # n-th /Parking -> wheel of each robot
    1: {"left": "FL", "right": "FR"},
    2: {"left": "RL", "right": "RR"},
}

# steps the sequencer sends, in order (MountSide methods in mount_control.py)
STEPS = ("prepare", "wait_tire", "pick", "transfer", "align", "insert", "release", "home", "judge")
# wall-clock timeout per step for the sequencer (the sim runs at ~0.6x real time headless)
STEP_TIMEOUT_S = {"wait_tire": 90., "default": 60.}

# fake AMR in the Isaac Sim process (the AMR team owns the real one)
FAKE_AMR_FIRST_PARKING_S = 1.0             # sim s after start: publish the 1st /Parking (car stands at A)
FAKE_AMR_INDEX_ON = ("FL", "FR")           # move A->B once these wheels reported on /Mount (success or fail)
