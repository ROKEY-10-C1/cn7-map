"""USD/PhysX helpers for the equipment modules (handling.py, gripper.py).

They live in physics.py; these modules are imported from inside physics.py's builders, after
physics.py has finished loading, so the re-export below is safe.
"""
import physics as _physics

constrain = _physics.constrain
drive = _physics.drive
mass_properties = _physics.mass_properties
rotate = _physics.rotate


def body(stage, path, pos, orientation, mass, kinematic=False, light=False):
    # light: drive-dominated tool bodies; solver iterations are set scene-wide here.
    return _physics.body(stage, path, pos, orientation, mass, kinematic)
convex_prism = _physics.convex_prism
