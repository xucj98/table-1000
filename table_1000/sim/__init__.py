"""Simulation of robots in scenes: the Isaac Lab environment (``table_1000.sim.isaaclab``).

Control timing shared by every backend: one action per ``CONTROL_HZ`` step,
``SUBSTEPS`` physics steps of ``PHYSICS_DT`` each.
"""

CONTROL_HZ = 25
PHYSICS_DT = .004
SUBSTEPS = round(1 / (CONTROL_HZ * PHYSICS_DT))
