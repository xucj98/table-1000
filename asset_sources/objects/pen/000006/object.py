"""Generate this pen only; +X rear, -X writing tip, dimensions in metres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.pen_extensions import build_pen


def build_base():
    build_pen("teal_marker")


from table_1000.modeling.pen_detents import add_detents
DETENT = {'seat': -0.032, 'neck': 0.0078, 'segments': 12, 'balls': 3, 'radius': 0.0006, 'height': 0.00033749999999999996, 'travel': 0.0006, 'mass': 0.00042187499999999983, 'stiffness': 600.0, 'damping': 0.22499999999999995, 'preload': 0.0001, 'flat': False, 'dt': 0.001}

def build():
    build_base()
    add_detents(DETENT)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
