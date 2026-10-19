"""Generate this pen only; +X rear, -X writing tip, dimensions in metres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.pen_extensions import build_pen


def build_base():
    build_pen("white_fine")


from table_1000.modeling.pen_detents import add_detents
DETENT = {'seat': -0.0275, 'neck': 0.0038, 'segments': 12, 'balls': 3, 'radius': 0.00035, 'height': 0.000196875, 'travel': 0.00035, 'mass': 8.3740234375e-05, 'stiffness': 765.625, 'damping': 0.13125, 'preload': 0.00013125, 'flat': False, 'dt': 0.001}

def build():
    build_base()
    add_detents(DETENT)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
