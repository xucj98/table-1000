"""Generate this pen only; +X rear, -X writing tip, dimensions in metres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.pen_extensions import build_pen


def build_base():
    build_pen("white_marker")


from table_1000.modeling.pen_detents import add_detents
DETENT = {'seat': -0.025, 'neck': 0.0056, 'segments': 12, 'balls': 3, 'radius': 0.00045, 'height': 0.000253125, 'travel': 0.00045000000000000004, 'mass': 0.000177978515625, 'stiffness': 984.375, 'damping': 0.16874999999999998, 'preload': 0.00016874999999999998, 'flat': False, 'dt': 0.001}

def build():
    build_base()
    add_detents(DETENT)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
