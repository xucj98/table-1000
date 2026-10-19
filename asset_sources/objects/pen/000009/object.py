"""Three-body retractable pen with independent 3 mm native sliders.

Geometry only; finite-force drives and state changes are authored separately.
"""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.pen_extensions import build_pen


def build_base():
    build_pen("blue_click")


from table_1000.modeling.pen_detents import split_click

def build():
    build_base()
    split_click(axis_height=0., anchor_x=.067, shaft_rear_x=.0638)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
