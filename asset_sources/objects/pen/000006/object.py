"""Generate this pen only; +X rear, -X writing tip, dimensions in metres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.pen_extensions import build_pen


def build():
    build_pen("teal_marker")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
