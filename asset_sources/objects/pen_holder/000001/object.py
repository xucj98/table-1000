"""Build the black pen_holder; dimensions are metres."""
from table_1000.modeling.pen_holder_assets import build_holder
from table_1000.modeling.asset_builders import generate


def build():
    build_holder("black")


def main(argv=None):
    generate(build,__file__,argv)


if __name__=="__main__":
    main()
