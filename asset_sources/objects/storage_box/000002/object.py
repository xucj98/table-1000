"""Build the mini storage_box; dimensions are metres."""
from table_1000.modeling.storage_box_assets import build_box
from table_1000.modeling.asset_builders import generate


def build():
    build_box("mini")


def main(argv=None):
    generate(build,__file__,argv)


if __name__=="__main__":
    main()
