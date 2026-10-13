"""Build the white short knife, blade retracted; dimensions are metres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.small_stationery_assets import build_white_knife

SIZE_M = (0.095, 0.020, 0.010)


def build():
    build_white_knife(SIZE_M)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
