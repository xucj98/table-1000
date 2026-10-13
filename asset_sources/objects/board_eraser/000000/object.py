"""Build the white board eraser with fixed black pad; dimensions are metres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.small_stationery_assets import build_white_board_eraser

SIZE_M = (0.130, 0.055, 0.025)


def build():
    build_white_board_eraser(SIZE_M)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
