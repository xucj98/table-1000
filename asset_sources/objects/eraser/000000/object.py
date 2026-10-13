"""Build the red-sleeved eraser; dimensions are metres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.small_stationery_assets import build_red_eraser

SIZE_M = (0.060, 0.015, 0.010)


def build():
    build_red_eraser(SIZE_M)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
