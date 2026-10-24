"""Build calculator/000001 (graphing); dimensions live in workbench_clutter_assets.PROFILES."""
from table_1000.modeling.workbench_clutter_assets import build_clutter_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_clutter_asset("calculator", "graphing")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
