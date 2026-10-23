"""Build water_bottle/000001 (empty); dimensions live in workbench_clutter_assets.PROFILES."""
from table_1000.modeling.workbench_clutter_assets import build_clutter_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_clutter_asset("water_bottle", "empty")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
