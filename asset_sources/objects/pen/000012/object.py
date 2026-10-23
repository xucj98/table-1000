"""Build pen/000012 (grey_ballpoint); dimensions live in workbench_clutter_assets.PROFILES."""
from table_1000.modeling.workbench_clutter_assets import build_clutter_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_clutter_asset("pen", "grey_ballpoint")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
