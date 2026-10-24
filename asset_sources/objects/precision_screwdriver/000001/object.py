"""Build precision_screwdriver/000001 (red); dimensions live in workbench_assets.PROFILES."""
from table_1000.modeling.workbench_assets import build_workbench_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_workbench_asset("precision_screwdriver", "red")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
