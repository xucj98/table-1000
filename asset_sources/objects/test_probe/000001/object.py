"""Build test_probe/000001 (black); dimensions live in workbench_assets.PROFILES."""
from table_1000.modeling.workbench_assets import build_workbench_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_workbench_asset("test_probe", "black")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
