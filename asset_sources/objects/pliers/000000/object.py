"""Build pliers/000000 (needle_nose); dimensions live in workbench_assets.PROFILES."""
from table_1000.modeling.workbench_assets import build_workbench_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_workbench_asset("pliers", "needle_nose")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
