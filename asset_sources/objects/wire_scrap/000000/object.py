"""Build wire_scrap/000000 (twisted_pair); dimensions live in workbench_clutter_assets.PROFILES."""
from table_1000.modeling.workbench_clutter_assets import build_clutter_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_clutter_asset("wire_scrap", "twisted_pair")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
