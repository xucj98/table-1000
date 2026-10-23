"""Build vanity_tray/000000 (vanity_tray); dimensions in the helper are millimetres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.vanity_assets import build_vanity_asset


def build():
    build_vanity_asset("vanity_tray")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
