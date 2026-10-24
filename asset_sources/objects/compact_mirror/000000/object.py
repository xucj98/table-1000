"""Build compact_mirror/000000 (compact_mirror); dimensions in the helper are millimetres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.vanity_assets import build_vanity_asset


def build():
    build_vanity_asset("compact_mirror")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
