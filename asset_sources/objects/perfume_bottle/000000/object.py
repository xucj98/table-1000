"""Build perfume_bottle/000000 (perfume_bottle/rect); dimensions in the helper are millimetres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.vanity_assets import build_vanity_asset


def build():
    build_vanity_asset("perfume_bottle/rect")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
