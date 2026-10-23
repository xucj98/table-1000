"""Build clip_lead/000000 (red); dimensions live in soldering_assets.PROFILES."""
from table_1000.modeling.soldering_assets import build_soldering_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_soldering_asset("clip_lead", "red")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
