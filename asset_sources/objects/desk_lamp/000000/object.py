"""Build desk_lamp/000000 (default); dimensions live in bench_backdrop_assets.PROFILES."""
from table_1000.modeling.bench_backdrop_assets import build_backdrop_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_backdrop_asset("desk_lamp", "default")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
