"""Build monitor/000000 (default); dimensions live in bench_instrument_assets.PROFILES."""
from table_1000.modeling.bench_instrument_assets import build_instrument_asset
from table_1000.modeling.asset_builders import generate


def build():
    build_instrument_asset("monitor", "default")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
