"""Build cotton_pad_jar/000000 (cotton_pad_jar); dimensions in the helper are millimetres."""
from table_1000.modeling.asset_builders import generate
from table_1000.modeling.vanity_assets import build_vanity_asset


def build():
    build_vanity_asset("cotton_pad_jar")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
