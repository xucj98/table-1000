"""Build the tool_chest drawer unit; dimensions are metres."""
from table_1000.modeling.drawer_unit_assets import build_drawer_unit
from table_1000.modeling.asset_builders import generate


def build():
    build_drawer_unit("tool_chest")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
