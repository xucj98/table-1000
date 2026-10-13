"""Build this complete small prop as one rigid body with semantic parts."""
from table_1000.modeling.small_props_assets import build_small_prop, generate_small_prop


def main(argv=None):
    generate_small_prop(lambda: build_small_prop("remote_control"), __file__, argv)


if __name__ == "__main__":
    main()
