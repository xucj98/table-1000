"""Build the orange disk magnet as one complete rigid body."""
from table_1000.modeling.magnet_assets import generate_magnet


def main(argv=None):
    generate_magnet("orange", __file__, argv)


if __name__ == "__main__":
    main()
