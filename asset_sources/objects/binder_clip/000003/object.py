"""Build the large pink binder clip with rigid jaws and complete hinged loops."""
from table_1000.modeling.binder_clip_assets import generate_clip


def main(argv=None):
    generate_clip("pink", __file__, argv, size="large", articulated=True)


if __name__ == "__main__":
    main()
