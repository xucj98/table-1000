"""Build the mini pale_yellow binder clip with rigid jaws and complete hinged loops."""
from table_1000.modeling.binder_clip_assets import generate_clip


def main(argv=None):
    generate_clip("pale_yellow", __file__, argv, size="mini", articulated=True)


if __name__ == "__main__":
    main()
