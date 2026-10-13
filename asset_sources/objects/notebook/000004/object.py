"""Build the closed slate medium notebook."""
from table_1000.modeling.notebook_assets import generate_notebook


def main(argv=None):
    generate_notebook("slate", __file__, argv, size="medium")


if __name__ == "__main__":
    main()
