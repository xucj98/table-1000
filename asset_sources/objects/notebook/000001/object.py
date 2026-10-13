"""Build the closed navy pocket notebook."""
from table_1000.modeling.notebook_assets import generate_notebook


def main(argv=None):
    generate_notebook("navy", __file__, argv)


if __name__ == "__main__":
    main()
