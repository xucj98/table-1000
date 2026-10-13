"""Build the closed black pocket notebook."""
from table_1000.modeling.notebook_assets import generate_notebook


def main(argv=None):
    generate_notebook("black", __file__, argv)


if __name__ == "__main__":
    main()
