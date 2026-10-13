"""Build the closed taupe large notebook."""
from table_1000.modeling.notebook_assets import generate_notebook


def main(argv=None):
    generate_notebook("taupe", __file__, argv, size="large")


if __name__ == "__main__":
    main()
