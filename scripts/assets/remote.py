"""Pull or push archived assets/ directories (see table_1000/assets/remote.py)."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from table_1000.assets.remote import main
if __name__ == '__main__':
    main()
