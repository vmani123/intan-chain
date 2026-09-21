"""Put host/ on sys.path so `import chain` works from a bare `pytest host/tests`."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
