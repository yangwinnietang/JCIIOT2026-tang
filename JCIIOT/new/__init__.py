import os

# Legacy scratch package (pre-robosuite factory_sorting prototype). The
# original ``world`` module was never committed and nothing imports this
# package at runtime; keep the init import-safe.
try:
    from .world import MujocoWorldBase
except ImportError:
    MujocoWorldBase = None

assets_root = os.path.join(os.path.dirname(__file__), "assets")
