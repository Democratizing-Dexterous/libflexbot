"""Interactive libflexbot debug script.

The robot YAML config is a command line argument:

    python -i cli.py path/to/motors.yaml

Every option of the ``flexcli`` console script works here as well, for example
``python -i cli.py motors.yaml --freq 500`` or
``python -i cli.py motors.yaml --mode pv``. Run ``python -m libflexbot.cli
--help`` for the full list.

The installed ``flexcli`` script runs this same setup and then opens the
interactive shell, so ``flexcli path/to/motors.yaml`` is equivalent to
``python -i cli.py path/to/motors.yaml``.
"""

import traceback
from pathlib import Path
import time

import numpy as np

from libflexbot import CanFD, Robot
from libflexbot.cli import bring_up, parse_args

if __name__ == "__main__":
    args = parse_args()
    namespace = {
        "args": args,
        "np": np,
        "time": time,
        "Path": Path,
        "CanFD": CanFD,
        "Robot": Robot,
    }

    try:
        bring_up(args, namespace)
    except Exception:
        # Same behaviour as `python -i`: keep the shell so the error can be
        # inspected with whatever objects were created before it.
        traceback.print_exc()

    globals().update(namespace)
