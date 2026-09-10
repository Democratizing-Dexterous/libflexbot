"""Interactive debugging entry point for the libflexbot SDK.

The installed ``flexcli`` console script behaves like ``python -i cli.py``:
it configures the CAN-FD device and the robot from a YAML file, enables the
motors and then drops into a Python shell with the live objects in scope.

    $ flexcli config/motors.yaml

    libflexbot interactive shell
      canfd  : device_index=0, initialized=True
      robot  : mode=mit, ids=[1], running=True
      exit() or Ctrl-D to quit; enabled motors are disabled on exit

    >>> robot.getj()["pos"]
"""

import argparse
import code
import sys
import time
import traceback
from pathlib import Path

import numpy as np

from . import MODES, CanFD, Robot

__all__ = ["build_parser", "parse_args", "bring_up", "interact", "main"]

# Keeps the live objects alive until interpreter shutdown, so the atexit handler
# can stop the control loop while the device is still open.
_SESSION = None


def build_parser():
    parser = argparse.ArgumentParser(
        prog="flexcli",
        description="Set up a libflexbot robot from a YAML config and open an interactive shell.",
    )
    parser.add_argument(
        "config",
        type=Path,
        help="path to the robot YAML configuration",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="CAN-FD device index (default: %(default)s)",
    )
    parser.add_argument(
        "--serial",
        default=None,
        help="CAN-FD device serial number, needed when several devices are present",
    )
    parser.add_argument(
        "--lib",
        default="",
        help="path to libcontrolcanfd.so (default: the bundled driver)",
    )
    parser.add_argument(
        "--abit",
        type=int,
        default=1_000_000,
        help="arbitration bitrate in bit/s (default: %(default)s)",
    )
    parser.add_argument(
        "--bbit",
        type=int,
        default=5_000_000,
        help="data bitrate in bit/s (default: %(default)s)",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=1,
        help="CAN channel index, zero based (default: %(default)s)",
    )
    parser.add_argument(
        "--freq",
        type=int,
        default=1000,
        help="control loop frequency in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--soft-limit",
        dest="soft_limit",
        action="store_true",
        help="apply the YAML position soft limits; the control loop stops when a measured position leaves its range",
    )
    parser.add_argument(
        "--no-soft-limit",
        dest="soft_limit",
        action="store_false",
        default=False,
        help="do not apply the YAML position soft limits (default)",
    )
    parser.add_argument(
        "--mode",
        choices=sorted(MODES),
        default="mit",
        help="motor control mode (default: %(default)s)",
    )
    parser.add_argument(
        "--cpu",
        type=int,
        default=None,
        help="pin the control loop to this CPU core (default: unpinned)",
    )
    parser.add_argument(
        "--no-enable",
        dest="enable",
        action="store_false",
        default=True,
        help="configure the robot but do not send the enable frames",
    )
    return parser


def parse_args(argv=None):
    return build_parser().parse_args(argv)


def bring_up(args, namespace=None):
    """Create the device and robot described by ``args``.

    Live objects are stored in ``namespace`` as soon as they exist, so a caller
    that catches an error can still inspect whatever came up. Raises on failure.
    """
    if namespace is None:
        namespace = {}

    config = Path(args.config).expanduser()
    if not config.is_file():
        raise FileNotFoundError(f"config file not found: {config}")
    namespace["config"] = config

    canfd = CanFD(device_index=args.device_index, serial=args.serial, lib_path=args.lib)
    namespace["canfd"] = canfd
    canfd.init(Abit=args.abit, Bbit=args.bbit)

    robot = Robot(
        canfd,
        can_channel=args.channel,
        freq=args.freq,
        config=config,
        soft_limit=args.soft_limit,
        mode=args.mode,
    )
    namespace["robot"] = robot

    if args.enable:
        robot.enable(cpu=args.cpu)

    return namespace


def _base_namespace(args):
    return {
        "args": args,
        "np": np,
        "time": time,
        "Path": Path,
        "CanFD": CanFD,
        "Robot": Robot,
    }


def interact(namespace):
    lines = ["", "libflexbot interactive shell"]
    canfd = namespace.get("canfd")
    robot = namespace.get("robot")
    if canfd is not None:
        lines.append(f"  canfd  : device_index={canfd.device_index}, initialized={canfd.initialized}")
    if robot is not None:
        lines.append(
            f"  robot  : mode={robot.mode}, ids={robot.motor_ids}, running={robot.running}"
        )
    lines.append("  exit() or Ctrl-D to quit; enabled motors are disabled on exit")
    lines.append("")
    code.interact(banner="\n".join(lines), local=namespace, exitmsg="")


def main(argv=None):
    global _SESSION
    args = parse_args(argv)
    namespace = _base_namespace(args)
    _SESSION = namespace
    ok = True
    try:
        bring_up(args, namespace)
    except Exception:
        ok = False
        traceback.print_exc()
        print(
            "flexcli: setup failed, dropping into the shell anyway (see 'args' for the parsed options)",
            file=sys.stderr,
        )
    interact(namespace)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
