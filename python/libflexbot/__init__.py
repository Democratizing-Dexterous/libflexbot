import atexit
from pathlib import Path
import weakref

from ._libflexbot import _CanFD, _Robot


_ACTIVE_ROBOTS = weakref.WeakSet()


def _shutdown_robots():
    for robot in list(_ACTIVE_ROBOTS):
        try:
            robot.disable()
        except Exception:
            pass


atexit.register(_shutdown_robots)


#: Robot modes mapped to their motor protocol register values.
MODES = {"mit": 1, "pv": 2, "pvt": 4}


def _resolve_config(config):
    return str(Path(config))


def _resolve_mode(mode):
    if not isinstance(mode, str) or mode.lower() not in MODES:
        raise ValueError("mode must be one of " + ", ".join(sorted(MODES)) + f"; got {mode!r}")
    return mode.lower()


class CanFD:
    def __init__(self, device_index=0, serial=None, lib_path=""):
        if not lib_path:
            packaged_lib = Path(__file__).with_name("libcontrolcanfd.so")
            lib_path = str(packaged_lib) if packaged_lib.exists() else ""
        self._native = _CanFD(device_index, "" if serial is None else serial, lib_path)

    def init(self, Abit=1000000, Bbit=5000000):
        return self._native.init(Abit, Bbit)

    def close(self):
        return self._native.close()

    @property
    def serial(self):
        return self._native.serial()

    @property
    def device_index(self):
        return self._native.device_index()

    @property
    def initialized(self):
        return self._native.initialized()


class Robot:
    def __init__(self, canfd, can_channel=1, freq=1000, config=None, soft_limit=True, mode="mit"):
        if config is None:
            raise ValueError("config must be provided by the user")
        self._mode = _resolve_mode(mode)
        native = canfd._native if isinstance(canfd, CanFD) else canfd
        # The native robot is created before _canfd on purpose: attributes are
        # released in creation order, and the native destructor sends the
        # disable frames, so the device must still be open when it runs.
        self._native = _Robot(
            native, can_channel, freq, _resolve_config(config), soft_limit, MODES[self._mode]
        )
        self._canfd = canfd

    def enable(self, cpu=None):
        """Start the control loop, optionally pinned to one CPU core.

        cpu is a zero based core index; None leaves the control thread free to
        run on any core. Pinning it to a dedicated core keeps the loop from being
        preempted by the rest of the system.
        """
        result = self._native.enable(-1 if cpu is None else cpu)
        _ACTIVE_ROBOTS.add(self)
        return result

    def disable(self):
        try:
            return self._native.disable()
        finally:
            _ACTIVE_ROBOTS.discard(self)

    def control_mit(self, id, kp, kd, p_des, v_des, t_ff):
        return self._native.control_mit(id, kp, kd, p_des, v_des, t_ff)

    def control_pv(self, id, p_des, v_des):
        return self._native.control_pv(id, p_des, v_des)

    def control_pvt(self, id, p_des, v_des, i_des):
        return self._native.control_pvt(id, p_des, v_des, i_des)

    def set_zero(self, id):
        return self._native.set_zero(id)

    def read_register(self, id, register):
        """Read a motor register and return its 32 bit little endian value."""
        return self._native.read_register(id, register)

    def read_timeout(self, id):
        """Read the motor's CAN feedback timeout in milliseconds."""
        return self._native.read_timeout(id)

    def write_register(self, id, register, value):
        """Write a motor register; returns False and sets last_error on failure."""
        return self._native.write_register(id, register, value)

    def write_timeout(self, id, timeout):
        """Write the motor's CAN feedback timeout in milliseconds."""
        if not isinstance(timeout, int) or timeout < 0:
            raise ValueError(f"timeout must be a non-negative int in milliseconds; got {timeout!r}")
        return self._native.write_timeout(id, timeout)

    def save_register(self, id, rid=0):
        """Persist a motor register to flash and wait for the motor's ack.

        rid 0 saves every register, a specific rid saves just that register.
        Returns False and records last_error when the motor does not answer.
        """
        if not isinstance(rid, int) or isinstance(rid, bool) or not 0 <= rid <= 0xFF:
            raise ValueError(f"rid must be an int in 0..0xFF; got {rid!r}")
        return self._native.save_register(id, rid)

    def getj(self):
        return self._native.getj()

    @property
    def running(self):
        return self._native.running()

    @property
    def mode(self):
        """Configured control mode: 'mit', 'pv' or 'pvt'."""
        return self._mode

    @property
    def last_error(self):
        return self._native.last_error()

    @property
    def motor_ids(self):
        return list(self._native.motor_ids())

__all__ = ["CanFD", "Robot", "MODES"]
