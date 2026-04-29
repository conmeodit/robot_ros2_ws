"""Build command strings for the Arduino Mega serial protocol.

Each command is a single line terminated by ``\\n``.
"""


def cmd_vel(linear: float, angular: float) -> str:
    """Differential-drive velocity command.

    Args:
        linear: Forward speed (m/s, positive = forward).
        angular: Turning rate (rad/s, positive = counter-clockwise).

    Returns:
        ``CMD_VEL,{v:.4f},{w:.4f}\\n``
    """
    return f'CMD_VEL,{linear:.4f},{angular:.4f}\n'


def estop(active: bool) -> str:
    """Emergency stop.

    Args:
        active: ``True`` to engage e-stop, ``False`` to release.

    Returns:
        ``ESTOP,1\\n`` or ``ESTOP,0\\n``
    """
    return f'ESTOP,{1 if active else 0}\n'


def motor_enable(enabled: bool) -> str:
    """Enable or disable motors.

    Args:
        enabled: ``True`` to enable, ``False`` to disable.

    Returns:
        ``MOTOR,1\\n`` or ``MOTOR,0\\n``
    """
    return f'MOTOR,{1 if enabled else 0}\n'


def servo(servo_id: int, degrees: int) -> str:
    """Set a servo angle.

    Args:
        servo_id: Servo index (1–5).
        degrees: Target angle 0–180.

    Returns:
        ``SERVO,{id},{deg}\\n``
    """
    degrees = max(0, min(180, degrees))
    return f'SERVO,{servo_id},{degrees}\n'


def stop() -> str:
    """Immediate stop (single-char command).

    Returns:
        ``S\\n``
    """
    return 'S\n'
