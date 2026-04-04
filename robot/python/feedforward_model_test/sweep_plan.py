from __future__ import annotations

from dataclasses import dataclass

from feedforward_model_test.protocol import MOTOR_FIELDS


@dataclass(frozen=True)
class SweepStep:
    """One planned sweep step with timing and command metadata."""

    step_index: int
    motor_under_test: str
    sweep_direction: str
    step_target_command: float
    settle_s: float
    sample_s: float
    stop_between_steps_s: float


def build_sweep_steps(test_cfg: dict) -> list[SweepStep]:
    """Build the ordered sweep steps from the test config."""
    motor_under_test = str(test_cfg["motor_under_test"])
    _require_supported_motor(motor_under_test)

    sweep_direction = str(test_cfg["sweep_direction"]).lower()
    settle_s = float(test_cfg["settle_s"])
    sample_s = float(test_cfg["sample_s"])
    sweep_cfg = _resolve_sweep_config(test_cfg)
    stop_between_steps_s = float(sweep_cfg["stop_between_steps_s"])

    commands: list[tuple[str, float]] = []
    if sweep_direction in ("positive", "both"):
        commands.extend(("positive", float(v)) for v in sweep_cfg["positive_sweep"])
    if sweep_direction in ("negative", "both"):
        commands.extend(("negative", float(v)) for v in sweep_cfg["negative_sweep"])

    if sweep_direction not in ("positive", "negative", "both"):
        raise ValueError("sweep_direction must be 'positive', 'negative', or 'both'")

    steps: list[SweepStep] = []
    for idx, (direction, value) in enumerate(commands):
        steps.append(
            SweepStep(
                step_index=idx,
                motor_under_test=motor_under_test,
                sweep_direction=direction,
                step_target_command=value,
                settle_s=settle_s,
                sample_s=sample_s,
                stop_between_steps_s=stop_between_steps_s,
            )
        )

    return steps


def commands_for_step(step: SweepStep) -> dict[str, float]:
    """Build the full four-channel command dictionary for one sweep step."""
    commands = zero_commands()
    commands[step.motor_under_test] = float(step.step_target_command)
    return commands


def zero_commands() -> dict[str, float]:
    """Return a zero-command dictionary for all supported motor channels."""
    return {motor: 0.0 for motor in MOTOR_FIELDS}


def _require_supported_motor(motor_under_test: str) -> None:
    """Reject unknown motor names early so the test plan is explicit."""
    if motor_under_test not in MOTOR_FIELDS:
        raise ValueError(
            f"motor_under_test must be one of {sorted(MOTOR_FIELDS)}, got '{motor_under_test}'"
        )


def _resolve_sweep_config(test_cfg: dict) -> dict:
    """Return the active sweep configuration based on stop-between-steps."""
    if "positive_sweep_with_stop" not in test_cfg:
        return {
            "stop_between_steps_s": float(test_cfg["stop_between_steps_s"]),
            "positive_sweep": list(test_cfg["positive_sweep"]),
            "negative_sweep": list(test_cfg["negative_sweep"]),
        }

    stop_between_steps_s = float(test_cfg["stop_between_steps_s"])
    use_stop_sweeps = stop_between_steps_s > 0.0

    pos_key = "positive_sweep_with_stop" if use_stop_sweeps else "positive_sweep_no_stop"
    neg_key = "negative_sweep_with_stop" if use_stop_sweeps else "negative_sweep_no_stop"

    for key in (pos_key, neg_key):
        if key not in test_cfg:
            raise KeyError(f"Missing config key: 'test.{key}'")

    return {
        "stop_between_steps_s": stop_between_steps_s,
        "positive_sweep": list(test_cfg[pos_key]),
        "negative_sweep": list(test_cfg[neg_key]),
    }
