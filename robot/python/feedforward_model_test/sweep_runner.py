from __future__ import annotations

from feedforward_model_test.sweep_plan import SweepStep, build_sweep_steps, commands_for_step


class SweepRunner:
    """Container for the planned sweep sequence and per-step command maps."""

    def __init__(self, test_cfg: dict) -> None:
        """Build and store the sweep steps from the test config."""
        self.steps = build_sweep_steps(test_cfg)

    def planned_commands(self) -> list[dict[str, float]]:
        """Return the command dictionary for each planned sweep step."""
        return [commands_for_step(step) for step in self.steps]

    def total_planned_runtime_s(self) -> float:
        """Estimate the full runtime of the sweep based on step timings."""
        total = 0.0
        for step in self.steps:
            total += step.stop_between_steps_s
            total += step.settle_s
            total += step.sample_s
        return total

    def describe_step(self, step: SweepStep) -> str:
        """Create a short human-readable summary for one sweep step."""
        return (
            f"step={step.step_index} "
            f"motor={step.motor_under_test} "
            f"dir={step.sweep_direction} "
            f"cmd={step.step_target_command:.3f} "
            f"stop={step.stop_between_steps_s:.2f}s "
            f"settle={step.settle_s:.2f}s "
            f"sample={step.sample_s:.2f}s"
        )
