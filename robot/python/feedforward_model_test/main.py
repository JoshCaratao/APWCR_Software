"""
Feedforward motor model identification test runner.

Purpose:
This module runs a dedicated serial-based sweep test against the separate
feedforward test firmware. It commands one selected motor channel through a
configured positive and/or negative sweep, waits for each step to settle, and
logs the returned output RPM telemetry to a CSV file for later analysis.

Responsibilities:
- Load the dedicated feedforward-model-test YAML config
- Build the ordered sweep plan from config
- Open the serial link to the test firmware
- Send motor command steps for the selected channel
- Log telemetry samples during each measurement window
- Save raw command/RPM data to CSV for later analysis in Excel

Not responsible for:
- Robot autonomy or GUI control
- Plotting or regression fitting
- Running the deployed robot firmware/runtime
"""

from __future__ import annotations

import time

from feedforward_model_test.config_loader import get_project_root, load_config, require_keys
from feedforward_model_test.csv_logger import CsvLogger
from feedforward_model_test.serial_link import SerialLink
from feedforward_model_test.sweep_plan import commands_for_step
from feedforward_model_test.sweep_runner import SweepRunner


def _write_logged_sample(
    *,
    csv_logger: CsvLogger,
    telemetry: dict,
    step,
    step_phase: str,
    command_mode: str,
) -> None:
    """Write one sampled telemetry row to the CSV with step metadata."""
    csv_logger.write_row({
        "host_time_s": telemetry.get("host_time_s"),
        "arduino_time_ms": telemetry.get("arduino_time_ms"),
        "seq": telemetry.get("ack_seq"),
        "motor_under_test": step.motor_under_test,
        "sweep_direction": step.sweep_direction,
        "step_index": step.step_index,
        "step_phase": step_phase,
        "step_target_command": step.step_target_command,
        "command_mode": command_mode,
        "drive_lhs_cmd": telemetry.get("drive_lhs_cmd"),
        "drive_rhs_cmd": telemetry.get("drive_rhs_cmd"),
        "mech_rhs_cmd": telemetry.get("mech_rhs_cmd"),
        "mech_lhs_cmd": telemetry.get("mech_lhs_cmd"),
        "drive_lhs_rpm": telemetry.get("drive_lhs_rpm"),
        "drive_rhs_rpm": telemetry.get("drive_rhs_rpm"),
        "mech_rhs_rpm": telemetry.get("mech_rhs_rpm"),
        "mech_lhs_rpm": telemetry.get("mech_lhs_rpm"),
    })


def _drain_until(deadline_s: float, serial_link: SerialLink) -> None:
    """Read and discard telemetry until the requested deadline."""
    while time.perf_counter() < deadline_s:
        serial_link.read_telemetry()


def _log_until(
    *,
    deadline_s: float,
    serial_link: SerialLink,
    csv_logger: CsvLogger,
    step,
    step_phase: str,
    command_mode: str,
    loop_period_s: float,
) -> int:
    """Read telemetry until the deadline and log each decoded sample."""
    rows_written = 0
    next_tick_s = time.perf_counter()
    while time.perf_counter() < deadline_s:
        telemetry = serial_link.read_telemetry()
        if telemetry is None:
            _sleep_to_next_tick(next_tick_s, loop_period_s)
            next_tick_s += loop_period_s
            continue
        _write_logged_sample(
            csv_logger=csv_logger,
            telemetry=telemetry,
            step=step,
            step_phase=step_phase,
            command_mode=command_mode,
        )
        rows_written += 1
        _sleep_to_next_tick(next_tick_s, loop_period_s)
        next_tick_s += loop_period_s
    return rows_written


def _run_for_duration(
    *,
    duration_s: float,
    serial_link: SerialLink,
    active_commands: dict[str, float],
    command_period_s: float,
    telemetry_period_s: float,
    csv_logger: CsvLogger | None = None,
    step=None,
    step_phase: str | None = None,
    command_mode: str | None = None,
) -> int:
    """Run one phase with separate command refresh and telemetry poll rates."""
    if duration_s <= 0.0:
        return 0

    rows_written = 0
    deadline_s = time.perf_counter() + duration_s
    next_command_s = time.perf_counter()
    next_telemetry_s = next_command_s

    while time.perf_counter() < deadline_s:
        now_s = time.perf_counter()

        if now_s >= next_command_s:
            serial_link.send_commands(active_commands)
            next_command_s += command_period_s

        if now_s >= next_telemetry_s:
            telemetry = serial_link.read_telemetry()

            if (
                telemetry is not None
                and csv_logger is not None
                and step is not None
                and step_phase is not None
                and command_mode is not None
            ):
                _write_logged_sample(
                    csv_logger=csv_logger,
                    telemetry=telemetry,
                    step=step,
                    step_phase=step_phase,
                    command_mode=command_mode,
                )
                rows_written += 1

            next_telemetry_s += telemetry_period_s

        next_event_s = min(next_command_s, next_telemetry_s)
        _sleep_to_next_time(next_event_s)

    return rows_written


def _sleep_to_next_time(next_time_s: float) -> None:
    """Sleep until the next scheduled event when time remains."""
    sleep_s = next_time_s - time.perf_counter()
    if sleep_s > 0.0:
        time.sleep(sleep_s)


def main(config_name: str = "robot_ff_model_test.yaml") -> None:
    # Load and validate the dedicated feedforward-model-test config.
    cfg = load_config(config_name)
    require_keys(cfg, {
        "comms": ["port", "baud", "timeout_s", "write_timeout_s"],
        "test": [
            "motor_under_test",
            "command_mode",
            "sweep_direction",
            "command_hz",
            "telemetry_hz",
            "settle_s",
            "sample_s",
            "stop_between_steps_s",
            "positive_sweep",
            "negative_sweep",
        ],
        "logging": ["output_dir", "file_stem", "append_timestamp"],
    })

    comms_cfg = cfg.get("comms", {})
    test_cfg = cfg.get("test", {})
    log_cfg = cfg.get("logging", {})
    project_root = get_project_root()
    command_mode = str(test_cfg["command_mode"])
    command_hz = float(test_cfg["command_hz"])
    telemetry_hz = float(test_cfg["telemetry_hz"])
    command_period_s = 1.0 / command_hz
    telemetry_period_s = 1.0 / telemetry_hz

    # Build CSV output and the ordered sweep plan from config.
    csv_logger = CsvLogger(
        project_root=project_root,
        output_dir=log_cfg["output_dir"],
        file_stem=log_cfg["file_stem"],
        append_timestamp=bool(log_cfg["append_timestamp"]),
    )
    csv_logger.open()
    sweep_runner = SweepRunner(test_cfg)

    # Print the planned run so settings are easy to verify before motion.
    print("Feedforward Model Test")
    print("----------------------")
    print(f"Config: {config_name}")
    print(f"Port: {comms_cfg.get('port')}")
    print(f"Baud: {comms_cfg.get('baud')}")
    print(f"Motor Under Test: {test_cfg.get('motor_under_test')}")
    print(f"Command Mode: {test_cfg.get('command_mode')}")
    print(f"Sweep Direction: {test_cfg.get('sweep_direction')}")
    print(f"Command Hz: {command_hz}")
    print(f"Telemetry Hz: {telemetry_hz}")
    print(f"Settle Time (s): {test_cfg.get('settle_s')}")
    print(f"Sample Time (s): {test_cfg.get('sample_s')}")
    print(f"Stop Between Steps (s): {test_cfg.get('stop_between_steps_s')}")
    print(f"CSV Output Dir: {log_cfg.get('output_dir')}")
    print(f"CSV File Stem: {log_cfg.get('file_stem')}")
    print(f"Append Timestamp: {log_cfg.get('append_timestamp')}")
    print(f"CSV Path: {csv_logger.path}")
    print("CSV header created.")
    print(f"Planned Steps: {len(sweep_runner.steps)}")
    print(f"Estimated Runtime (s): {sweep_runner.total_planned_runtime_s():.2f}")

    if sweep_runner.steps:
        print("Planned Sweep:")
        for step in sweep_runner.steps:
            print(f"  {sweep_runner.describe_step(step)}")

    # Run the sweep over serial and log sampled telemetry rows to CSV.
    serial_link = SerialLink(comms_cfg)
    rows_written = 0

    try:
        serial_link.open()
        serial_link.send_commands(serial_link.zero_commands())
        print("Serial link opened. Zero command sent.")

        for step in sweep_runner.steps:
            print(f"Running: {sweep_runner.describe_step(step)}")

            if step.stop_between_steps_s > 0.0:
                # Optional zero-command pause between commanded steps.
                _run_for_duration(
                    duration_s=step.stop_between_steps_s,
                    serial_link=serial_link,
                    active_commands=serial_link.zero_commands(),
                    command_period_s=command_period_s,
                    telemetry_period_s=telemetry_period_s,
                )

            step_commands = commands_for_step(step)

            # Log the transient response during the settle window.
            rows_written += _run_for_duration(
                duration_s=step.settle_s,
                serial_link=serial_link,
                active_commands=step_commands,
                command_period_s=command_period_s,
                telemetry_period_s=telemetry_period_s,
                csv_logger=csv_logger,
                step=step,
                step_phase="settle",
                command_mode=command_mode,
            )

            # Log all decoded telemetry samples during the measurement window.
            rows_written += _run_for_duration(
                duration_s=step.sample_s,
                serial_link=serial_link,
                active_commands=step_commands,
                command_period_s=command_period_s,
                telemetry_period_s=telemetry_period_s,
                csv_logger=csv_logger,
                step=step,
                step_phase="sample",
                command_mode=command_mode,
            )

        serial_link.send_commands(serial_link.zero_commands())
        print("Sweep complete. Zero command sent.")
        print(f"Rows Logged: {rows_written}")

    finally:
        # Always send zero on exit so the test leaves motors in a safe state.
        try:
            serial_link.send_commands(serial_link.zero_commands())
        except Exception:
            pass
        serial_link.close()
        print("Serial link closed.")


if __name__ == "__main__":
    main()
