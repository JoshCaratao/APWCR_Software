from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path


class CsvLogger:
    """
    Small CSV logger for feedforward model test data.

    Responsibilities:
    - Build the output CSV path from config
    - Create parent directories if needed
    - Write a header once
    - Append rows in a consistent column order
    """

    FIELDNAMES = [
        "host_time_s",
        "arduino_time_ms",
        "seq",
        "motor_under_test",
        "sweep_direction",
        "step_index",
        "step_phase",
        "step_target_command",
        "command_mode",
        "motor_command_echo",
        "motor_measured_rpm",
    ]

    def __init__(self, *, project_root: Path, output_dir: str, file_stem: str, append_timestamp: bool) -> None:
        """Resolve the CSV output path from config values."""
        self._project_root = Path(project_root)
        self._output_dir = self._project_root / output_dir

        suffix = ""
        if append_timestamp:
            suffix = "_" + datetime.now().strftime("%Y%m%d_%H%M%S")

        self.path = self._output_dir / f"{file_stem}{suffix}.csv"

    def open(self) -> None:
        """Create the output directory and write the CSV header row."""
        self._output_dir.mkdir(parents=True, exist_ok=True)

        with open(self.path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            writer.writeheader()

    def write_row(self, row: dict) -> None:
        """Append one data row to the CSV file."""
        with open(self.path, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            writer.writerow(row)
