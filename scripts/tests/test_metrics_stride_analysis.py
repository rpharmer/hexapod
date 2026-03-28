#!/usr/bin/env python3
"""Regression tests for scripts/metrics_stride_analysis.py."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "scripts" / "metrics_stride_analysis.py"


class MetricsStrideAnalysisTests(unittest.TestCase):
    def run_script(self, input_path: Path) -> dict:
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = Path(tmpdir) / "metrics.json"
            subprocess.run(
                [
                    "python3",
                    str(SCRIPT_PATH),
                    "--input",
                    str(input_path),
                    "--json-output",
                    str(output_path),
                ],
                check=True,
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
            )
            return json.loads(output_path.read_text(encoding="utf-8"))

    def test_geometry_carries_forward_across_frames(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            capture = Path(tmpdir) / "capture.ndjson"
            frames = [
                {
                    "captured_at_unix_ms": 1,
                    "datagram": {
                        "schema_version": 1,
                        "timestamp_ms": 1000,
                        "geometry": {"coxa": 35.0, "femur": 70.0, "tibia": 110.0, "body_radius": 60.0},
                        "angles_deg": {
                            "LF": [0.0, 20.0, -40.0],
                            "LM": [0.0, 20.0, -40.0],
                            "LR": [0.0, 20.0, -40.0],
                            "RF": [0.0, 20.0, -40.0],
                            "RM": [0.0, 20.0, -40.0],
                            "RR": [0.0, 20.0, -40.0],
                        },
                    },
                },
                {
                    "captured_at_unix_ms": 2,
                    "datagram": {
                        "schema_version": 1,
                        "timestamp_ms": 1033,
                        # No geometry in this frame; analyzer should reuse prior geometry.
                        "angles_deg": {
                            "LF": [1.0, 20.0, -40.0],
                            "LM": [1.0, 20.0, -40.0],
                            "LR": [1.0, 20.0, -40.0],
                            "RF": [1.0, 20.0, -40.0],
                            "RM": [1.0, 20.0, -40.0],
                            "RR": [1.0, 20.0, -40.0],
                        },
                    },
                },
            ]
            capture.write_text("\n".join(json.dumps(frame) for frame in frames) + "\n", encoding="utf-8")
            result = self.run_script(capture)

            for leg in ("LF", "LM", "LR", "RF", "RM", "RR"):
                self.assertGreaterEqual(
                    result["per_leg"][leg]["sample_count"],
                    2,
                    f"expected carry-forward geometry to preserve {leg} samples",
                )

    def test_missing_capture_errors(self) -> None:
        missing = REPO_ROOT / ".tmp" / "does_not_exist.ndjson"
        proc = subprocess.run(
            ["python3", str(SCRIPT_PATH), "--input", str(missing)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("missing/empty capture", proc.stderr)


if __name__ == "__main__":
    unittest.main()
