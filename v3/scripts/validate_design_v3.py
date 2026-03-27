#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from v3.model.analysis import evaluate_validation, write_validation_outputs
from v3.model.spec import load_parameters, load_validation_targets


def main() -> int:
    parameters = load_parameters(ROOT / "spec" / "parameters_v3.json")
    targets = load_validation_targets(ROOT / "spec" / "validation_targets_v3.json")
    derived, metrics, report, control_report, interference = evaluate_validation(parameters, targets, ROOT)
    results_dir = ROOT / "v3" / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    write_validation_outputs(
        metrics_path=results_dir / "validation_metrics_v3.json",
        report_path=results_dir / "validation_report_v3.md",
        control_report_path=results_dir / "control_authority_report_v3.md",
        interference_path=results_dir / "interference_sweep_v3.json",
        metrics=metrics,
        report=report,
        control_report=control_report,
        interference_sweep=interference,
    )
    print(f"Wing area {derived.wing_area_m2:.6f} m^2")
    print(f"Wrote {results_dir / 'validation_report_v3.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
