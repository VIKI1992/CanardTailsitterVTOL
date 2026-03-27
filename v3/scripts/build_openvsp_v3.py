#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from v3.model.geometry import derive_geometry, write_derived_geometry
from v3.model.openvsp_builder import build_vspscript, export_vsp3, find_script_runner
from v3.model.spec import load_parameters


def main() -> int:
    parameters = load_parameters(ROOT / "spec" / "parameters_v3.json")
    derived = derive_geometry(parameters)
    results_dir = ROOT / "v3" / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    script_path = results_dir / "fixed_wing_tailsitter_v3.vspscript"
    vsp3_path = results_dir / "fixed_wing_tailsitter_v3.vsp3"
    script_text = build_vspscript(parameters, vsp3_path.relative_to(ROOT).as_posix())
    script_path.write_text(script_text, encoding="utf-8")
    write_derived_geometry(results_dir / "derived_geometry_v3.json", derived)

    runner = find_script_runner(ROOT)
    if runner is None:
        raise RuntimeError("OpenVSP script runner not found under tools/.")
    result = export_vsp3(ROOT, runner, script_path)
    if result.returncode != 0 and not vsp3_path.exists():
        details = result.stderr.strip() or result.stdout.strip() or f"exit code {result.returncode}"
        raise RuntimeError(f"OpenVSP export failed: {details}")

    print(f"Wrote {script_path}")
    print(f"Wrote {vsp3_path}")
    print(f"Used {runner}")
    print(f"Wing area {derived.wing_area_m2:.6f} m^2")
    print(f"Wake clearance {derived.wake_clearance_m:.6f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
