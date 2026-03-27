#!/usr/bin/env python3
"""Independent evaluation of the V3 canard tailsitter design.

Re-derives geometry, re-runs aerodynamic validation, compares against saved
artifacts, and performs additional sanity checks beyond the main validation.

Limitation: this evaluator re-uses the same AeroSandbox model, so it validates
internal consistency and physical reasonableness — not absolute aerodynamic
accuracy.  A truly independent check would require a different solver
(VSPAERO, XFLR5, or CFD).
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from v3.model.analysis import evaluate_validation
from v3.model.geometry import derive_geometry
from v3.model.spec import load_parameters, load_validation_targets

RESULTS_DIR = ROOT / "v3" / "results"
REPORT_PATH = RESULTS_DIR / "independent_evaluation_v3.md"


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _compare_geometry(derived: dict[str, float], saved: dict[str, float]) -> list[str]:
    """Compare re-derived geometry against saved artifact field-by-field."""
    lines: list[str] = []
    all_match = True
    for key in saved:
        expected = saved[key]
        actual = derived.get(key)
        if actual is None:
            lines.append(f"- MISSING: `{key}` not in re-derived geometry")
            all_match = False
            continue
        if abs(expected) > 1e-12:
            rel_err = abs(actual - expected) / abs(expected)
        else:
            rel_err = abs(actual - expected)
        ok = rel_err < 1e-6
        status = "MATCH" if ok else "MISMATCH"
        if not ok:
            all_match = False
        lines.append(f"- {status}: `{key}` saved={expected:.8g}, re-derived={actual:.8g}")
    if all_match:
        lines.insert(0, f"All {len(saved)} geometry fields match within 1e-6 relative tolerance.")
    else:
        lines.insert(0, "GEOMETRY MISMATCH DETECTED — see fields below.")
    return lines


def _compare_metrics(fresh: dict[str, Any], saved: dict[str, Any]) -> list[str]:
    """Compare re-computed validation metrics against saved artifact."""
    lines: list[str] = []
    all_match = True
    for section in saved:
        for key, expected in saved[section].items():
            actual = fresh.get(section, {}).get(key)
            if actual is None:
                lines.append(f"- MISSING: `{section}.{key}`")
                all_match = False
                continue
            if abs(expected) > 1e-12:
                rel_err = abs(actual - expected) / abs(expected)
            else:
                rel_err = abs(actual - expected)
            ok = rel_err < 1e-6
            if not ok:
                all_match = False
                lines.append(f"- MISMATCH: `{section}.{key}` saved={expected:.8g}, fresh={actual:.8g}")
    if all_match:
        count = sum(len(v) for v in saved.values())
        lines.insert(0, f"All {count} metrics match within 1e-6 relative tolerance.")
    else:
        lines.insert(0, "METRICS MISMATCH DETECTED — see fields below.")
    return lines


def _sanity_checks(metrics: dict[str, Any], parameters: Any) -> list[tuple[str, str, str]]:
    """Additional reasonableness checks beyond the main validation suite.

    Returns list of (check_name, status, detail).
    """
    checks: list[tuple[str, str, str]] = []
    g = metrics["geometry"]
    ff = metrics["forward_flight"]
    ce = metrics["control_effectiveness"]

    # 1. CL surplus margin
    cl_surplus = ff["CL"] - ff["required_cruise_cl"]
    cl_margin_pct = 100.0 * cl_surplus / ff["required_cruise_cl"]
    ok = cl_surplus > 0
    checks.append((
        "CL surplus at cruise",
        "OK" if ok else "WARN",
        f"CL={ff['CL']:.4f}, required={ff['required_cruise_cl']:.4f}, "
        f"surplus={cl_surplus:.4f} ({cl_margin_pct:.1f}%)"
    ))

    # 2. L/D at cruise
    ld = ff["CL"] / ff["CD"]
    ok = 5.0 < ld < 20.0
    checks.append((
        "Cruise L/D ratio",
        "OK" if ok else "WARN",
        f"L/D = {ld:.2f} (typical range for 2m-class UAV: 6-15)"
    ))

    # 3. Wing loading
    mass_kg = parameters.mass_properties.analysis_mass_kg
    wing_loading = mass_kg / g["wing_area_m2"]
    ok = 5.0 < wing_loading < 25.0
    checks.append((
        "Wing loading",
        "OK" if ok else "WARN",
        f"{wing_loading:.2f} kg/m^2 (typical for small UAV: 5-20 kg/m^2)"
    ))

    # 4. Static margin estimate
    mac = parameters.wing.stations[0].chord_m  # approximate
    # Better: use derived MAC from geometry
    # Cm_alpha = dCm/dalpha; static margin = -Cm_alpha / CL_alpha
    if abs(ff["cl_alpha"]) > 1e-6:
        static_margin = -ff["cm_alpha"] / ff["cl_alpha"]
        sm_pct = 100.0 * static_margin
        ok = 0.05 < static_margin < 0.40
        checks.append((
            "Static margin estimate",
            "OK" if ok else "WARN",
            f"{sm_pct:.1f}% of reference chord "
            f"(from -Cm_alpha/CL_alpha = -{ff['cm_alpha']:.4f}/{ff['cl_alpha']:.4f})"
        ))

    # 5. Control power ratio: canard vs elevon
    ratio = ce["cm_delta_canard_per_rad"] / ce["cm_delta_elevon_sym_per_rad"] if abs(ce["cm_delta_elevon_sym_per_rad"]) > 1e-8 else float("inf")
    checks.append((
        "Canard/elevon pitch authority ratio",
        "INFO",
        f"{ratio:.1f}:1 canard-to-elevon (unpowered). "
        f"Canard={ce['cm_delta_canard_per_rad']:.4f}/rad, "
        f"elevon={ce['cm_delta_elevon_sym_per_rad']:.4f}/rad"
    ))

    # 6. Powered vs unpowered elevon comparison
    powered_ratio = ce["cm_delta_elevon_sym_powered_gain_ratio"]
    expected_gain = parameters.power_effects.elevon_dynamic_pressure_gain_powered
    gain_match = abs(powered_ratio - expected_gain) < 0.01
    checks.append((
        "Powered elevon gain consistency",
        "OK" if gain_match else "WARN",
        f"Computed gain={powered_ratio:.3f}, specified gain={expected_gain:.3f}"
    ))

    # 7. Wake clearance margin above minimum
    wake_min_target = 0.04
    wake_actual = g["wake_clearance_m"]
    wake_margin = wake_actual - wake_min_target
    wake_margin_pct = 100.0 * wake_margin / wake_min_target
    ok = wake_margin > 0
    checks.append((
        "Wake clearance margin",
        "OK" if ok else "FAIL",
        f"Actual={wake_actual:.4f} m, minimum={wake_min_target:.4f} m, "
        f"margin={wake_margin:.4f} m ({wake_margin_pct:.1f}%)"
    ))

    # 8. Canard radial clearance margin
    radial_min = 0.03
    radial_actual = g["canard_to_prop_radial_clearance_m"]
    radial_margin_pct = 100.0 * (radial_actual - radial_min) / radial_min
    checks.append((
        "Canard radial clearance margin",
        "OK" if radial_actual >= radial_min else "FAIL",
        f"Actual={radial_actual:.4f} m, minimum={radial_min:.4f} m, "
        f"margin={radial_margin_pct:.1f}%"
    ))

    # 9. Trim canard vs available authority
    trim_abs = abs(ff["trim_canard_deg"])
    forward_limit = parameters.canard.deflection_limits_deg.forward_command_abs
    trim_usage_pct = 100.0 * trim_abs / forward_limit
    checks.append((
        "Trim authority usage",
        "OK" if trim_usage_pct < 75 else "WARN",
        f"Trim uses {trim_abs:.2f} deg of {forward_limit:.1f} deg forward limit "
        f"({trim_usage_pct:.1f}% of authority)"
    ))

    # 10. Directional stability adequacy
    cn_beta = ff["cn_beta"]
    ok = cn_beta > 0.02
    checks.append((
        "Directional stability adequacy",
        "OK" if ok else "WARN",
        f"Cn_beta = {cn_beta:.5f} /rad "
        f"({'adequate' if cn_beta > 0.02 else 'marginal, consider larger fins'})"
    ))

    # 11. Roll damping adequacy
    cl_p = ff["cl_p"]
    ok = cl_p < -0.3
    checks.append((
        "Roll damping adequacy",
        "OK" if ok else "WARN",
        f"Cl_p = {cl_p:.4f} /rad (strong negative = good damping)"
    ))

    # 12. Cruise alpha assessment
    cruise_alpha = parameters.analysis_conditions.cruise_alpha_deg
    ok = 2.0 <= cruise_alpha <= 10.0
    checks.append((
        "Cruise alpha reasonableness",
        "OK" if ok else "WARN",
        f"alpha_cruise = {cruise_alpha:.1f} deg "
        f"(chosen to achieve CL={ff['CL']:.3f} at {parameters.analysis_conditions.cruise_speed_m_s} m/s)"
    ))

    return checks


def main() -> int:
    parameters = load_parameters(ROOT / "spec" / "parameters_v3.json")
    targets = load_validation_targets(ROOT / "spec" / "validation_targets_v3.json")

    # Re-derive geometry
    derived = derive_geometry(parameters)
    derived_dict = asdict(derived)

    # Load saved artifacts for comparison
    saved_geometry = _load_json(RESULTS_DIR / "derived_geometry_v3.json")
    saved_metrics = _load_json(RESULTS_DIR / "validation_metrics_v3.json")

    # Re-run full validation
    _, fresh_metrics, report_text, _, interference = evaluate_validation(parameters, targets, ROOT)

    # Compare geometry
    geometry_lines = _compare_geometry(derived_dict, saved_geometry)

    # Compare metrics
    metrics_lines = _compare_metrics(fresh_metrics, saved_metrics)

    # Count validation pass/fail from the report
    pass_count = report_text.count("PASS")
    fail_count = report_text.count("FAIL")

    # Run sanity checks
    sanity_checks = _sanity_checks(fresh_metrics, parameters)

    # Build report
    lines: list[str] = []
    lines.append("# Independent Evaluation Report — V3 Canard Tailsitter")
    lines.append("")
    lines.append("## 1. Artifact Reproducibility")
    lines.append("")
    lines.append("### Geometry Comparison")
    lines.append("Re-derived geometry vs saved `derived_geometry_v3.json`:")
    lines.append("")
    for line in geometry_lines:
        lines.append(line)
    lines.append("")
    lines.append("### Metrics Comparison")
    lines.append("Re-computed validation metrics vs saved `validation_metrics_v3.json`:")
    lines.append("")
    for line in metrics_lines:
        lines.append(line)
    lines.append("")

    lines.append("## 2. Validation Gate")
    lines.append("")
    lines.append(f"Standard validation: **{pass_count} pass / {fail_count} fail**")
    lines.append("")

    lines.append("## 3. Design Sanity Checks")
    lines.append("")
    lines.append("| # | Check | Status | Detail |")
    lines.append("|---|-------|--------|--------|")
    for i, (name, status, detail) in enumerate(sanity_checks, 1):
        lines.append(f"| {i} | {name} | {status} | {detail} |")
    lines.append("")

    # Summary
    warn_count = sum(1 for _, s, _ in sanity_checks if s == "WARN")
    fail_sanity = sum(1 for _, s, _ in sanity_checks if s == "FAIL")
    ok_count = sum(1 for _, s, _ in sanity_checks if s == "OK")
    info_count = sum(1 for _, s, _ in sanity_checks if s == "INFO")

    lines.append("## 4. Summary")
    lines.append("")
    lines.append(f"- **Artifact reproducibility:** {'PASS' if 'MISMATCH' not in ' '.join(geometry_lines + metrics_lines) else 'FAIL'}")
    lines.append(f"- **Validation gate:** {pass_count}/{pass_count + fail_count} checks pass")
    lines.append(f"- **Sanity checks:** {ok_count} OK, {info_count} INFO, {warn_count} WARN, {fail_sanity} FAIL (of {len(sanity_checks)} total)")
    lines.append("")

    lines.append("## 5. Limitations")
    lines.append("")
    lines.append("- This evaluation re-uses the same AeroSandbox AeroBuildup solver as the")
    lines.append("  primary validation pipeline. Systematic model errors are not detectable.")
    lines.append("- Powered elevon effectiveness uses a gain proxy (dynamic pressure multiplier),")
    lines.append("  not full slipstream-resolved aerodynamics.")
    lines.append("- Wake clearance is computed via geometric projection, not CFD or panel method.")
    lines.append("- For absolute confidence, validate with VSPAERO (bundled in tools/), XFLR5, or CFD.")
    lines.append("")

    report = "\n".join(lines)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report, encoding="utf-8")

    print(f"Validation: {pass_count} pass / {fail_count} fail")
    print(f"Sanity checks: {ok_count} OK, {info_count} INFO, {warn_count} WARN, {fail_sanity} FAIL")
    print(f"Wrote {REPORT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
