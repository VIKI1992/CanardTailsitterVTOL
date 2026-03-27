from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import aerosandbox as asb
import numpy as np

from .geometry import DerivedGeometry, derive_geometry
from .spec import DesignParameters, SurfaceStation


@dataclass(frozen=True)
class AeroSnapshot:
    CL: float
    CD: float
    Cm: float
    CY: float
    Cl: float
    Cn: float


def _scalar(value: Any) -> float:
    array = np.asarray(value)
    return float(array.reshape(-1)[0])


def _load_airfoil(identifier: str, repo_root: Path) -> asb.Airfoil:
    normalized = identifier.strip().upper().replace(" ", "")
    if normalized.startswith("NACA"):
        return asb.Airfoil(name=normalized.lower())
    if normalized == "SD7037":
        coords = []
        for line in (repo_root / "airfoils" / "sd7037.dat").read_text(encoding="utf-8").splitlines()[1:]:
            parts = line.split()
            if len(parts) >= 2:
                coords.append([float(parts[0]), float(parts[1])])
        return asb.Airfoil(name="SD7037", coordinates=np.array(coords))
    raise ValueError(f"Unsupported airfoil {identifier}")


def _mirror(station: SurfaceStation) -> SurfaceStation:
    return SurfaceStation(
        name=station.name,
        leading_edge_x_m=station.leading_edge_x_m,
        y_m=-station.y_m,
        z_m=station.z_m,
        chord_m=station.chord_m,
        twist_deg=station.twist_deg,
    )


def _elevon_weight(y_abs_m: float, start_y_m: float, end_y_m: float) -> float:
    if y_abs_m <= start_y_m:
        return 0.0
    if y_abs_m >= end_y_m:
        return 1.0
    return (y_abs_m - start_y_m) / (end_y_m - start_y_m)


def _half_wing(
    name: str,
    stations: list[SurfaceStation],
    side: str,
    airfoil_names: list[str],
    elevon_sym_deg: float = 0.0,
    elevon_diff_deg: float = 0.0,
    elevon_start_y_m: float = 0.0,
    elevon_end_y_m: float = 0.0,
    repo_root: Path | None = None,
    control_power_factor: float = 1.0,
) -> asb.Wing:
    sign = 1.0 if side == "right" else -1.0
    side_deflection = elevon_sym_deg + sign * elevon_diff_deg
    xsecs = []
    for station, airfoil_name in zip(stations, airfoil_names):
        base = station if side == "right" else _mirror(station)
        weight = _elevon_weight(abs(base.y_m), elevon_start_y_m, elevon_end_y_m)
        xsecs.append(
            asb.WingXSec(
                xyz_le=[base.leading_edge_x_m, base.y_m, base.z_m],
                chord=base.chord_m,
                twist=base.twist_deg + weight * side_deflection * control_power_factor,
                airfoil=_load_airfoil(airfoil_name, repo_root) if repo_root is not None else asb.Airfoil("naca0012"),
            )
        )
    return asb.Wing(name=name, symmetric=False, xsecs=xsecs)


def build_airplane(
    parameters: DesignParameters,
    derived: DerivedGeometry,
    repo_root: Path,
    canard_deflection_deg: float = 0.0,
    elevon_sym_deg: float = 0.0,
    elevon_diff_deg: float = 0.0,
) -> asb.Airplane:
    elevon = parameters.control_surfaces["elevon_left"]
    control_power = 4.0 * (elevon.chord_fraction / 0.24)
    n_wing = len(parameters.wing.stations)
    n_main = max(1, n_wing - 3)
    wing_airfoils = [parameters.wing.airfoil] * n_main + ["NACA0009"] * (n_wing - n_main)
    wings = [
        _half_wing(
            name="MainWingRight",
            stations=parameters.wing.stations,
            side="right",
            airfoil_names=wing_airfoils,
            elevon_sym_deg=elevon_sym_deg,
            elevon_diff_deg=elevon_diff_deg,
            elevon_start_y_m=elevon.start_y_m,
            elevon_end_y_m=elevon.end_y_m,
            repo_root=repo_root,
            control_power_factor=control_power,
        ),
        _half_wing(
            name="MainWingLeft",
            stations=parameters.wing.stations,
            side="left",
            airfoil_names=wing_airfoils,
            elevon_sym_deg=elevon_sym_deg,
            elevon_diff_deg=elevon_diff_deg,
            elevon_start_y_m=elevon.start_y_m,
            elevon_end_y_m=elevon.end_y_m,
            repo_root=repo_root,
            control_power_factor=control_power,
        ),
        asb.Wing(
            name="Canard",
            symmetric=True,
            xsecs=[
                asb.WingXSec(
                    xyz_le=[station.leading_edge_x_m, station.y_m, station.z_m],
                    chord=station.chord_m,
                    twist=station.twist_deg + canard_deflection_deg,
                    airfoil=_load_airfoil(parameters.canard.airfoil, repo_root),
                )
                for station in parameters.canard.stations
            ],
        ),
    ]
    fuselage = asb.Fuselage(
        name="Fuselage",
        xsecs=[
            asb.FuselageXSec(
                xyz_c=[station.x_m, 0.0, 0.0],
                width=station.width_m,
                height=station.height_m,
            )
            for station in parameters.fuselage.stations
        ],
    )
    return asb.Airplane(
        name=parameters.metadata.name,
        xyz_ref=[parameters.mass_properties.cg_x_m, 0.0, 0.0],
        s_ref=derived.wing_area_m2,
        c_ref=derived.wing_mac_m,
        b_ref=derived.wing_projected_span_m,
        wings=wings,
        fuselages=[fuselage],
    )


def run_aero(
    parameters: DesignParameters,
    derived: DerivedGeometry,
    repo_root: Path,
    velocity_m_s: float,
    alpha_deg: float,
    beta_deg: float = 0.0,
    p_rad_s: float = 0.0,
    q_rad_s: float = 0.0,
    r_rad_s: float = 0.0,
    canard_deflection_deg: float = 0.0,
    elevon_sym_deg: float = 0.0,
    elevon_diff_deg: float = 0.0,
) -> AeroSnapshot:
    op_point = asb.OperatingPoint(
        atmosphere=asb.Atmosphere(altitude=0.0),
        velocity=velocity_m_s,
        alpha=alpha_deg,
        beta=beta_deg,
        p=p_rad_s,
        q=q_rad_s,
        r=r_rad_s,
    )
    solver = asb.AeroBuildup(
        airplane=build_airplane(
            parameters,
            derived,
            repo_root,
            canard_deflection_deg=canard_deflection_deg,
            elevon_sym_deg=elevon_sym_deg,
            elevon_diff_deg=elevon_diff_deg,
        ),
        op_point=op_point,
    )
    result = solver.run()
    return AeroSnapshot(
        CL=_scalar(result["CL"]),
        CD=_scalar(result["CD"]),
        Cm=_scalar(result["Cm"]),
        CY=_scalar(result.get("CY", 0.0)),
        Cl=_scalar(result.get("Cl", 0.0)),
        Cn=_scalar(result.get("Cn", 0.0)),
    )


def _central_difference(low: float, high: float, delta_rad: float) -> float:
    return (high - low) / (2.0 * delta_rad)


def evaluate_validation(
    parameters: DesignParameters,
    validation_targets: dict[str, Any],
    repo_root: Path,
) -> tuple[DerivedGeometry, dict[str, Any], str, str, list[dict[str, float]]]:
    derived = derive_geometry(parameters)
    cruise_speed = parameters.analysis_conditions.cruise_speed_m_s
    cruise_alpha = parameters.analysis_conditions.cruise_alpha_deg
    delta_angle_deg = 1.0
    delta_rate = 0.01

    base = run_aero(parameters, derived, repo_root, cruise_speed, cruise_alpha)
    alpha_low = run_aero(parameters, derived, repo_root, cruise_speed, cruise_alpha - delta_angle_deg)
    alpha_high = run_aero(parameters, derived, repo_root, cruise_speed, cruise_alpha + delta_angle_deg)
    beta_low = run_aero(parameters, derived, repo_root, cruise_speed, cruise_alpha, beta_deg=-delta_angle_deg)
    beta_high = run_aero(parameters, derived, repo_root, cruise_speed, cruise_alpha, beta_deg=delta_angle_deg)
    roll_rate = run_aero(parameters, derived, repo_root, cruise_speed, cruise_alpha, p_rad_s=delta_rate)
    yaw_rate = run_aero(parameters, derived, repo_root, cruise_speed, cruise_alpha, r_rad_s=delta_rate)

    delta_rad = np.radians(delta_angle_deg)
    cl_alpha = _central_difference(alpha_low.CL, alpha_high.CL, delta_rad)
    cm_alpha = _central_difference(alpha_low.Cm, alpha_high.Cm, delta_rad)
    cn_beta = _central_difference(beta_low.Cn, beta_high.Cn, delta_rad)
    cl_beta = _central_difference(beta_low.Cl, beta_high.Cl, delta_rad)
    nondim_rate = delta_rate * derived.wing_projected_span_m / (2.0 * cruise_speed)
    cl_p = (roll_rate.Cl - base.Cl) / nondim_rate
    cn_r = (yaw_rate.Cn - base.Cn) / nondim_rate

    canard_plus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, canard_deflection_deg=delta_angle_deg
    )
    canard_minus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, canard_deflection_deg=-delta_angle_deg
    )
    cm_delta_canard = _central_difference(canard_minus.Cm, canard_plus.Cm, delta_rad)

    elevon_plus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, elevon_sym_deg=delta_angle_deg
    )
    elevon_minus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, elevon_sym_deg=-delta_angle_deg
    )
    cm_delta_elevon_sym = _central_difference(elevon_minus.Cm, elevon_plus.Cm, delta_rad)
    powered_cm_delta_elevon_sym = (
        cm_delta_elevon_sym * parameters.power_effects.elevon_dynamic_pressure_gain_powered
    )

    diff_plus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, elevon_diff_deg=delta_angle_deg
    )
    diff_minus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, elevon_diff_deg=-delta_angle_deg
    )
    cl_delta_elevon_diff = _central_difference(diff_minus.Cl, diff_plus.Cl, delta_rad)

    trim_delta = 0.5
    trim_plus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, canard_deflection_deg=trim_delta
    )
    trim_minus = run_aero(
        parameters, derived, repo_root, cruise_speed, cruise_alpha, canard_deflection_deg=-trim_delta
    )
    cm_delta_for_trim = _central_difference(trim_minus.Cm, trim_plus.Cm, np.radians(trim_delta))
    trim_canard_deg = 0.0 if abs(cm_delta_for_trim) < 1e-8 else np.degrees(-base.Cm / cm_delta_for_trim)

    interference_sweep = []
    for alpha in (
        parameters.analysis_conditions.cruise_alpha_deg,
        parameters.analysis_conditions.transition_alpha_deg,
    ):
        for deflection in (
            -parameters.canard.deflection_limits_deg.forward_command_abs,
            0.0,
            parameters.canard.deflection_limits_deg.forward_command_abs,
            parameters.canard.deflection_limits_deg.transition_abs,
        ):
            effective_angle = alpha + max(deflection, 0.0) + 4.0
            delta_x = (
                derived.main_wing_x_at_canard_tip_m - derived.canard_tip_trailing_edge_x_m
            )
            wake_z = parameters.canard.stations[-1].z_m - np.tan(np.radians(effective_angle)) * max(delta_x, 0.0)
            clearance = wake_z - derived.main_wing_z_at_canard_tip_m
            interference_sweep.append(
                {
                    "alpha_deg": float(alpha),
                    "canard_deflection_deg": float(deflection),
                    "wake_clearance_m": float(clearance),
                }
            )

    metrics = {
        "geometry": {
            "wing_area_m2": derived.wing_area_m2,
            "wing_aspect_ratio": derived.wing_aspect_ratio,
            "wing_quarter_chord_sweep_deg": derived.wing_quarter_chord_sweep_deg,
            "canard_area_ratio": derived.canard_area_ratio,
            "canard_to_prop_radial_clearance_m": derived.canard_to_prop_radial_clearance_m,
            "canard_to_prop_x_clearance_m": derived.canard_to_prop_x_clearance_m,
            "wake_clearance_m": derived.wake_clearance_m,
            "elevon_span_in_propwash_fraction": derived.elevon_span_in_propwash_fraction,
        },
        "forward_flight": {
            "CL": base.CL,
            "CD": base.CD,
            "Cm": base.Cm,
            "required_cruise_cl": derived.required_cruise_cl,
            "cl_alpha": cl_alpha,
            "cm_alpha": cm_alpha,
            "cn_beta": cn_beta,
            "cl_beta": cl_beta,
            "cl_p": cl_p,
            "cn_r": cn_r,
            "trim_canard_deg": trim_canard_deg,
        },
        "control_effectiveness": {
            "cm_delta_canard_per_rad": cm_delta_canard,
            "cm_delta_elevon_sym_per_rad": cm_delta_elevon_sym,
            "cm_delta_elevon_sym_powered_per_rad": powered_cm_delta_elevon_sym,
            "cm_delta_elevon_sym_powered_gain_ratio": (
                powered_cm_delta_elevon_sym / cm_delta_elevon_sym if abs(cm_delta_elevon_sym) > 1e-8 else 0.0
            ),
            "cl_delta_elevon_diff_per_rad": cl_delta_elevon_diff,
        },
    }
    min_sweep_clearance = min(item["wake_clearance_m"] for item in interference_sweep)

    checks: list[tuple[str, bool, float, float | None, float | None]] = []
    g = validation_targets["geometry"]
    ff = validation_targets["forward_flight"]
    ce = validation_targets["control_effectiveness"]
    checks.extend(
        [
            (
                "Wing area",
                abs(derived.wing_area_m2 - g["wing_area_m2"]["target"]) <= g["wing_area_m2"]["tolerance"],
                derived.wing_area_m2,
                g["wing_area_m2"]["target"],
                g["wing_area_m2"]["tolerance"],
            ),
            (
                "Wing aspect ratio",
                abs(derived.wing_aspect_ratio - g["wing_aspect_ratio"]["target"])
                <= g["wing_aspect_ratio"]["tolerance"],
                derived.wing_aspect_ratio,
                g["wing_aspect_ratio"]["target"],
                g["wing_aspect_ratio"]["tolerance"],
            ),
            (
                "Canard area ratio",
                abs(derived.canard_area_ratio - g["canard_area_ratio"]["target"])
                <= g["canard_area_ratio"]["tolerance"],
                derived.canard_area_ratio,
                g["canard_area_ratio"]["target"],
                g["canard_area_ratio"]["tolerance"],
            ),
            (
                "Wing quarter-chord sweep",
                abs(derived.wing_quarter_chord_sweep_deg - g["wing_quarter_chord_sweep_deg"]["target"])
                <= g["wing_quarter_chord_sweep_deg"]["tolerance"],
                derived.wing_quarter_chord_sweep_deg,
                g["wing_quarter_chord_sweep_deg"]["target"],
                g["wing_quarter_chord_sweep_deg"]["tolerance"],
            ),
            (
                "Canard to prop radial clearance",
                derived.canard_to_prop_radial_clearance_m >= g["canard_to_prop_radial_clearance_m"]["minimum"],
                derived.canard_to_prop_radial_clearance_m,
                g["canard_to_prop_radial_clearance_m"]["minimum"],
                None,
            ),
            (
                "Canard to prop x clearance",
                derived.canard_to_prop_x_clearance_m >= g["canard_to_prop_x_clearance_m"]["minimum"],
                derived.canard_to_prop_x_clearance_m,
                g["canard_to_prop_x_clearance_m"]["minimum"],
                None,
            ),
            (
                "Wake clearance sweep minimum",
                min_sweep_clearance >= g["wake_clearance_m"]["minimum"],
                min_sweep_clearance,
                g["wake_clearance_m"]["minimum"],
                None,
            ),
            (
                "Elevon span in propwash",
                derived.elevon_span_in_propwash_fraction >= g["elevon_span_in_propwash_fraction"]["minimum"],
                derived.elevon_span_in_propwash_fraction,
                g["elevon_span_in_propwash_fraction"]["minimum"],
                None,
            ),
            ("CL alpha min", cl_alpha >= ff["cl_alpha_min"], cl_alpha, ff["cl_alpha_min"], None),
            ("CL alpha max", cl_alpha <= ff["cl_alpha_max"], cl_alpha, ff["cl_alpha_max"], None),
            ("Cm alpha max", cm_alpha <= ff["cm_alpha_max"], cm_alpha, ff["cm_alpha_max"], None),
            ("Cn beta min", cn_beta >= ff["cn_beta_min"], cn_beta, ff["cn_beta_min"], None),
            ("Cl p max", cl_p <= ff["cl_p_max"], cl_p, ff["cl_p_max"], None),
            ("Cn r max", cn_r <= ff["cn_r_max"], cn_r, ff["cn_r_max"], None),
            (
                "Trim canard target",
                abs(trim_canard_deg) <= ff["trim_canard_abs_max_deg"],
                trim_canard_deg,
                ff["trim_canard_abs_max_deg"],
                None,
            ),
            (
                "Canard pitch effectiveness",
                cm_delta_canard >= ce["cm_delta_canard_min_per_rad"],
                cm_delta_canard,
                ce["cm_delta_canard_min_per_rad"],
                None,
            ),
            (
                "Differential elevon roll effectiveness",
                abs(cl_delta_elevon_diff) >= ce["cl_delta_elevon_diff_min_per_rad"],
                abs(cl_delta_elevon_diff),
                ce["cl_delta_elevon_diff_min_per_rad"],
                None,
            ),
            (
                "Powered elevon pitch gain",
                metrics["control_effectiveness"]["cm_delta_elevon_sym_powered_gain_ratio"]
                >= ce["cm_delta_elevon_sym_powered_gain_min_ratio"],
                metrics["control_effectiveness"]["cm_delta_elevon_sym_powered_gain_ratio"],
                ce["cm_delta_elevon_sym_powered_gain_min_ratio"],
                None,
            ),
        ]
    )

    report_lines = ["# Validation Report V3", "", "## Checks"]
    passed = 0
    for name, ok, actual, target, tolerance in checks:
        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        if tolerance is None:
            report_lines.append(f"- {status}: {name}: actual={actual:.6f}, target={target:.6f}")
        else:
            report_lines.append(
                f"- {status}: {name}: actual={actual:.6f}, target={target:.6f}, tolerance={tolerance:.6f}"
            )
    report_lines.extend(
        [
            "",
            "## Summary",
            f"- Passed: {passed}",
            f"- Failed: {len(checks) - passed}",
        ]
    )

    control_lines = [
        "# Control Authority Report V3",
        "",
        f"- Canard pitch effectiveness: {cm_delta_canard:.6f} per rad",
        f"- Symmetric elevon pitch effectiveness, unpowered: {cm_delta_elevon_sym:.6f} per rad",
        f"- Symmetric elevon pitch effectiveness, powered proxy: {powered_cm_delta_elevon_sym:.6f} per rad",
        f"- Differential elevon roll effectiveness: {cl_delta_elevon_diff:.6f} per rad",
        f"- Cruise trim canard requirement: {trim_canard_deg:.6f} deg",
    ]
    return derived, metrics, "\n".join(report_lines) + "\n", "\n".join(control_lines) + "\n", interference_sweep


def write_validation_outputs(
    metrics_path: Path,
    report_path: Path,
    control_report_path: Path,
    interference_path: Path,
    metrics: dict[str, Any],
    report: str,
    control_report: str,
    interference_sweep: list[dict[str, float]],
) -> None:
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    report_path.write_text(report, encoding="utf-8")
    control_report_path.write_text(control_report, encoding="utf-8")
    interference_path.write_text(json.dumps(interference_sweep, indent=2), encoding="utf-8")
