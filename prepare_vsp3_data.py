"""
Prepare geometry data JSON for the OpenVSP Python API builder.
Runs in the main Python env (has scipy/aerosandbox).
"""
import sys, json, math
from pathlib import Path

REPO = Path(__file__).parent
sys.path.insert(0, str(REPO))

from v3.model.spec import load_parameters
from v3.optimization.design_space import encode, PARAM_NAMES, _LOWERS, _UPPERS, to_design_parameters
import numpy as np

# Find the latest optimized results file
results = sorted(REPO.glob("results/optimized_params_*.json"))
result_path = results[-1]
print(f"Using: {result_path.name}")

base = load_parameters(REPO / "spec" / "parameters_v3.json")
data = json.loads(result_path.read_text())
raw_dict = data["best_candidate"]["design_vector_raw"]
raw = np.array([raw_dict[n] for n in PARAM_NAMES])
x = encode(np.clip(raw, _LOWERS, _UPPERS))
p = to_design_parameters(x, base)

def stations_to_list(stations):
    return [
        {
            "name": s.name,
            "y_m": float(s.y_m),
            "z_m": float(s.z_m),
            "chord_m": float(s.chord_m),
            "le_x_m": float(s.leading_edge_x_m),
            "twist_deg": float(s.twist_deg),
        }
        for s in stations
    ]

out = {
    "wing": {
        "stations": stations_to_list(p.wing.stations),
        "airfoil": p.wing.airfoil,
        "root_incidence_deg": float(p.wing.stations[0].twist_deg),
    },
    "canard": {
        "stations": stations_to_list(p.canard.stations),
        "airfoil": p.canard.airfoil,
        "incidence_deg": float(p.canard.baseline_incidence_deg),
    },
    "fuselage": {
        "stations": [
            {
                "name": s.name,
                "x_m": float(s.x_m),
                "shape": s.shape,
                "width_m": float(s.width_m) if s.width_m else None,
                "height_m": float(s.height_m) if s.height_m else None,
                "corner_radius_m": float(s.corner_radius_m) if s.corner_radius_m else None,
            }
            for s in p.fuselage.stations
        ]
    },
    "propulsion": {
        "motor_x_m": float(p.propulsion.motor_x_m),
        "motor_y_m": float(p.propulsion.motor_y_m),
        "motor_z_m": float(p.propulsion.motor_z_m),
        "prop_diameter_m": float(p.propulsion.prop_diameter_m),
        "left_rotation": p.propulsion.left_rotation,
        "right_rotation": p.propulsion.right_rotation,
    },
    "elevon": {
        "start_y_m": float(p.control_surfaces["elevon_left"].start_y_m),
        "end_y_m": float(p.control_surfaces["elevon_left"].end_y_m),
        "hinge_frac": float(p.control_surfaces["elevon_left"].hinge_fraction_from_le),
    },
    "out_vsp3": str(REPO / "v3" / "results" / "fixed_wing_tailsitter_optimized.vsp3"),
    "source_json": result_path.name,
}

# Print summary
ws = p.wing.stations
print(f"\nWing summary:")
print(f"  Root chord:   {ws[0].chord_m*1000:.1f} mm")
print(f"  Outer chord:  {ws[2].chord_m*1000:.1f} mm")
print(f"  Wingspan:     {2*ws[-1].y_m*1000:.1f} mm")
print(f"  Flat half-span (to outer_wing): {ws[2].y_m*1000:.1f} mm")
print(f"Canard:")
print(f"  Root chord:  {p.canard.stations[0].chord_m*1000:.1f} mm")
print(f"  LE x:        {p.canard.stations[0].le_x_m*1000:.1f} mm") if False else None
print(f"  LE x:        {p.canard.stations[0].leading_edge_x_m*1000:.1f} mm")

geo_path = REPO / "v3" / "results" / "vsp3_geometry.json"
geo_path.write_text(json.dumps(out, indent=2))
print(f"\nGeometry data written to: {geo_path}")
