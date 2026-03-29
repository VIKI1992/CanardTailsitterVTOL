"""
deflection_viz.py — AeroSandbox VLM visualization of canard and elevon deflections.
Opens one interactive window per configuration (close each to advance).

Canard deflections: -10, -5, 0, +5, +10°  (elevon=0)
Elevon deflections: -20, -10, 0, +10, +20° (canard=0)
"""
import json
import numpy as np
import aerosandbox as asb
from pathlib import Path

REPO = Path(__file__).parent
geo = json.loads((REPO / "v3" / "results" / "vsp3_geometry.json").read_text())
ws_raw = geo["wing"]["stations"]
cs_raw = geo["canard"]["stations"]
ev     = geo["elevon"]

CANARD_DEFL = [-10, -5, 0, 5, 10]
ELEVON_DEFL = [-20, -10, 0, 10, 20]
OP = asb.OperatingPoint(velocity=18.0, alpha=6.0)


def interp_station(stations, y_target):
    for a, b in zip(stations[:-1], stations[1:]):
        if a["y_m"] <= y_target <= b["y_m"]:
            t = (y_target - a["y_m"]) / (b["y_m"] - a["y_m"])
            return {k: a[k] + t * (b[k] - a[k])
                    if isinstance(a[k], float) else a[k] for k in a}
    return stations[-1]


def build_airplane(canard_defl: float = 0.0, elevon_defl: float = 0.0) -> asb.Airplane:
    y0, y1, hf = ev["start_y_m"], ev["end_y_m"], ev["hinge_frac"]

    ws = []
    inserted = set()
    for i, s in enumerate(ws_raw):
        if i > 0:
            prev_y = ws_raw[i - 1]["y_m"]
            for yb in [y0, y1]:
                if prev_y < yb < s["y_m"] and yb not in inserted:
                    ws.append(interp_station(ws_raw, yb))
                    inserted.add(yb)
        ws.append(s)

    elevon_cs = asb.ControlSurface(
        name="Elevon", deflection=elevon_defl, hinge_point=hf, symmetric=True,
    )

    wing = asb.Wing(
        name="Wing", symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=np.array([s["le_x_m"], s["y_m"], s["z_m"]]),
                chord=s["chord_m"], twist=s["twist_deg"],
                airfoil=asb.Airfoil("sd7037"),
                control_surfaces=[elevon_cs] if y0 <= s["y_m"] <= y1 else [],
            )
            for s in ws
        ],
    )

    canard_incidence = cs_raw[0]["twist_deg"]
    canard = asb.Wing(
        name="Canard", symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=np.array([s["le_x_m"], s["y_m"], s["z_m"]]),
                chord=s["chord_m"],
                twist=canard_incidence + canard_defl,
                airfoil=asb.Airfoil("naca0009"),
            )
            for s in cs_raw
        ],
    )

    fuselage = asb.Fuselage(
        name="Fuselage",
        xsecs=[
            asb.FuselageXSec(
                xyz_c=np.array([s["x_m"], 0.0, 0.0]),
                radius=max((s.get("width_m") or 0.01), (s.get("height_m") or 0.01)) / 2,
            )
            for s in geo["fuselage"]["stations"]
        ],
    )

    return asb.Airplane(name="CanardTailsitter",
                        wings=[wing, canard], fuselages=[fuselage])


def run_vlm(airplane):
    vlm = asb.VortexLatticeMethod(
        airplane=airplane, op_point=OP,
        spanwise_resolution=8, chordwise_resolution=4,
    )
    vlm.run()
    return vlm


# ─── run and show each configuration ─────────────────────────────────────────

configs = (
    [(f"Canard {d:+d}°  (elevon 0°)", dict(canard_defl=float(d)))
     for d in CANARD_DEFL]
    +
    [(f"Elevon {d:+d}°  (canard 0°)", dict(elevon_defl=float(d)))
     for d in ELEVON_DEFL]
)

for title, kw in configs:
    print(f"Running VLM: {title} ...", flush=True)
    vlm = run_vlm(build_airplane(**kw))
    print(f"  → {title}", flush=True)

    # Normalize by local panel chord × V  →  dimensionless Cl-like distribution.
    # Then clip to 5th–95th percentile so outlier canard panels (tiny chord →
    # very high Γ/(V·c)) don't compress the wing colourmap to a single hue.
    mid_front = 0.5 * (vlm.front_left_vertices + vlm.front_right_vertices)
    mid_back  = 0.5 * (vlm.back_left_vertices  + vlm.back_right_vertices)
    panel_chord = np.linalg.norm(mid_back - mid_front, axis=1).clip(1e-6)
    c = vlm.vortex_strengths / (panel_chord * OP.velocity)
    c = np.clip(c, np.percentile(c, 5), np.percentile(c, 95))

    vlm.draw(c=c, cmap="plasma", colorbar_label="Γ / (V·c)  ≈ local Cl")
