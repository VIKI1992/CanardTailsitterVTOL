"""
design_space.py — 12-dimensional parametric design space for the canard tailsitter VTOL.

Free parameters:
  0  wing_root_chord_mm       [360, 480]
  1  wing_tip_chord_mm        [160, 280]   (outer_wing station, y=0.78 m)
  2  wing_taper_knee_frac     [0.30, 0.70] (chord blend fraction at inboard break)
  3  wing_washout_tip_deg     [-4.0, 0.0]
  4  wing_root_incidence_deg  [-1.0, 2.0]
  5  wing_le_sweep_deg        [12.0, 28.0]
  6  canard_area_ratio        [0.08, 0.20]
  7  canard_incidence_deg     [-1.0, 4.0]
  8  canard_x_le_root_mm      [180.0, 280.0]
  9  cruise_velocity_ms       [15.0, 25.0]
  10 cruise_alpha_deg         [2.0, 10.0]
  11 wing_ar_scale            [0.85, 1.15]  (span scale factor)

Fuselage, winglet shape, propulsion geometry are held fixed.
"""
from __future__ import annotations

import copy
import math
from dataclasses import replace
from typing import TYPE_CHECKING

import numpy as np
from scipy.stats.qmc import LatinHypercube

if TYPE_CHECKING:
    from v3.model.spec import DesignParameters, SurfaceStation

N_DIM = 10

# Raw parameter bounds: (lower, upper) in natural units
# Cruise conditions are FIXED at baseline (18 m/s, 6°) — geometry-only optimisation.
_BOUNDS_RAW: list[tuple[float, float]] = [
    (360.0, 480.0),    # 0  wing_root_chord_mm
    (160.0, 280.0),    # 1  wing_tip_chord_mm
    (0.30,  0.70),     # 2  wing_taper_knee_frac
    (-4.0,  0.0),      # 3  wing_washout_tip_deg
    (-1.0,  2.0),      # 4  wing_root_incidence_deg
    (12.0,  28.0),     # 5  wing_le_sweep_deg
    (0.08,  0.20),     # 6  canard_area_ratio
    (-1.0,  4.0),      # 7  canard_incidence_deg
    (-480.0, -280.0),  # 8  canard_x_le_root_mm (negative = forward of CG)
    (0.85,  1.15),     # 9  wing_ar_scale
]

PARAM_NAMES = [
    "wing_root_chord_mm",
    "wing_tip_chord_mm",
    "wing_taper_knee_frac",
    "wing_washout_tip_deg",
    "wing_root_incidence_deg",
    "wing_le_sweep_deg",
    "canard_area_ratio",
    "canard_incidence_deg",
    "canard_x_le_root_mm",
    "wing_ar_scale",
]

# Fixed cruise conditions (same as baseline)
CRUISE_VELOCITY_MS = 18.0
CRUISE_ALPHA_DEG   = 6.0

_LOWERS = np.array([b[0] for b in _BOUNDS_RAW], dtype=np.float64)
_UPPERS = np.array([b[1] for b in _BOUNDS_RAW], dtype=np.float64)


def encode(raw: np.ndarray) -> np.ndarray:
    """Map raw parameter vector to [0, 1]^N_DIM."""
    return (raw - _LOWERS) / (_UPPERS - _LOWERS)


def decode(x: np.ndarray) -> np.ndarray:
    """Map [0, 1]^N_DIM back to raw parameter values."""
    return _LOWERS + x * (_UPPERS - _LOWERS)


def clamp(x: np.ndarray) -> np.ndarray:
    """Project normalised vector back into [0, 1]^N_DIM."""
    return np.clip(x, 0.0, 1.0)


def x_to_dict(x: np.ndarray) -> dict[str, float]:
    """Convert normalised vector to a named dict of raw values."""
    raw = decode(x)
    return {name: float(raw[i]) for i, name in enumerate(PARAM_NAMES)}


def sample_lhs(n: int, seed: int = 42) -> np.ndarray:
    """Latin-Hypercube sample: returns (n, N_DIM) in [0, 1]^N_DIM."""
    sampler = LatinHypercube(d=N_DIM, seed=seed)
    return sampler.random(n).astype(np.float64)


def get_bounds_normalised() -> list[tuple[float, float]]:
    """Bounds for scipy optimizers: always [(0,1)] * N_DIM."""
    return [(0.0, 1.0)] * N_DIM


# ---------------------------------------------------------------------------
# Wing station rebuilding helpers
# ---------------------------------------------------------------------------

# Fixed y-positions and z-positions of the 7 wing stations (metres)
_WING_Y = np.array([0.000, 0.350, 0.780, 0.860, 0.920, 0.960, 1.000])
_WING_Z = np.array([0.000, 0.000, 0.000, 0.008, 0.040, 0.120, 0.250])
# Twist of winglet stations (stations 3-6) relative to each other — fixed
_WINGLET_TWIST_OFFSETS = np.array([-1.5, -1.0, 0.0, 0.0])  # stations 3..6


def _build_wing_stations(
    root_chord_m: float,
    tip_chord_m: float,
    knee_frac: float,
    washout_tip_deg: float,
    root_incidence_deg: float,
    le_sweep_deg: float,
    ar_scale: float,
    base_stations: list,  # list[SurfaceStation] — used to read winglet chords
) -> list:
    """
    Reconstruct the 7 wing SurfaceStation objects from the 6 shape parameters.

    Stations 0-2 (root, inboard_break, outer_wing) have their chords and
    twists recomputed; stations 3-6 (winglet blend) keep their baseline chord
    ratios relative to the new tip chord and are swept/scaled consistently.
    """
    from v3.model.spec import SurfaceStation

    # Scale y-positions by ar_scale (changes span while keeping station fracs)
    y = _WING_Y * ar_scale

    # --- Chord distribution for stations 0..2 ---
    # Linearly interpolate from root to tip, with a knee at the inboard break
    # station 0: y_frac = 0
    # station 1: y_frac = 0.35 / 0.78 ≈ 0.449 (of the main panel span)
    # station 2: y_frac = 1.0 (tip of main panel)
    inboard_y_frac = _WING_Y[1] / _WING_Y[2]  # ≈ 0.449

    # Chord at inboard break via Bezier-style blend
    # knee_frac controls how much taper happens inboard vs outboard
    chord_inboard = root_chord_m + knee_frac * inboard_y_frac * (tip_chord_m - root_chord_m)
    chords_main = [root_chord_m, chord_inboard, tip_chord_m]

    # Winglet stations: scale chords proportional to the new tip chord
    baseline_tip_chord = base_stations[2].chord_m
    if baseline_tip_chord > 1e-6:
        winglet_chord_scale = tip_chord_m / baseline_tip_chord
    else:
        winglet_chord_scale = 1.0
    chords_winglet = [s.chord_m * winglet_chord_scale for s in base_stations[3:]]

    all_chords = chords_main + chords_winglet  # length 7

    # --- Leading-edge x-positions ---
    # Root LE stays fixed; sweep angle determines how far back subsequent LEs are
    root_le_x = base_stations[0].leading_edge_x_m
    tan_sweep = math.tan(math.radians(le_sweep_deg))
    le_x = [root_le_x + tan_sweep * yi for yi in y]

    # --- Twist distribution ---
    # Stations 0-2: linear wash-out from root to tip
    twists_main = [
        root_incidence_deg,
        root_incidence_deg + (washout_tip_deg - root_incidence_deg) * (_WING_Y[1] / _WING_Y[2]),
        root_incidence_deg + washout_tip_deg,
    ]
    # Winglet stations: fixed offsets relative to the tip twist
    tip_twist = twists_main[2]
    twists_winglet = [tip_twist + offset for offset in _WINGLET_TWIST_OFFSETS]
    all_twists = twists_main + twists_winglet

    # --- Z positions: scale with ar_scale (winglet height stays proportional) ---
    z_scaled = _WING_Z * ar_scale

    # Build SurfaceStation objects
    station_names = [s.name for s in base_stations]
    result = []
    for i in range(7):
        result.append(SurfaceStation(
            name=station_names[i],
            leading_edge_x_m=float(le_x[i]),
            y_m=float(y[i]),
            z_m=float(z_scaled[i]),
            chord_m=float(all_chords[i]),
            twist_deg=float(all_twists[i]),
        ))
    return result


def _build_canard_stations(
    canard_area_ratio: float,
    canard_incidence_deg: float,
    canard_x_le_root_m: float,
    wing_area_m2: float,
    base_canard,  # CanardDefinition
) -> list:
    """
    Reconstruct canard SurfaceStation objects from 3 canard parameters.

    Fixed taper ratio = tip_chord / root_chord (from baseline = 0.11/0.16 = 0.6875).
    The canard spans half-span = base half-span (0.30 m) — span is not optimised.
    Area = 2 * 0.5 * (root + tip) * span_half
    """
    from v3.model.spec import SurfaceStation

    span_half = base_canard.stations[-1].y_m  # 0.30 m, fixed
    baseline_taper = base_canard.stations[-1].chord_m / base_canard.stations[0].chord_m  # ≈ 0.6875

    # Solve for root_chord given area_ratio and fixed taper
    # canard_area = 2 * 0.5 * (root + tip) * span_half = (root + root*taper) * span_half
    # canard_area = root * (1 + taper) * span_half
    # canard_area = canard_area_ratio * wing_area_m2
    target_canard_area = canard_area_ratio * wing_area_m2
    root_chord = target_canard_area / ((1.0 + baseline_taper) * span_half)
    tip_chord = root_chord * baseline_taper

    # LE x: root is at canard_x_le_root_m; tip follows baseline sweep
    baseline_sweep_dx = base_canard.stations[-1].leading_edge_x_m - base_canard.stations[0].leading_edge_x_m
    tip_le_x = canard_x_le_root_m + baseline_sweep_dx

    station_names = [s.name for s in base_canard.stations]
    y_positions = [s.y_m for s in base_canard.stations]
    z_positions = [s.z_m for s in base_canard.stations]

    result = []
    chords = [root_chord, tip_chord]
    le_xs = [canard_x_le_root_m, tip_le_x]
    for i, s in enumerate(base_canard.stations):
        result.append(SurfaceStation(
            name=station_names[i],
            leading_edge_x_m=float(le_xs[i]),
            y_m=float(y_positions[i]),
            z_m=float(z_positions[i]),
            chord_m=float(chords[i]),
            twist_deg=float(canard_incidence_deg),
        ))
    return result


def to_design_parameters(
    x: np.ndarray,
    base_params: "DesignParameters",
) -> "DesignParameters":
    """
    Convert a normalised [0,1]^12 vector into a full DesignParameters object.

    Only wing stations, canard stations, and analysis_conditions are mutated.
    Fuselage, propulsion, mass_properties, power_effects are unchanged.
    """
    from v3.model.geometry import derive_geometry
    from v3.model.spec import (
        AnalysisConditions,
        CanardDefinition,
        WingDefinition,
    )

    raw = decode(x)
    p = {name: float(raw[i]) for i, name in enumerate(PARAM_NAMES)}

    root_chord_m = p["wing_root_chord_mm"] / 1000.0
    tip_chord_m = p["wing_tip_chord_mm"] / 1000.0

    # Build new wing stations
    new_wing_stations = _build_wing_stations(
        root_chord_m=root_chord_m,
        tip_chord_m=tip_chord_m,
        knee_frac=p["wing_taper_knee_frac"],
        washout_tip_deg=p["wing_washout_tip_deg"],
        root_incidence_deg=p["wing_root_incidence_deg"],
        le_sweep_deg=p["wing_le_sweep_deg"],
        ar_scale=p["wing_ar_scale"],
        base_stations=base_params.wing.stations,
    )

    new_wing = WingDefinition(
        name=base_params.wing.name,
        airfoil=base_params.wing.airfoil,
        stations=new_wing_stations,
    )

    # Estimate wing area for canard area ratio (approximate — 3 main stations)
    # Quick trapezoidal: 2 * sum of half-panel areas
    wing_area_approx = 2.0 * sum(
        0.5 * (new_wing_stations[i].chord_m + new_wing_stations[i + 1].chord_m)
        * (new_wing_stations[i + 1].y_m - new_wing_stations[i].y_m)
        for i in range(len(new_wing_stations) - 1)
    )

    # Build new canard stations
    new_canard_stations = _build_canard_stations(
        canard_area_ratio=p["canard_area_ratio"],
        canard_incidence_deg=p["canard_incidence_deg"],
        canard_x_le_root_m=p["canard_x_le_root_mm"] / 1000.0,
        wing_area_m2=wing_area_approx,
        base_canard=base_params.canard,
    )

    new_canard = CanardDefinition(
        name=base_params.canard.name,
        airfoil=base_params.canard.airfoil,
        pivot_fraction=base_params.canard.pivot_fraction,
        baseline_incidence_deg=p["canard_incidence_deg"],
        stations=new_canard_stations,
        deflection_limits_deg=base_params.canard.deflection_limits_deg,
    )

    new_conditions = AnalysisConditions(
        cruise_speed_m_s=CRUISE_VELOCITY_MS,
        transition_speed_m_s=base_params.analysis_conditions.transition_speed_m_s,
        air_density_kg_m3=base_params.analysis_conditions.air_density_kg_m3,
        cruise_alpha_deg=CRUISE_ALPHA_DEG,
        transition_alpha_deg=base_params.analysis_conditions.transition_alpha_deg,
    )

    # Return a new DesignParameters with patched fields
    # (using dataclass replace pattern via constructor since fields are frozen)
    from v3.model.spec import DesignParameters
    return DesignParameters(
        metadata=base_params.metadata,
        wing=new_wing,
        canard=new_canard,
        control_surfaces=base_params.control_surfaces,
        fuselage=base_params.fuselage,
        propulsion=base_params.propulsion,
        mass_properties=base_params.mass_properties,
        analysis_conditions=new_conditions,
        power_effects=base_params.power_effects,
    )


def baseline_x(base_params: "DesignParameters") -> np.ndarray:
    """Return the normalised encoding of the baseline design parameters."""
    from v3.model.geometry import derive_geometry

    derived = derive_geometry(base_params)
    wing = base_params.wing.stations

    # Recover approximate sweep from root->outer_wing LE positions
    root_le = wing[0].leading_edge_x_m
    outer_le = wing[2].leading_edge_x_m
    outer_y = wing[2].y_m
    le_sweep_deg = math.degrees(math.atan2(outer_le - root_le, outer_y))

    raw = np.array([
        wing[0].chord_m * 1000.0,           # root chord mm
        wing[2].chord_m * 1000.0,           # tip chord mm
        0.5,                                 # knee_frac (neutral)
        wing[2].twist_deg - wing[0].twist_deg,  # washout_tip (approx)
        wing[0].twist_deg,                   # root_incidence
        le_sweep_deg,
        derived.canard_area_ratio,
        base_params.canard.baseline_incidence_deg,
        base_params.canard.stations[0].leading_edge_x_m * 1000.0,  # negative
        1.0,                                 # ar_scale = 1 (no scaling)
    ], dtype=np.float64)

    return encode(np.clip(raw, _LOWERS, _UPPERS))
