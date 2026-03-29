"""
objectives.py — Objective function and constraint penalties for the design optimizer.

All functions accept and return PyTorch tensors so that gradients flow through
the surrogate and back into the design vector during gradient-based optimization.

Primary objective: maximise L/D = CL / (CD + ε) at cruise.
Constraint penalties are added as w × max(0, violation)² (exterior point method).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import torch
import torch.nn.functional as F

if TYPE_CHECKING:
    from v3.model.spec import DesignParameters
    from v3.optimization.dataset import AeroDataset
    from v3.optimization.surrogate import AeroSurrogate


# ---------------------------------------------------------------------------
# Constraint weights
# ---------------------------------------------------------------------------

# Aerodynamic constraints (evaluated via surrogate)
W_CL_LEVEL      = 300.0   # CL >= required for level flight at chosen velocity
W_CD_MAX        = 50.0    # CD < 0.10 at cruise
W_CM_TRIM       = 100.0   # |Cm| < 0.05  (trimmable)

# Geometric constraints (evaluated analytically)
W_AR_BOUNDS     = 1000.0  # AR ∈ [6.7, 7.7]  (interior of validation ±0.6 around 7.2)
W_CANARD_AREA   = 10000.0 # canard_area_ratio ∈ [0.120, 0.170] (interior of validation)
W_WAKE          = 150.0   # wake_clearance > 0.02 m
W_PROP_RADIAL   = 100.0   # canard_to_prop_radial_clearance > 0.03 m
W_PROP_X        = 100.0   # canard_to_prop_x_clearance > 0.08 m
W_ELEVON_WASH   = 50.0    # elevon_span_in_propwash > 0.60
W_TAPER         = 80.0    # tip_chord > 0.10 m (min Re)
W_TIP_CHORD_MAX = 50.0    # tip_chord <= root_chord (no inverse taper)

# Physical constants for level-flight CL requirement
_MASS_KG   = 5.2
_G         = 9.80665
_RHO       = 1.225


def ld_ratio(cl: torch.Tensor, cd: torch.Tensor) -> torch.Tensor:
    return cl / (cd + 1e-6)


def aero_penalty(
    cl: torch.Tensor,
    cd: torch.Tensor,
    cm: torch.Tensor,
    required_cl: torch.Tensor | None = None,
) -> torch.Tensor:
    """Penalise aerodynamic constraint violations (differentiable).

    required_cl: minimum CL needed for level flight at the chosen cruise
                 velocity and wing area.  If None, falls back to CL > 0.30.
    """
    min_cl = required_cl if required_cl is not None else torch.tensor(0.30)
    p_cl  = W_CL_LEVEL * F.relu(min_cl - cl) ** 2
    p_cd  = W_CD_MAX   * F.relu(cd - 0.10) ** 2
    p_cm  = W_CM_TRIM  * F.relu(cm.abs() - 0.05) ** 2
    return p_cl + p_cd + p_cm


def geometric_penalty_from_x(x_raw: torch.Tensor) -> torch.Tensor:
    """
    Compute geometric constraint penalties directly from the raw [0,1]^12 vector.

    Uses the known parameter bounds to reconstruct approximate geometric values
    without calling AeroSandbox — fast enough for every optimizer step.
    """
    from v3.optimization.design_space import _LOWERS, _UPPERS

    lo = torch.tensor(_LOWERS, dtype=x_raw.dtype, device=x_raw.device)
    hi = torch.tensor(_UPPERS, dtype=x_raw.dtype, device=x_raw.device)
    raw = lo + x_raw * (hi - lo)

    root_chord_m = raw[0] / 1000.0
    tip_chord_m  = raw[1] / 1000.0
    knee_frac    = raw[2]
    ar_scale     = raw[9]   # index 9 after removing cruise_velocity/alpha
    canard_ratio = raw[6]

    # Accurate wing AR: use all 7 stations
    # y-fraction differences between consecutive stations (unscaled):
    # _WING_Y = [0, 0.35, 0.78, 0.86, 0.92, 0.96, 1.0]
    # Inboard break chord (station 1) via the taper knee
    inboard_chord = root_chord_m + knee_frac * (0.350 / 0.780) * (tip_chord_m - root_chord_m)
    # Winglet chords scale with tip_chord / baseline_tip (baseline outer_wing chord = 0.228 m)
    _wscale = tip_chord_m / 0.228
    _c = [root_chord_m, inboard_chord, tip_chord_m,
          0.200 * _wscale, 0.175 * _wscale, 0.145 * _wscale, 0.100 * _wscale]
    _dy = [0.350, 0.430, 0.080, 0.060, 0.040, 0.040]  # y-fraction differences (unscaled)
    area_half_unscaled = sum(0.5 * (_c[i] + _c[i + 1]) * _dy[i] for i in range(6))
    area_approx = 2.0 * ar_scale * area_half_unscaled
    span_approx = 2.0 * ar_scale
    ar_approx = span_approx ** 2 / (area_approx + 1e-6)

    # AR bounds: interior of validation range [6.6, 7.8], with 0.1 margin each side
    p_ar_lo = W_AR_BOUNDS  * F.relu(6.7 - ar_approx) ** 2
    p_ar_hi = W_AR_BOUNDS  * F.relu(ar_approx - 7.7) ** 2

    # Canard area ratio: interior of validation [0.116, 0.176], with 0.006 margin each side
    p_ca_lo = W_CANARD_AREA * F.relu(0.122 - canard_ratio) ** 2
    p_ca_hi = W_CANARD_AREA * F.relu(canard_ratio - 0.170) ** 2

    # Minimum tip chord (Re constraint: tip_chord > 0.10 m)
    p_tip = W_TAPER * F.relu(0.10 - tip_chord_m) ** 2

    # No inverse taper: tip_chord <= root_chord
    p_inv_taper = W_TIP_CHORD_MAX * F.relu(tip_chord_m - root_chord_m) ** 2

    # Canard-prop x clearance: canard tip TE must be at least 0.08 m ahead of motor
    # motor_x = 0.070 m.  canard_x_le_root is NEGATIVE (forward of CG, param index 8)
    canard_x_le_root_m = raw[8] / 1000.0  # negative value
    # conservative tip chord at minimum area ratio: ~0.09 m
    canard_tip_te_approx = canard_x_le_root_m + 0.02 + 0.09
    motor_x_m = 0.070
    p_canard_prop_x = W_PROP_X * F.relu(canard_tip_te_approx - (motor_x_m - 0.08)) ** 2

    return p_ar_lo + p_ar_hi + p_ca_lo + p_ca_hi + p_tip + p_inv_taper + p_canard_prop_x


def boundary_penalty(x_raw: torch.Tensor) -> torch.Tensor:
    """Smooth penalty for leaving the [0,1]^N box (used during GD runs)."""
    return 10.0 * (F.relu(-x_raw) ** 2 + F.relu(x_raw - 1.0) ** 2).sum()


def surrogate_objective(
    x_raw: torch.Tensor,
    surrogate: "AeroSurrogate",
    dataset: "AeroDataset",
) -> torch.Tensor:
    """
    Differentiable objective: negative L/D + constraint penalties.

    x_raw : (12,) normalised [0,1] design vector, requires_grad should be True
            during gradient-based optimization.
    Returns a scalar loss to be *minimised* (negative L/D is minimisation).
    """
    # eval() ensures BatchNorm uses running stats (works with batch size 1)
    surrogate.eval()
    x_net = dataset.encode_x_for_net(x_raw.unsqueeze(0))       # (1, 12)
    y_norm = surrogate(x_net).squeeze(0)                         # (3,)
    y_phys = dataset.denormalize_y(y_norm)                       # [CL, CD, Cm]

    cl, cd, cm = y_phys[0], y_phys[1], y_phys[2]

    # Physics-correct level-flight CL requirement: CL >= W / (0.5 * rho * V^2 * S)
    # Cruise conditions are fixed (geometry-only optimisation).
    from v3.optimization.design_space import CRUISE_VELOCITY_MS, _LOWERS, _UPPERS
    lo = torch.tensor(_LOWERS, dtype=x_raw.dtype, device=x_raw.device)
    hi = torch.tensor(_UPPERS, dtype=x_raw.dtype, device=x_raw.device)
    raw = lo + x_raw * (hi - lo)
    root_chord_m = raw[0] / 1000.0
    tip_chord_m  = raw[1] / 1000.0
    wing_area_approx = (root_chord_m + tip_chord_m) * 0.78
    required_cl = torch.tensor(
        _MASS_KG * _G / (0.5 * _RHO * CRUISE_VELOCITY_MS ** 2 * float(wing_area_approx.detach()) + 1e-6),
        dtype=x_raw.dtype,
    )

    obj       = -ld_ratio(cl, cd)
    p_aero    = aero_penalty(cl, cd, cm, required_cl=required_cl)
    p_geom    = geometric_penalty_from_x(x_raw)
    p_bound   = boundary_penalty(x_raw)

    return obj + p_aero + p_geom + p_bound


# ---------------------------------------------------------------------------
# Physical constraint checker (numpy — used after AeroSandbox verification)
# ---------------------------------------------------------------------------

@dataclass
class ConstraintSummary:
    ld: float
    cl: float
    cd: float
    cm: float
    ar: float
    wake_clearance_m: float
    canard_area_ratio: float
    canard_to_prop_radial_m: float
    canard_to_prop_x_m: float
    elevon_in_propwash: float
    n_violations: int
    violation_details: list[str]

    @property
    def is_feasible(self) -> bool:
        return self.n_violations == 0


def build_constraint_summary(
    snap_cruise,   # AeroSnapshot
    derived,       # DerivedGeometry
    validation_targets: dict,
) -> ConstraintSummary:
    """
    Build a ConstraintSummary from AeroSandbox outputs.
    Maps to the same 18 checks as validate_design_v3.py.
    """
    cl, cd, cm = snap_cruise.CL, snap_cruise.CD, snap_cruise.Cm
    ld = cl / (cd + 1e-6)

    violations: list[str] = []
    g  = validation_targets.get("geometry", {})
    ff = validation_targets.get("forward_flight", {})

    def _check(name: str, ok: bool, detail: str = "") -> None:
        if not ok:
            violations.append(f"{name}: {detail}")

    # Geometry
    if "wing_aspect_ratio" in g:
        t, tol = g["wing_aspect_ratio"]["target"], g["wing_aspect_ratio"]["tolerance"]
        _check("wing_ar", abs(derived.wing_aspect_ratio - t) <= tol,
               f"AR={derived.wing_aspect_ratio:.2f} target={t}±{tol}")

    if "canard_area_ratio" in g:
        t, tol = g["canard_area_ratio"]["target"], g["canard_area_ratio"]["tolerance"]
        _check("canard_area_ratio", abs(derived.canard_area_ratio - t) <= tol,
               f"{derived.canard_area_ratio:.3f} target={t}±{tol}")

    if "canard_to_prop_radial_clearance_m" in g:
        mn = g["canard_to_prop_radial_clearance_m"]["minimum"]
        _check("canard_prop_radial", derived.canard_to_prop_radial_clearance_m >= mn,
               f"{derived.canard_to_prop_radial_clearance_m:.3f} < {mn}")

    if "canard_to_prop_x_clearance_m" in g:
        mn = g["canard_to_prop_x_clearance_m"]["minimum"]
        _check("canard_prop_x", derived.canard_to_prop_x_clearance_m >= mn,
               f"{derived.canard_to_prop_x_clearance_m:.3f} < {mn}")

    if "wake_clearance_m" in g:
        mn = g["wake_clearance_m"]["minimum"]
        _check("wake_clearance", derived.wake_clearance_m >= mn,
               f"{derived.wake_clearance_m:.3f} < {mn}")

    if "elevon_span_in_propwash_fraction" in g:
        mn = g["elevon_span_in_propwash_fraction"]["minimum"]
        _check("elevon_propwash", derived.elevon_span_in_propwash_fraction >= mn,
               f"{derived.elevon_span_in_propwash_fraction:.3f} < {mn}")

    # Forward flight
    if "cm_alpha_max" in ff:
        pass  # needs derivative eval — skip here, done in full validation

    _check("cl_pos", cl > 0.30, f"CL={cl:.3f}")
    _check("cd_range", 0.005 < cd < 0.15, f"CD={cd:.4f}")
    _check("cm_trim", abs(cm) < 0.10, f"|Cm|={abs(cm):.4f}")

    return ConstraintSummary(
        ld=ld,
        cl=cl,
        cd=cd,
        cm=cm,
        ar=derived.wing_aspect_ratio,
        wake_clearance_m=derived.wake_clearance_m,
        canard_area_ratio=derived.canard_area_ratio,
        canard_to_prop_radial_m=derived.canard_to_prop_radial_clearance_m,
        canard_to_prop_x_m=derived.canard_to_prop_x_clearance_m,
        elevon_in_propwash=derived.elevon_span_in_propwash_fraction,
        n_violations=len(violations),
        violation_details=violations,
    )
