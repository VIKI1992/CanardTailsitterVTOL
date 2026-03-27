# Canard Tailsitter VTOL

A parametric design and analysis pipeline for a twin-motor canard fixed-wing tailsitter drone. The aircraft takes off and lands vertically on its tail, then transitions to efficient forward flight.

## Aircraft Overview

| Parameter | Value |
|-----------|-------|
| Wingspan | 2.0 m |
| Wing area | 0.56 m² |
| Aspect ratio | ~9 |
| Root chord | 380 mm |
| MTOW | 5.2 kg |
| Cruise speed | 18 m/s (65 km/h) |
| Propulsion | Twin fixed counter-rotating props, 533 mm (21") diameter |
| Canard | All-moving, NACA 0009, pivots at 33% chord |
| Wing airfoil | SD7037 |
| Elevons | 24% chord, propwash-fed for hover pitch/roll control |
| Winglets | Blended transition, 250 mm height, NACA 0009 |

## Project Structure

```
FixedWingTaisitterV2/
  airfoils/           Airfoil coordinate data (sd7037.dat)
  research/           Research digest informing the design
  spec/               Authoritative design specification (source of truth)
    parameters_v3.json        All geometry, mass, propulsion, analysis parameters
    validation_targets_v3.json  Pass/fail thresholds
    control_model_v3.json       Control allocation per flight regime
    design_spec_v3.md           Design specification document
    design_rationale_v4.md      Every design decision with sources
  tools/              Bundled OpenVSP 3.48.2 (win64)
  v3/                 Active implementation
    model/
      spec.py           Typed dataclass parameter loader
      geometry.py       Derived geometry calculations
      analysis.py       AeroSandbox aerodynamic validation
      openvsp_builder.py  OpenVSP vspscript generator (legacy path)
      __init__.py       Package API re-exports
    scripts/
      build_openvsp_python.py   Build OpenVSP model via Python API (primary)
      build_openvsp_v3.py       Build via vspscript (legacy path)
      validate_design_v3.py     Run 18-check validation suite
      evaluate_independent_v3.py  Independent sanity checks
    results/            Generated artifacts (vsp3, reports, metrics)
  requirements.txt    Python dependencies
```

## Quick Start

### Prerequisites

- Python 3.13 (installed locally)
- Dependencies: `pip install -r requirements.txt`
- OpenVSP 3.48.2 is bundled under `tools/` (no separate install needed)

### Build the OpenVSP Model

```powershell
python v3/scripts/build_openvsp_python.py
```

This generates `v3/results/fixed_wing_tailsitter_v3.vsp3` using the OpenVSP Python API directly. The model includes the main wing with smooth winglet transition, canard, fuselage, and prop disks with correct dimensions and symmetry.

### Run Validation

```powershell
python v3/scripts/validate_design_v3.py
```

Runs 18 checks covering geometry (area, AR, sweep, clearances), stability (Cm_alpha, Cn_beta, damping), trim, and control effectiveness. Current status: **18 pass / 0 fail**.

### Run Independent Evaluation

```powershell
python v3/scripts/evaluate_independent_v3.py
```

Re-derives all geometry and metrics from scratch, compares against saved artifacts, and runs additional sanity checks (L/D, wing loading, static margin, control power ratios). Outputs `v3/results/independent_evaluation_v3.md`.

### Open in OpenVSP

```powershell
& 'tools\OpenVSP-3.48.2\OpenVSP-3.48.2-win64\vsp.exe' 'v3\results\fixed_wing_tailsitter_v3.vsp3'
```

## Design Highlights

- **Canard configuration**: All-moving symmetric canard at the nose tip provides pitch trim and control. Pivots as one unit around an axis at 33% chord from the leading edge.
- **Propwash-fed elevons**: Elevons on the main wing panel (35-78% semi-span) sit within the propeller slipstream for hover/transition pitch and roll authority.
- **Smooth winglet transition**: Wing-to-winglet blends through 7 stations with a gradual dihedral sweep curve (0° to 73°), providing directional stability without a separate vertical tail.
- **Control allocation**: Blended across flight regimes — canard dominates pitch in cruise, elevons dominate in hover, differential thrust provides yaw.

## Key Design Documents

- [Design Rationale](spec/design_rationale_v4.md) — every geometry decision traced to a source (Scholz wing design handbook, NASA canard studies, real drone surveys)
- [Design Specification](spec/design_spec_v3.md) — configuration overview
- [Research Digest](research/research_digest.md) — literature review informing the design
- [Control Model](spec/control_model_v3.json) — actuator allocation for cruise/transition/hover

## Validation Summary

| Category | Checks | Status |
|----------|--------|--------|
| Wing geometry (area, AR, sweep) | 4 | Pass |
| Clearances (canard-prop, wake, propwash) | 4 | Pass |
| Stability (Cm_alpha, Cn_beta, damping) | 6 | Pass |
| Trim & control effectiveness | 4 | Pass |
| **Total** | **18** | **18 pass / 0 fail** |

## Dependencies

Listed in `requirements.txt`:
- `aerosandbox` — vortex lattice / aero buildup solver
- `numpy` — numerical computation
- `scipy` — optimization and interpolation

OpenVSP 3.48.2 is bundled in `tools/` with its Python packages (`openvsp`, `degen_geom`, `utilities`, `openvsp_config`).
