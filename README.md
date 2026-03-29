# Canard Tailsitter VTOL

A parametric design and analysis pipeline for a twin-motor canard fixed-wing tailsitter drone. The aircraft takes off and lands vertically on its tail, then transitions to efficient forward flight.

## Aircraft Overview

| Parameter | Value |
|-----------|-------|
| Wingspan | 2.65 m (total incl. winglets) |
| Wing airfoil | SD7037 |
| Root chord | 455 mm |
| MTOW | 5.2 kg |
| Cruise speed | 18 m/s (65 km/h) |
| Propulsion | Twin fixed counter-rotating props, 533 mm diameter |
| Canard | All-moving, NACA 0009, 600 mm span |
| Elevons | 24% chord, y=0.35-0.75 semi-span (straight wing only) |
| Winglets | Blended transition, 288 mm height |
| Fuselage | 1.02 m length, blunt elliptical nose, 155 mm max width |

## Quick Start

### Prerequisites

- Python 3.12+
- OpenVSP 3.48.2 installed system-wide (`sudo dpkg -i openvsp.deb`)
- `pip install -r requirements.txt`

### Build the VSP3 Model

```bash
PYTHONPATH=/opt/OpenVSP/python/openvsp:/opt/OpenVSP/python/degen_geom:/opt/OpenVSP/python/utilities \
python3 gen_vsp3_api.py
```

Generates `v3/results/fixed_wing_tailsitter_optimized.vsp3` plus STEP, IGES, and STL exports.

### Selective Export (e.g. skip propellers)

```bash
# Exclude specific components
python3 gen_vsp3_api.py --exclude props
python3 gen_vsp3_api.py --exclude props,canard

# Or include only specific components
python3 gen_vsp3_api.py --only wing,fuselage
```

Valid component names: `wing`, `canard`, `fuselage`, `props`

The VSP3 file always contains all components. Only the STEP/IGES/STL exports are filtered. Per-component STL files are also generated for multi-body CAD import.

### Open in OpenVSP

```bash
vsp v3/results/fixed_wing_tailsitter_optimized.vsp3
```

### Upload to Onshape

```bash
python3 upload_onshape.py
```

Combines per-component STLs into a single binary STL and uploads to a new Onshape document. Each component appears as a separate solid body in one Part Studio.

Requires Onshape API credentials in `.env` (not tracked by git):
```
ONSHAPE_ACCESS_KEY=your_access_key
ONSHAPE_SECRET_KEY=your_secret_key
```

### Run Surrogate Optimizer

```bash
pip install -r requirements_optimizer.txt
python3 -m v3.optimization.sweep        # generate training data
python3 -m v3.optimization.surrogate     # train neural surrogate
python3 -m v3.optimization.nn_optimize   # multi-objective optimization
```

## Project Structure

```
CanardTailsitterVTOL/
  gen_vsp3_api.py          Build VSP3 model via OpenVSP Python API
  upload_onshape.py        Upload to Onshape (per-component, multi-body)
  deflection_viz.py        AeroSandbox VLM deflection visualization
  gen_optimized_vsp3.py    Apply optimizer results to VSP3
  prepare_vsp3_data.py     Prepare geometry JSON from optimizer output
  v3/
    optimization/          Surrogate-based optimizer
      design_space.py        Design variable bounds
      sweep.py               Parameter sweep / training data
      surrogate.py           Neural network surrogate model
      nn_optimize.py         Multi-objective optimization
      objectives.py          Objective functions
    results/               Generated artifacts
      vsp3_geometry.json     Geometry definition (source of truth for gen_vsp3_api.py)
      *.vsp3, *.step, *.iges, *.stl   Model exports
    model/                 Typed parameter loader and analysis
  spec/                    Design specification and rationale
  airfoils/                Airfoil coordinate data
  research/                Literature review
  checkpoints/             Surrogate model weights
  data/                    Training sweep data
  results/                 Optimizer output parameters
```

## Design Highlights

- **Canard configuration**: All-moving symmetric canard provides pitch trim and control
- **Propwash-fed elevons**: On the straight wing panel within propeller slipstream for hover authority
- **Smooth winglet transition**: 7-station blended dihedral curve (0-73deg) for directional stability
- **Blunt nose fuselage**: 8-station loft with round end cap; canard root fully enveloped
- **OpenVSP to Onshape pipeline**: Automated export with selective component filtering and per-body import
