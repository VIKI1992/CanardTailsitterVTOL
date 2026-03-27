# Canard Tailsitter V3 Design Specification

## Design Intent
This is a clean-sheet 2 m class fixed-wing tailsitter optimized for efficient forward flight while retaining hover and transition controllability through:
- Twin fixed propellers.
- Propwash-fed elevons.
- A symmetric all-moving lifting canard.
- Active control allocation across thrust, elevons, and canard.

The geometry is intentionally not derived from any previous workspace model.

## Baseline Configuration
### Main Wing
- Airfoil: `SD7037`
- Symmetry: mirrored about aircraft centerline
- Stations:
  - Root: `x_le=0.000 m`, `y=0.000 m`, `z=0.000 m`, `chord=0.380 m`, `twist=1.0 deg`
  - Inboard break: `x_le=0.030 m`, `y=0.340 m`, `z=0.000 m`, `chord=0.300 m`, `twist=0.5 deg`
  - Outer control station: `x_le=0.090 m`, `y=0.820 m`, `z=0.030 m`, `chord=0.180 m`, `twist=-1.5 deg`
  - Tip transition: `x_le=0.160 m`, `y=0.930 m`, `z=0.220 m`, `chord=0.170 m`, `twist=-1.0 deg`
  - Fin tip: `x_le=0.220 m`, `y=1.040 m`, `z=0.480 m`, `chord=0.140 m`, `twist=0.0 deg`
- Geometry intent:
  - Moderate sweep, not a flying-wing style high-sweep planform.
  - Continuous tip transition into an upturned winglet/fin structure.
  - Washout retained outboard of the prop disks.

### Canard
- Airfoil: `NACA0009`
- Type: full-flying symmetric canard
- Pivot fraction: `0.33` of local chord from leading edge
- Baseline incidence: `2.0 deg`
- Stations:
  - Root: `x_le=-0.530 m`, `y=0.000 m`, `z=0.360 m`, `chord=0.180 m`, `twist=2.0 deg`
  - Tip: `x_le=-0.500 m`, `y=0.320 m`, `z=0.360 m`, `chord=0.120 m`, `twist=2.0 deg`
- Placement intent:
  - Sits above the main-wing plane.
  - Lies ahead of the prop disks in x.
  - Lies inboard of the prop-disk inboard edge in y.

### Elevons
- Surface type: independent left and right trailing-edge controls
- Span extent: `y=0.430 m` to `y=0.860 m`
- Chord fraction: `0.24`
- Hinge line fraction from leading edge: `0.76`
- Commanded forward-flight limit: `+-18 deg`
- Commanded hover/transition limit: `+-22 deg`
- Mechanical hard-stop assumption: `+-25 deg`

### Tip Stability Surfaces
- Implemented as the outer two wing stations, creating a continuous wing-to-fin transition.
- No separate detached winglet geometry is permitted in the baseline.

### Fuselage
- Geometry type: smooth lofted fuselage
- Cross-section family: ellipse and rounded-rectangle blend
- Stations:
  - Nose point: `x=-0.330 m`, `width=0.010 m`, `height=0.020 m`, `shape=point`
  - Forward nose: `x=-0.260 m`, `width=0.055 m`, `height=0.065 m`, `shape=ellipse`
  - Avionics shoulder: `x=-0.160 m`, `width=0.105 m`, `height=0.125 m`, `shape=ellipse`
  - Battery bay front: `x=-0.020 m`, `width=0.145 m`, `height=0.175 m`, `shape=rounded_rectangle`
  - Wing carry-through: `x=0.180 m`, `width=0.165 m`, `height=0.190 m`, `shape=rounded_rectangle`
  - Payload bay aft: `x=0.360 m`, `width=0.160 m`, `height=0.175 m`, `shape=ellipse`
  - Aft taper: `x=0.560 m`, `width=0.100 m`, `height=0.110 m`, `shape=ellipse`
  - Tail point: `x=0.760 m`, `width=0.020 m`, `height=0.030 m`, `shape=point`
- Continuity requirements:
  - Continuous curvature through nose, shoulder, and tail taper.
  - No flat cylindrical mid-body.
  - No abrupt wing-root shoulder or broken taper.

### Propulsion
- Motor count: `2`
- Propeller diameter: `0.533 m`
- Propeller radius: `0.2665 m`
- Motor positions:
  - Left: `x=-0.050 m`, `y=0.640 m`, `z=0.035 m`
  - Right: mirrored
- Rotation:
  - Left motor: `CW`
  - Right motor: `CCW`
- Design intent:
  - Propwash must intersect most of the elevon span.
  - Canard must remain outside the propwash columns in the baseline geometry.

## Mass and Flight Condition Assumptions
- Analysis mass: `5.2 kg`
- Reference CG location: `x=0.000 m`
- Reference cruise speed: `18.0 m/s`
- Reference trim alpha for evaluation: `6.0 deg`
- Transition analysis speed: `12.0 m/s`
- Air density: `1.225 kg/m^3`

## Control Envelope
### Canard Symmetric Command
- Cruise trim target band: `+-4 deg`
- Forward-flight command limit: `+-6 deg`
- Transition limit: `+-15 deg`
- Mechanical hard-stop assumption: `+-22 deg`

### Elevons
- Cruise trim usage: secondary pitch trim and primary roll control
- Hover and transition usage: primary aerodynamic pitch and roll surface

### Differential Thrust
- Primary yaw control in hover and low-speed transition
- Supplemental roll authority in hover when elevons approach saturation

## Control Allocation
### Cruise
- Pitch priority: `canard_symmetric`, then symmetric elevon bias.
- Roll priority: differential elevons.
- Yaw priority: differential thrust only for correction or contingency.

### Transition
- Pitch priority: symmetric elevons and canard together, with canard gain scheduled up as dynamic pressure builds.
- Roll priority: differential elevons, then differential thrust.
- Yaw priority: differential thrust.

### Hover
- Pitch priority: symmetric elevons in propwash.
- Roll priority: differential elevons, then differential thrust.
- Canard usage: scheduled but deweighted relative to forward flight because it is not in main propwash.

## Validation Rules
1. The canard wake projection at maximum planned deflection must remain above the main-wing reference plane by at least `0.040 m` at the projected interception station.
2. The canard tip must remain at least `0.050 m` inboard of the prop-disk inboard edge.
3. The canard trailing-edge reference line must remain ahead of the prop-disk plane by at least `0.080 m`.
4. At least `60 percent` of each elevon span must lie inside the corresponding prop-disk projected column.
5. The main wing must trim in cruise with canard deflection inside the `+-4 deg` target band.
6. Longitudinal stability may be near-neutral, but the design must show usable trim and positive pitch control effectiveness.
