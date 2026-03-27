# Independent Evaluation Report — V3 Canard Tailsitter

## 1. Artifact Reproducibility

### Geometry Comparison
Re-derived geometry vs saved `derived_geometry_v3.json`:

All 19 geometry fields match within 1e-6 relative tolerance.
- MATCH: `wing_area_m2` saved=0.5663, re-derived=0.5663
- MATCH: `wing_aspect_ratio` saved=7.063394, re-derived=7.063394
- MATCH: `wing_mac_m` saved=0.29565978, re-derived=0.29565978
- MATCH: `wing_mac_y_m` saved=0.44033787, re-derived=0.44033787
- MATCH: `wing_mac_le_x_m` saved=0.0035105068, re-derived=0.0035105068
- MATCH: `wing_quarter_chord_sweep_deg` saved=15.376251, re-derived=15.376251
- MATCH: `wing_projected_span_m` saved=2, re-derived=2
- MATCH: `canard_area_m2` saved=0.096, re-derived=0.096
- MATCH: `canard_area_ratio` saved=0.16952146, re-derived=0.16952146
- MATCH: `canard_quarter_chord_x_m` saved=-0.355, re-derived=-0.355
- MATCH: `canard_tip_y_m` saved=0.32, re-derived=0.32
- MATCH: `canard_tip_trailing_edge_x_m` saved=-0.25, re-derived=-0.25
- MATCH: `main_wing_x_at_canard_tip_m` saved=-0.035771429, re-derived=-0.035771429
- MATCH: `main_wing_z_at_canard_tip_m` saved=0, re-derived=0
- MATCH: `canard_to_prop_radial_clearance_m` saved=0.0385, re-derived=0.0385
- MATCH: `canard_to_prop_x_clearance_m` saved=0.31, re-derived=0.31
- MATCH: `elevon_span_in_propwash_fraction` saved=0.96909091, re-derived=0.96909091
- MATCH: `wake_clearance_m` saved=0.11856719, re-derived=0.11856719
- MATCH: `required_cruise_cl` saved=0.45376015, re-derived=0.45376015

### Metrics Comparison
Re-computed validation metrics vs saved `validation_metrics_v3.json`:

All 24 metrics match within 1e-6 relative tolerance.

## 2. Validation Gate

Standard validation: **18 pass / 0 fail**

## 3. Design Sanity Checks

| # | Check | Status | Detail |
|---|-------|--------|--------|
| 1 | CL surplus at cruise | OK | CL=0.5317, required=0.4538, surplus=0.0779 (17.2%) |
| 2 | Cruise L/D ratio | OK | L/D = 9.34 (typical range for 2m-class UAV: 6-15) |
| 3 | Wing loading | OK | 9.18 kg/m^2 (typical for small UAV: 5-20 kg/m^2) |
| 4 | Static margin estimate | OK | 6.7% of reference chord (from -Cm_alpha/CL_alpha = --0.3336/4.9445) |
| 5 | Canard/elevon pitch authority ratio | INFO | 2.9:1 canard-to-elevon (unpowered). Canard=0.6250/rad, elevon=0.2121/rad |
| 6 | Powered elevon gain consistency | OK | Computed gain=1.550, specified gain=1.550 |
| 7 | Wake clearance margin | OK | Actual=0.1186 m, minimum=0.0400 m, margin=0.0786 m (196.4%) |
| 8 | Canard radial clearance margin | OK | Actual=0.0385 m, minimum=0.0300 m, margin=28.3% |
| 9 | Trim authority usage | OK | Trim uses 2.83 deg of 6.0 deg forward limit (47.2% of authority) |
| 10 | Directional stability adequacy | OK | Cn_beta = 0.06635 /rad (adequate) |
| 11 | Roll damping adequacy | OK | Cl_p = -0.5734 /rad (strong negative = good damping) |
| 12 | Cruise alpha reasonableness | OK | alpha_cruise = 6.0 deg (chosen to achieve CL=0.532 at 18.0 m/s) |

## 4. Summary

- **Artifact reproducibility:** PASS
- **Validation gate:** 18/18 checks pass
- **Sanity checks:** 11 OK, 1 INFO, 0 WARN, 0 FAIL (of 12 total)

## 5. Limitations

- This evaluation re-uses the same AeroSandbox AeroBuildup solver as the
  primary validation pipeline. Systematic model errors are not detectable.
- Powered elevon effectiveness uses a gain proxy (dynamic pressure multiplier),
  not full slipstream-resolved aerodynamics.
- Wake clearance is computed via geometric projection, not CFD or panel method.
- For absolute confidence, validate with VSPAERO (bundled in tools/), XFLR5, or CFD.
