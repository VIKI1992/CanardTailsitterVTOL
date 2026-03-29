# CanardTailsitterVTOL — Claude Notes

## OpenVSP Python API: Critical Gotchas

### 1. Use `X_Rel_Location` not `X_Location`

When positioning geoms via the Python API, the settable parameter is `X_Rel_Location`
(group `"XForm"`), not `X_Location`. `X_Location` is a read-only derived world-space
coordinate — setting it via `SetParmVal` silently does nothing, leaving the geom at the
origin. Same applies to `Y_Rel_Location` / `Z_Rel_Location`.

```python
# WRONG — geom stays at origin
sp(wing_id, "X_Location", "XForm", -0.1)

# CORRECT
sp(wing_id, "X_Rel_Location", "XForm", -0.1)
```

### 2. Call `SetDriverGroup` before setting wing section Span/Chord

OpenVSP wing sections default to the **AR driver** (`AR_WSECT_DRIVER`), where Span is a
*derived* output. Setting `Span`, `Root_Chord`, `Tip_Chord` without first switching to
`SPAN_WSECT_DRIVER` stores wrong Aspect values in the file and the rendered model will
have completely wrong chord-to-span proportions (chord visually longer than the full
wingspan).

```python
# REQUIRED before every section's Span/Root_Chord/Tip_Chord assignment
vsp.SetDriverGroup(wing_id, i, vsp.SPAN_WSECT_DRIVER,
                               vsp.ROOTC_WSECT_DRIVER,
                               vsp.TIPC_WSECT_DRIVER)
vsp.Update()
sp(wing_id, "Span",       grp, span_m)
sp(wing_id, "Root_Chord", grp, root_m)
sp(wing_id, "Tip_Chord",  grp, tip_m)
```

### 3. OpenVSP has no units — values are interpreted in whatever unit you choose

OpenVSP is unit-agnostic. There are no unit labels in the GUI. All values in this
project are in **metres**. Verify proportions visually (e.g. wingspan ~5.8× root chord).

### 4. `GetParmVal` after WriteVSPFile+ReadVSPFile is buggy

After a write/read roundtrip, `GetParmVal` returns scaled/wrong values (~3.9× for spans,
~18× for chords). The XML file itself contains correct values. This is a Python API bug
that does not affect OpenVSP GUI rendering. Do not use post-roundtrip `GetParmVal` to
validate geometry — grep the XML instead.

### 5. Elevon subsurface: EtaStart/EtaEnd is U-parameter, not physical y-fraction

`EtaStart`/`EtaEnd` on a `SS_CONTROL` subsurface are **linear physical span fractions**
(y / y_tip). Do NOT use equal-U-per-section mapping — that formula places the elevon on
the winglets instead of the main wing.

```python
def y_to_eta(y_m, stations):
    return y_m / stations[-1]["y_m"]
```

**Critical: set `EtaFlag=1` before setting EtaStart/EtaEnd.** The default is `EtaFlag=0`
(length-driven mode), where `vsp.Update()` recomputes EtaStart/EtaEnd from `Length_Start`/
`Length_End`, overriding any values set via `SetParmVal`. With `EtaFlag=1`, Eta values are
used directly and persist through Update().

**Critical: set subsurface parms via the subsurface container ID (returned by `AddSubSurf`),
not via the geom ID.** Use group `"SS_Control"`. Using `wing_id` with group `"SS_Control_1"`
finds the wrong parm or nothing.

```python
elevon_ss = vsp.AddSubSurf(wing_id, vsp.SS_CONTROL, 0)
vsp.Update()
for name, val in [("EtaFlag", 1.0), ("EtaStart", eta_start), ("EtaEnd", eta_end),
                  ("UStart", u_start), ("UEnd", 1.0), ("SE_Const_Flag", 1.0)]:
    p = vsp.FindParm(elevon_ss, name, "SS_Control")
    vsp.SetParmVal(p, val)
```

`UStart` is the hinge chord fraction from LE (e.g. 0.76 = 76% chord). `UEnd` = 1.0
(trailing edge).

## Running the VSP3 builder

```bash
PYTHONPATH=/opt/OpenVSP/python/openvsp:/opt/OpenVSP/python/degen_geom:/opt/OpenVSP/python/utilities \
python3 gen_vsp3_api.py
```

## Opening in OpenVSP

```bash
vsp v3/results/fixed_wing_tailsitter_optimized.vsp3
```
