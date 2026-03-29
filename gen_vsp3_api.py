"""
gen_vsp3_api.py — Build optimized VSP3 using official OpenVSP Python API.

Run with:
  LD_LIBRARY_PATH=/tmp/libglew_extract/usr/lib/x86_64-linux-gnu:/tmp/libcminpack_extract/usr/lib/x86_64-linux-gnu:/tmp/openvsp_extract/opt/OpenVSP \
  PYTHONPATH=/tmp/openvsp_extract/opt/OpenVSP/python/openvsp:/tmp/openvsp_extract/opt/OpenVSP/python/degen_geom:/tmp/openvsp_extract/opt/OpenVSP/python/utilities \
  /tmp/openvsp_venv/bin/python3 gen_vsp3_api.py
"""
import json
import math
from pathlib import Path

import openvsp as vsp

REPO = Path(__file__).parent
geo = json.loads((REPO / "v3" / "results" / "vsp3_geometry.json").read_text())
out_path = geo["out_vsp3"]


def sp(geom_id: str, parm_name: str, group_name: str, value: float) -> None:
    p = vsp.FindParm(geom_id, parm_name, group_name)
    if not p:
        print(f"  WARNING: parm {group_name}/{parm_name} not found on {geom_id}")
        return
    vsp.SetParmVal(p, value)


# ============================================================
# Helpers for converting station data to VSP section params
# ============================================================

def section_params(s_in: dict, s_out: dict) -> dict:
    """Compute span, sweep, dihedral for one wing/canard section."""
    dy = s_out["y_m"] - s_in["y_m"]
    dz = s_out["z_m"] - s_in["z_m"]
    dx_le = s_out["le_x_m"] - s_in["le_x_m"]
    span_3d = math.sqrt(dy**2 + dz**2)
    dihedral_deg = math.degrees(math.atan2(dz, dy))
    # LE sweep measured in plan view (dy component only — same as VLM convention)
    le_sweep_deg = math.degrees(math.atan2(dx_le, dy))
    return {
        "span": span_3d,
        "root_chord": s_in["chord_m"],
        "tip_chord": s_out["chord_m"],
        "sweep": le_sweep_deg,
        "dihedral": dihedral_deg,
    }


def y_to_eta(y_m: float, stations: list) -> float:
    """Convert a physical y-position to OpenVSP SS_CONTROL EtaStart/EtaEnd.
    OpenVSP parameterises subsurface Eta linearly by physical span fraction (y / y_tip).
    """
    y_tip = stations[-1]["y_m"]
    return y_m / y_tip


# ============================================================
# BUILD MODEL
# ============================================================

vsp.ClearVSPModel()

# ---- WING -------------------------------------------------
ws = geo["wing"]["stations"]  # 7 stations
root_twist = ws[0]["twist_deg"]

wing_id = vsp.AddGeom("WING", "")
vsp.SetGeomName(wing_id, "Wing")

# NOTE: use X_Rel_Location not X_Location — the latter is read-only world coords
sp(wing_id, "X_Rel_Location", "XForm", ws[0]["le_x_m"])
sp(wing_id, "Y_Rel_Location", "XForm", 0.0)
sp(wing_id, "Z_Rel_Location", "XForm", ws[0]["z_m"])
sp(wing_id, "Y_Rotation", "XForm", root_twist)   # root incidence
sp(wing_id, "Sym_Planar_Flag", "Sym", 2.0)        # XZ-plane mirror

# Insert 5 extra XSecs: default has 1 section (XSec_0 root + XSec_1).
# We need 6 sections → 7 XSecs total.
for i in range(5):
    vsp.InsertXSec(wing_id, i + 1, vsp.XS_FOUR_SERIES)
vsp.Update()

xsurf_w = vsp.GetXSecSurf(wing_id, 0)

for i in range(1, 7):
    prm = section_params(ws[i - 1], ws[i])
    grp = f"XSec_{i}"
    # SetDriverGroup REQUIRED before setting Span/Root_Chord/Tip_Chord.
    # Default driver is AR; without switching to SPAN driver, Span is a derived
    # output and the stored Aspect value will be wrong → wrong rendered proportions.
    vsp.SetDriverGroup(wing_id, i, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER)
    vsp.Update()
    sp(wing_id, "Span",          grp, prm["span"])
    sp(wing_id, "Root_Chord",    grp, prm["root_chord"])
    sp(wing_id, "Tip_Chord",     grp, prm["tip_chord"])
    sp(wing_id, "Sweep",         grp, prm["sweep"])
    sp(wing_id, "Sweep_Location",grp, 0.0)   # LE sweep
    sp(wing_id, "Dihedral",      grp, prm["dihedral"])
    # Twist: absolute deviation from root incidence
    sp(wing_id, "Twist",         grp, ws[i]["twist_deg"] - root_twist)
    sp(wing_id, "Twist_Location",grp, 0.25)

    # SD7037 approximation: NACA 4-series, ~9% t/c, 2% camber
    xs_id = vsp.GetXSec(xsurf_w, i)
    p_tc = vsp.GetXSecParm(xs_id, "ThickChord")
    if p_tc:
        vsp.SetParmVal(p_tc, 0.09)
    p_cam = vsp.GetXSecParm(xs_id, "Camber")
    if p_cam:
        vsp.SetParmVal(p_cam, 0.02)

vsp.Update()
print(f"Wing  — TotalSpan={vsp.GetParmVal(vsp.FindParm(wing_id,'TotalSpan','WingGeom')):.3f} m")

# ---- ELEVONS ----------------------------------------------
elevon = geo["elevon"]
eta_start = y_to_eta(elevon["start_y_m"], ws)
eta_end   = y_to_eta(elevon["end_y_m"],   ws)
u_start   = elevon["hinge_frac"]  # chord fraction from LE to hinge

# SS_CONTROL subsurface parms must be set via the subsurface's own container ID
# (returned by AddSubSurf) with group "SS_Control". Using wing_id with group
# "SS_Control_1" finds the wrong parms and leaves EtaStart/EtaEnd at defaults.
elevon_ss = vsp.AddSubSurf(wing_id, vsp.SS_CONTROL, 0)
vsp.Update()

def ss_set(ss_id: str, parm_name: str, value: float) -> None:
    p = vsp.FindParm(ss_id, parm_name, "SS_Control")
    if not p:
        print(f"  WARNING: subsurface parm SS_Control/{parm_name} not found")
        return
    vsp.SetParmVal(p, value)

ss_set(elevon_ss, "EtaFlag",       1.0)   # 1 = use Eta directly; 0 = length-driven (Update() overrides Eta)
ss_set(elevon_ss, "EtaStart",     eta_start)
ss_set(elevon_ss, "EtaEnd",       eta_end)
ss_set(elevon_ss, "UStart",       u_start)
ss_set(elevon_ss, "UEnd",         1.0)
ss_set(elevon_ss, "SE_Const_Flag",1.0)
vsp.Update()
print(f"Elevon — eta=[{eta_start:.3f}, {eta_end:.3f}], hinge={u_start:.2f}c")

# ---- CANARD -----------------------------------------------
cs = geo["canard"]["stations"]  # 2 stations

canard_id = vsp.AddGeom("WING", "")
vsp.SetGeomName(canard_id, "Canard")

sp(canard_id, "X_Rel_Location", "XForm", cs[0]["le_x_m"])
sp(canard_id, "Y_Rel_Location", "XForm", 0.0)
sp(canard_id, "Z_Rel_Location", "XForm", cs[0]["z_m"])
sp(canard_id, "Y_Rotation", "XForm", cs[0]["twist_deg"])
sp(canard_id, "Sym_Planar_Flag", "Sym", 2.0)

prm = section_params(cs[0], cs[1])
vsp.SetDriverGroup(canard_id, 1, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER)
vsp.Update()
sp(canard_id, "Span",          "XSec_1", prm["span"])
sp(canard_id, "Root_Chord",    "XSec_1", prm["root_chord"])
sp(canard_id, "Tip_Chord",     "XSec_1", prm["tip_chord"])
sp(canard_id, "Sweep",         "XSec_1", prm["sweep"])
sp(canard_id, "Sweep_Location","XSec_1", 0.0)
sp(canard_id, "Dihedral",      "XSec_1", prm["dihedral"])
sp(canard_id, "Twist",         "XSec_1", 0.0)

# NACA 0009 for canard
xsurf_c = vsp.GetXSecSurf(canard_id, 0)
xs_c1 = vsp.GetXSec(xsurf_c, 1)
p_tc = vsp.GetXSecParm(xs_c1, "ThickChord")
if p_tc:
    vsp.SetParmVal(p_tc, 0.09)

vsp.Update()
print(f"Canard — span={2*prm['span']:.3f} m, root_chord={prm['root_chord']*1000:.0f} mm")

# ---- FUSELAGE ---------------------------------------------
fs = geo["fuselage"]["stations"]

x_nose  = fs[0]["x_m"]
x_tail  = fs[-1]["x_m"]
fuse_len = x_tail - x_nose

fuse_id = vsp.AddGeom("FUSELAGE", "")
vsp.SetGeomName(fuse_id, "Fuselage")

sp(fuse_id, "X_Rel_Location", "XForm", x_nose)
sp(fuse_id, "Y_Rel_Location", "XForm", 0.0)
sp(fuse_id, "Z_Rel_Location", "XForm", 0.0)
sp(fuse_id, "Length", "Design", fuse_len)

xsurf_f = vsp.GetXSecSurf(fuse_id, 0)

# Default fuselage has 5 XSecs; insert extras if we have more stations
n_default = vsp.GetNumXSec(xsurf_f)
n_needed = len(fs)
for _ in range(n_needed - n_default):
    vsp.InsertXSec(fuse_id, 1, vsp.XS_ELLIPSE)
vsp.Update()

# Configure each XSec — explicitly set shape so nose ellipse overrides default point
for i, st in enumerate(fs):
    x_frac = (st["x_m"] - x_nose) / fuse_len

    if st["shape"] == "point":
        vsp.ChangeXSecShape(xsurf_f, i, vsp.XS_POINT)
    else:
        vsp.ChangeXSecShape(xsurf_f, i, vsp.XS_ELLIPSE)
    vsp.Update()

    xs_id = vsp.GetXSec(xsurf_f, i)
    p_xloc = vsp.GetXSecParm(xs_id, "XLocPercent")
    if p_xloc:
        vsp.SetParmVal(p_xloc, x_frac)

    if st["shape"] != "point":
        p_w = vsp.GetXSecParm(xs_id, "Ellipse_Width")
        p_h = vsp.GetXSecParm(xs_id, "Ellipse_Height")
        if p_w:
            vsp.SetParmVal(p_w, st["width_m"])
        if p_h:
            vsp.SetParmVal(p_h, st["height_m"])

vsp.Update()
print(f"Fuselage — length={fuse_len:.3f} m  ({x_nose:.3f}..{x_tail:.3f}), {n_needed} stations")

# ---- PROPELLERS -------------------------------------------
prop_data = geo["propulsion"]
diameter  = prop_data["prop_diameter_m"]
motor_x   = prop_data["motor_x_m"]
motor_y   = prop_data["motor_y_m"]
motor_z   = prop_data["motor_z_m"]

for side, rot_dir in [(-1, prop_data["left_rotation"]),
                      ( 1, prop_data["right_rotation"])]:
    prop_id = vsp.AddGeom("PROP", "")
    name    = "Prop_Left" if side < 0 else "Prop_Right"
    vsp.SetGeomName(prop_id, name)

    sp(prop_id, "X_Rel_Location", "XForm", motor_x)
    sp(prop_id, "Y_Rel_Location", "XForm", side * motor_y)
    sp(prop_id, "Z_Rel_Location", "XForm", motor_z)

    sp(prop_id, "Diameter",  "Design", diameter)
    sp(prop_id, "NumBlade",  "Design", 2.0)
    # Rotate so prop disk faces forward (+X thrust)
    sp(prop_id, "Y_Rotation", "XForm", 90.0)

    # Rotation direction: ReverseFlag=1 → CW when viewed from front
    sp(prop_id, "ReverseFlag", "Design", 0.0 if rot_dir == "CCW" else 1.0)

vsp.Update()
print(f"Props  — diameter={diameter*1000:.0f} mm, y=±{motor_y:.3f} m")

# ---- WRITE ------------------------------------------------
Path(out_path).parent.mkdir(parents=True, exist_ok=True)
vsp.WriteVSPFile(out_path, 0)
print(f"\nWritten: {out_path}")
vsp.ClearVSPModel()
