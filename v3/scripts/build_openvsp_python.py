#!/usr/bin/env python3
"""Build OpenVSP model using the Python API directly."""
from __future__ import annotations
import sys, os, math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

vsp_base = ROOT / "tools" / "OpenVSP-3.48.2" / "OpenVSP-3.48.2-win64"
for d in ["python", "python/openvsp", "python/utilities", "python/degen_geom", "python/openvsp_config"]:
    p = str(vsp_base / d)
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

import openvsp as vsp
from v3.model.spec import load_parameters
from v3.model.geometry import derive_geometry

params = load_parameters(ROOT / "spec" / "parameters_v3.json")
derived = derive_geometry(params)

vsp.VSPCheckSetup()
vsp.VSPRenew()


def section_geom(a, b):
    dy = b.y_m - a.y_m
    dz = b.z_m - a.z_m
    dx = b.leading_edge_x_m - a.leading_edge_x_m
    span = math.hypot(dy, dz)
    sweep = math.degrees(math.atan2(dx, max(dy, 1e-9)))
    dihedral = math.degrees(math.atan2(dz, max(dy, 1e-9)))
    return span, sweep, dihedral


def set_parm(gid, name, group, val):
    pid = vsp.FindParm(gid, name, group)
    if pid:
        vsp.SetParmVal(pid, val)
    else:
        print(f"  WARN: {name} in {group} not found")


def set_airfoil(gid, xsec_surf, idx, af_name):
    if af_name.upper().startswith("NACA") and len(af_name) >= 8:
        digits = af_name.upper().replace("NACA", "")
        vsp.ChangeXSecShape(xsec_surf, idx, vsp.XS_FOUR_SERIES)
        vsp.Update()
        set_parm(gid, "Camber", f"XSecCurve_{idx}", int(digits[0]) / 100.0)
        set_parm(gid, "CamberLoc", f"XSecCurve_{idx}", int(digits[1]) / 10.0)
        set_parm(gid, "ThickChord", f"XSecCurve_{idx}", int(digits[2:4]) / 100.0)
    else:
        vsp.ChangeXSecShape(xsec_surf, idx, vsp.XS_FILE_AIRFOIL)
        vsp.Update()
        xsec = vsp.GetXSec(xsec_surf, idx)
        vsp.ReadFileAirfoil(xsec, str(ROOT / "airfoils" / "sd7037.dat"))


def build_wing(name, stations, airfoils, root_twist, sym=True):
    gid = vsp.AddGeom("WING", "")
    vsp.SetGeomName(gid, name)
    root = stations[0]
    set_parm(gid, "X_Rel_Location", "XForm", root.leading_edge_x_m)
    set_parm(gid, "Y_Rel_Location", "XForm", root.y_m)
    set_parm(gid, "Z_Rel_Location", "XForm", root.z_m)
    set_parm(gid, "Y_Rel_Rotation", "XForm", root_twist)
    set_parm(gid, "RotateAirfoilMatchDideralFlag", "WingGeom", 1.0)
    if sym:
        set_parm(gid, "Sym_Planar_Flag", "Sym", 2.0)

    n_extra = len(stations) - 2
    for i in range(n_extra):
        vsp.InsertXSec(gid, i + 1, vsp.XS_FOUR_SERIES)
    vsp.Update()

    xsec_surf = vsp.GetXSecSurf(gid, 0)
    for idx in range(len(stations)):
        af = airfoils[idx] if idx < len(airfoils) else airfoils[-1]
        set_airfoil(gid, xsec_surf, idx, af)

    for i, (left, right) in enumerate(zip(stations[:-1], stations[1:]), start=1):
        span, sweep, dihedral = section_geom(left, right)
        twist_delta = right.twist_deg - stations[0].twist_deg
        vsp.SetDriverGroup(gid, i, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER)
        vsp.Update()
        set_parm(gid, "Span", f"XSec_{i}", span)
        set_parm(gid, "Root_Chord", f"XSec_{i}", left.chord_m)
        set_parm(gid, "Tip_Chord", f"XSec_{i}", right.chord_m)
        set_parm(gid, "Sweep", f"XSec_{i}", sweep)
        set_parm(gid, "Sweep_Location", f"XSec_{i}", 0.0)
        set_parm(gid, "Dihedral", f"XSec_{i}", dihedral)
        set_parm(gid, "Twist", f"XSec_{i}", twist_delta)
        set_parm(gid, "SectTess_U", f"XSec_{i}", 12)
        vsp.Update()

    return gid


def add_subsurface(gid, eta_start, eta_end, u_start, u_end):
    """Add SS_CONTROL subsurface with EtaFlag=1 for reliable positioning."""
    ss_id = vsp.AddSubSurf(gid, vsp.SS_CONTROL, 0)
    vsp.Update()
    all_parms = vsp.GetGeomParmIDs(gid)
    ss_parms = {}
    for pid in all_parms:
        pname = vsp.GetParmName(pid)
        if pname in ("EtaStart", "EtaEnd", "UStart", "UEnd", "EtaFlag", "SE_Const_Flag"):
            ss_parms[pname] = pid

    if "EtaFlag" in ss_parms:
        vsp.SetParmValUpdate(ss_parms["EtaFlag"], 1.0)
    if "SE_Const_Flag" in ss_parms:
        vsp.SetParmValUpdate(ss_parms["SE_Const_Flag"], 1.0)
    if "EtaStart" in ss_parms:
        vsp.SetParmValUpdate(ss_parms["EtaStart"], eta_start)
    if "EtaEnd" in ss_parms:
        vsp.SetParmValUpdate(ss_parms["EtaEnd"], eta_end)
    vsp.Update()
    # Set chordwise position after eta
    if "UStart" in ss_parms:
        vsp.SetParmValUpdate(ss_parms["UStart"], u_start)
    if "UEnd" in ss_parms:
        vsp.SetParmValUpdate(ss_parms["UEnd"], u_end)
    vsp.Update()

    # Verify
    for name in ("EtaStart", "EtaEnd"):
        if name in ss_parms:
            print(f"  {name} = {vsp.GetParmVal(ss_parms[name]):.4f}")
    return ss_id


# ═══════════════════════════════════════════
# MAIN WING — single geometry, 5 stations (3 flat + 2 winglet), smooth transition
# ═══════════════════════════════════════════
all_stations = params.wing.stations
n_main = max(1, len(all_stations) - 3)
wing_airfoils = [params.wing.airfoil] * n_main + ["NACA0009"] * (len(all_stations) - n_main)
wing_id = build_wing("MainWing", all_stations, wing_airfoils, all_stations[0].twist_deg)

proj = vsp.GetParmVal(vsp.FindParm(wing_id, "TotalProjectedSpan", "WingGeom"))
print(f"MainWing TotalProjectedSpan: {proj:.4f} m")
for i in range(1, len(all_stations)):
    s = vsp.GetParmVal(vsp.FindParm(wing_id, "Span", f"XSec_{i}"))
    rc = vsp.GetParmVal(vsp.FindParm(wing_id, "Root_Chord", f"XSec_{i}"))
    tc = vsp.GetParmVal(vsp.FindParm(wing_id, "Tip_Chord", f"XSec_{i}"))
    print(f"  Sec{i}: Span={s:.4f} Root={rc:.4f} Tip={tc:.4f}")

# ═══════════════════════════════════════════
# ELEVON — on flat wing only (sections 1-2 out of 4)
# With 4 sections: sec1=eta 0-0.25, sec2=0.25-0.50, sec3=0.50-0.75, sec4=0.75-1.0
# Elevon on section 2 (inboard_break to wing_tip): eta 0.25 to 0.50
# ═══════════════════════════════════════════
elevon = params.control_surfaces["elevon_left"]
n_sections = len(all_stations) - 1  # 4

# Compute eta for elevon y positions within the main wing sections only
def y_to_eta(y_m):
    for i, (a, b) in enumerate(zip(all_stations[:-1], all_stations[1:])):
        if y_m <= b.y_m:
            t = (y_m - a.y_m) / max(b.y_m - a.y_m, 1e-9)
            return (i + t) / n_sections
    return 1.0

# Clamp elevon end to last flat station (z < 0.01 threshold)
wing_tip_y = max(s.y_m for s in all_stations if s.z_m < 0.01)
eta_start = y_to_eta(elevon.start_y_m)
eta_end = y_to_eta(min(elevon.end_y_m, wing_tip_y - 0.01))  # stop just before winglet
print(f"Elevon eta: {eta_start:.4f} to {eta_end:.4f} (sections 1-2 of {n_sections})")

add_subsurface(wing_id, eta_start, eta_end, elevon.hinge_fraction_from_le, 1.0)

# ═══════════════════════════════════════════
# CANARD — full-flying with pivot at 33% chord
# ═══════════════════════════════════════════
canard_airfoils = [params.canard.airfoil] * len(params.canard.stations)
canard_id = build_wing("Canard", params.canard.stations, canard_airfoils, params.canard.stations[0].twist_deg)
canard_proj = vsp.GetParmVal(vsp.FindParm(canard_id, "TotalProjectedSpan", "WingGeom"))
print(f"Canard TotalProjectedSpan: {canard_proj:.4f} m")

# Canard: clean all-moving surface, no hinge lines or subsurfaces.
# Pivots as one unit for pitch control — not represented visually in OpenVSP.
print("Canard: clean surface (no subsurfaces)")

# ═══════════════════════════════════════════
# FUSELAGE — built with per-station verification
# ═══════════════════════════════════════════
fuse_id = vsp.AddGeom("FUSELAGE", "")
vsp.SetGeomName(fuse_id, "Fuselage")
fstations = params.fuselage.stations
base_x = fstations[0].x_m
length = fstations[-1].x_m - base_x
set_parm(fuse_id, "X_Rel_Location", "XForm", base_x)
set_parm(fuse_id, "Length", "Design", length)

# Insert extra xsecs if needed (default fuselage has 5)
n_extra = len(fstations) - 5
if n_extra > 0:
    for i in range(n_extra):
        vsp.InsertXSec(fuse_id, i + 1, vsp.XS_ELLIPSE)
        vsp.Update()

xsec_surf_f = vsp.GetXSecSurf(fuse_id, 0)
n_xsec = vsp.GetNumXSec(xsec_surf_f)
print(f"Fuselage: {n_xsec} XSecs (need {len(fstations)})")

# Set shapes first
shape_map = {"point": vsp.XS_POINT, "ellipse": vsp.XS_ELLIPSE, "rounded_rectangle": vsp.XS_ROUNDED_RECTANGLE}
for idx, st in enumerate(fstations):
    vsp.ChangeXSecShape(xsec_surf_f, idx, shape_map[st.shape.lower()])
vsp.Update()

# Set positions and dimensions one at a time with SetParmValUpdate + verification
for idx, st in enumerate(fstations):
    xsec = vsp.GetXSec(xsec_surf_f, idx)
    target_xloc = (st.x_m - base_x) / length
    if idx > 0:
        pid_x = vsp.FindParm(fuse_id, "XLocPercent", f"XSec_{idx}")
        if pid_x:
            vsp.SetParmValUpdate(pid_x, target_xloc)
            actual = vsp.GetParmVal(pid_x)
            if abs(actual - target_xloc) > 0.01:
                print(f"  WARN XSec_{idx} XLocPercent: target={target_xloc:.3f} actual={actual:.3f}")
        pid_z = vsp.FindParm(fuse_id, "ZLocPercent", f"XSec_{idx}")
        if pid_z:
            vsp.SetParmValUpdate(pid_z, 0.0)
    if st.shape.lower() == "ellipse":
        pid_w = vsp.FindParm(fuse_id, "Ellipse_Width", f"XSecCurve_{idx}")
        pid_h = vsp.FindParm(fuse_id, "Ellipse_Height", f"XSecCurve_{idx}")
        if pid_w: vsp.SetParmValUpdate(pid_w, st.width_m)
        if pid_h: vsp.SetParmValUpdate(pid_h, st.height_m)
    vsp.Update()

# Verify smoothness: sample width at evenly spaced points
print("Fuselage width at 15 stations (should be smooth):")
for j in range(15):
    u = j / 14.0
    pnt_r = vsp.CompPnt01(fuse_id, 0, u, 0.0)   # right side
    pnt_l = vsp.CompPnt01(fuse_id, 0, u, 0.5)   # left side
    width = abs(pnt_r.y() - pnt_l.y()) * 1000
    print(f"  u={u:.2f}  width={width:.0f}mm")

# ═══════════════════════════════════════════
# PROPS
# ═══════════════════════════════════════════
for side, y_sign in [("LeftProp", 1.0), ("RightProp", -1.0)]:
    pid = vsp.AddGeom("PROP", "")
    vsp.SetGeomName(pid, side)
    set_parm(pid, "PropMode", "Design", 1.0)  # disk mode
    set_parm(pid, "Diameter", "Design", params.propulsion.prop_diameter_m)
    set_parm(pid, "X_Rel_Location", "XForm", params.propulsion.motor_x_m)
    set_parm(pid, "Y_Rel_Location", "XForm", y_sign * params.propulsion.motor_y_m)
    set_parm(pid, "Z_Rel_Location", "XForm", params.propulsion.motor_z_m)
vsp.Update()

# ═══════════════════════════════════════════
# VERIFY prop clearance
# ═══════════════════════════════════════════
motor_x = params.propulsion.motor_x_m
motor_y = params.propulsion.motor_y_m
prop_r = params.propulsion.prop_radius_m
for a, b in zip(all_stations[:-1], all_stations[1:]):
    if a.y_m <= motor_y <= b.y_m:
        t = (motor_y - a.y_m) / (b.y_m - a.y_m)
        wing_le = a.leading_edge_x_m + t * (b.leading_edge_x_m - a.leading_edge_x_m)
        gap = wing_le - (motor_x + prop_r)
        print(f"\nProp clearance: prop aft edge x={motor_x + prop_r:.4f}, wing LE x={wing_le:.4f}, gap={gap*1000:.0f}mm")
        break

# ═══════════════════════════════════════════
# SAVE
# ═══════════════════════════════════════════
out_path = str(ROOT / "v3" / "results" / "fixed_wing_tailsitter_v3.vsp3")
vsp.WriteVSPFile(out_path, vsp.SET_ALL)
print(f"\nWrote {out_path}")
print(f"Wing area: {derived.wing_area_m2:.4f} m², Root chord: {all_stations[0].chord_m:.3f} m")
