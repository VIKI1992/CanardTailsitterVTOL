#!/usr/bin/env python3
"""
build_onshape.py — Recreate VTOL FDM geometry natively in Onshape via FeatureScript.

Generates FeatureScript code with embedded aircraft coordinates, uploads it to an
Onshape Feature Studio, then instantiates the custom feature in a Part Studio.
All geometry is created as native Onshape solid bodies — no file export.

Namespace discovery: the correct BTMFeature namespace for same-document Feature Studios
is  "e{fs_eid}::m{element_microversionId}"  where element_microversionId comes from
GET /api/v6/documents/d/{did}/w/{wid}/elements → element.microversionId

Usage:
  python3 build_onshape.py [--rebuild]
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

REPO = Path(__file__).parent
GEO_JSON = REPO / "v3" / "results" / "vsp3_geometry.json"
RESULTS_DIR = REPO / "v3" / "results"
BASE_URL = "https://cad.onshape.com"
DOC_STATE_FILE = RESULTS_DIR / "onshape_fdm_doc.json"

# ---------------------------------------------------------------------------
# Credentials
# ---------------------------------------------------------------------------
env_path = REPO / ".env"
if env_path.exists():
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ[k.strip()] = v.strip()

ACCESS_KEY = os.environ["ONSHAPE_ACCESS_KEY"]
SECRET_KEY = os.environ["ONSHAPE_SECRET_KEY"]

_session = requests.Session()
_session.auth = (ACCESS_KEY, SECRET_KEY)
_session.mount("https://", HTTPAdapter(max_retries=Retry(
    total=3, backoff_factor=2, status_forcelist=[502, 503, 504])))

parser = argparse.ArgumentParser()
parser.add_argument("--rebuild", action="store_true",
                    help="Create a fresh Onshape document")
args = parser.parse_args()

geo = json.loads(GEO_JSON.read_text())

# ---------------------------------------------------------------------------
# Airfoil / profile coordinate generation (Python side)
# ---------------------------------------------------------------------------

def naca4_coords(m: float, p: float, t: float, n: int = 30):
    beta = np.linspace(0, np.pi, n + 1)
    x = 0.5 * (1.0 - np.cos(beta))
    yt = 5.0 * t * (0.2969*np.sqrt(x+1e-12) - 0.1260*x
                    - 0.3516*x**2 + 0.2843*x**3 - 0.1015*x**4)
    if m < 1e-9:
        return x, yt, -yt
    yc  = np.where(x < p, m/p**2*(2*p*x - x**2),
                           m/(1-p)**2*((1-2*p) + 2*p*x - x**2))
    dyc = np.where(x < p, 2*m/p**2*(p-x), 2*m/(1-p)**2*(p-x))
    th  = np.arctan(dyc)
    return x, yc + yt*np.cos(th), yc - yt*np.cos(th)


def sd7037_coords():
    x, yu, yl = naca4_coords(m=0.022, p=0.44, t=0.092, n=30)
    mid = (yu + yl) / 2.0
    ht  = (yu - yl) / 2.0
    return x, mid + ht*(12.0/9.2), mid - ht*(12.0/9.2)   # scale to 12% t/c


def blunt_te(x, yu, yl, te_mm, chord_mm):
    te_half = te_mm / (2.0 * chord_mm)
    yu_n, yl_n = yu.copy(), yl.copy()
    te_cam = (yu[-1] + yl[-1]) / 2.0
    du = (te_cam + te_half) - yu[-1]
    dl = (te_cam - te_half) - yl[-1]
    for i in range(len(x)):
        if x[i] >= 0.70:
            a = (x[i] - 0.70) / 0.30
            yu_n[i] += a * du
            yl_n[i] += a * dl
    return x, yu_n, yl_n


def airfoil_loop(x, yu, yl, chord_m, twist_deg=0.0):
    """
    Closed loop [[sx, sy], ...] in metres for an airfoil cross-section.

    Sketch plane convention: normal = -Y_DIRECTION, xDir = X_DIRECTION
      → yDir = Z_DIRECTION
    So (sx, sy) maps to aircraft (X, Z):
      sx = chord fraction × chord_m (LE at origin)
      sy = thickness fraction × chord_m (positive = upper surface)
    Upper surface LE→TE then lower surface TE→LE (closes at LE).
    """
    pts = []
    for xi, yi in zip(x, yu):
        pts.append([float(xi * chord_m), float(yi * chord_m)])
    for xi, yi in zip(x[-2::-1], yl[-2::-1]):
        pts.append([float(xi * chord_m), float(yi * chord_m)])
    if abs(twist_deg) > 0.01:
        qc = 0.25 * chord_m
        th = math.radians(twist_deg)
        c, s = math.cos(th), math.sin(th)
        pts = [[qc + c*(p[0]-qc) - s*p[1], s*(p[0]-qc) + c*p[1]] for p in pts]
    return pts


def ellipse_loop(half_w, half_h, n=24):
    """
    Closed ellipse [[sx, sy], ...] for a fuselage cross-section.

    Sketch plane convention: normal = X_DIRECTION, xDir = Y_DIRECTION
      → yDir = Z_DIRECTION
    So sx = world Y (half-width), sy = world Z (half-height).
    """
    pts = []
    for i in range(2 * n):
        th = 2 * math.pi * i / (2 * n)
        pts.append([float(half_w * math.cos(th)), float(half_h * math.sin(th))])
    return pts


# ---------------------------------------------------------------------------
# FeatureScript code generation
# ---------------------------------------------------------------------------

def _fs_pts(pts, var_name):
    inner = ", ".join(f"[{p[0]:.6f}, {p[1]:.6f}]" for p in pts)
    return f"    var {var_name} = [{inner}];"


def _plane_call(origin_xyz, normal, xdir):
    ox, oy, oz = origin_xyz
    return (f"plane(vector({ox:.4f}, {oy:.4f}, {oz:.4f})*meter, "
            f"{normal}, {xdir})")


def generate_featurescript(geo):
    L = []

    # ---- header ----
    L += [
        "FeatureScript 2931;",
        'import(path : "onshape/std/geometry.fs", version : "2931.0");',
        "",
        "/** Closed polyline profile sketch on a plane. */",
        "function addProfile(context is Context, id is Id, skPlane is Plane, pts is array) returns Query",
        "{",
        '    var sk = newSketchOnPlane(context, id, { "sketchPlane" : skPlane });',
        "    var n = size(pts);",
        "    for (var i = 0; i < n; i += 1) {",
        "        var j = (i + 1) % n;",
        '        skLineSegment(sk, "edge" ~ i, {',
        '            "start" : vector(pts[i][0], pts[i][1]) * meter,',
        '            "end"   : vector(pts[j][0], pts[j][1]) * meter',
        "        });",
        "    }",
        "    skSolve(sk);",
        "    return qSketchRegion(id);",
        "}",
        "",
        'annotation { "Feature Type Name" : "VTOL Aircraft" }',
        "export const vtolAircraft = defineFeature(function(context is Context, id is Id, definition is map)",
        "{",
        "    // Symmetry planes",
        "    const symPlaneY = plane(vector(0, 0, 0) * meter, vector(0, 1, 0));  // XZ — wing/canard mirror",
        "    const symPlaneZ = plane(vector(0, 0, 0) * meter, vector(0, 0, 1));  // XY — fin mirror",
    ]

    # ---- CANARD ----
    cs = geo["canard"]["stations"]
    L.append("    // ===== CANARD: NACA0011 11%, 0.6 mm blunt TE =====")
    for i, st in enumerate(cs):
        chord_m = st["chord_m"]
        x, yu, yl = naca4_coords(0.0, 0.0, 0.11, n=30)
        x, yu, yl = blunt_te(x, yu, yl, 0.6, chord_m * 1000)
        pts = airfoil_loop(x, yu, yl, chord_m, st.get("twist_deg", 0.0))
        L.append(_fs_pts(pts, f"cPts{i}"))
        pl = _plane_call((st["le_x_m"], st["y_m"], st["z_m"]),
                         "-Y_DIRECTION", "X_DIRECTION")
        L.append(f"    var cProf{i} = addProfile(context, id+\"cP{i}\", {pl}, cPts{i});")
    profs = ", ".join(f"cProf{i}" for i in range(len(cs)))
    L += [
        f'    opLoft(context, id+"canardPos", {{',
        f'        "profileSubqueries" : [{profs}]',
        f'    }});',
        f'    opPattern(context, id+"canardNeg", {{',
        f'        "entities"      : qCreatedBy(id+"canardPos", EntityType.BODY),',
        f'        "transforms"    : [ mirrorAcross(symPlaneY) ],',
        f'        "instanceNames" : ["1"]',
        f'    }});',
        "",
    ]

    # ---- MAIN WING ----
    ws = geo["wing"]["stations"]
    L.append("    // ===== MAIN WING: SD7037 12%, 0.6 mm blunt TE =====")
    for i, st in enumerate(ws):
        chord_m = st["chord_m"]
        x, yu, yl = sd7037_coords()
        x, yu, yl = blunt_te(x, yu, yl, 0.6, chord_m * 1000)
        pts = airfoil_loop(x, yu, yl, chord_m, st.get("twist_deg", 0.0))
        L.append(_fs_pts(pts, f"wPts{i}"))
        pl = _plane_call((st["le_x_m"], st["y_m"], st["z_m"]),
                         "-Y_DIRECTION", "X_DIRECTION")
        L.append(f"    var wProf{i} = addProfile(context, id+\"wP{i}\", {pl}, wPts{i});")
    profs = ", ".join(f"wProf{i}" for i in range(len(ws)))
    L += [
        f'    opLoft(context, id+"wingPos", {{',
        f'        "profileSubqueries" : [{profs}]',
        f'    }});',
        f'    opPattern(context, id+"wingNeg", {{',
        f'        "entities"      : qCreatedBy(id+"wingPos", EntityType.BODY),',
        f'        "transforms"    : [ mirrorAcross(symPlaneY) ],',
        f'        "instanceNames" : ["1"]',
        f'    }});',
        "",
    ]

    # ---- FUSELAGE ----
    fuse = geo["fuselage"]["stations"]
    L.append("    // ===== FUSELAGE: elliptical cross-sections =====")
    for i, st in enumerate(fuse):
        x_pos = st["x_m"]
        if st["shape"] == "point":
            r = max(st.get("width_m", 0.005), st.get("height_m", 0.005)) / 2.0
            r = max(r, 0.003)
            pts = ellipse_loop(r, r, n=12)
        else:
            pts = ellipse_loop(st["width_m"] / 2.0, st["height_m"] / 2.0, n=24)
        L.append(_fs_pts(pts, f"fPts{i}"))
        # normal=X, xDir=Y → yDir=Z (width in Y, height in Z) ✓
        pl = _plane_call((x_pos, 0.0, 0.0), "X_DIRECTION", "Y_DIRECTION")
        L.append(f"    var fProf{i} = addProfile(context, id+\"fP{i}\", {pl}, fPts{i});")
    profs = ", ".join(f"fProf{i}" for i in range(len(fuse)))
    L += [
        f'    opLoft(context, id+"fuselage", {{',
        f'        "profileSubqueries" : [{profs}]',
        f'    }});',
        "",
    ]

    # ---- FIN ----
    fin = geo["fin"]
    root_le_x = fin["root_le_x_m"]
    tip_le_x  = root_le_x + math.tan(math.radians(fin["sweep_le_deg"])) * fin["semi_span_m"]
    L.append("    // ===== FIN: NACA0010 10%, 0.6 mm blunt TE, dorsal + ventral =====")
    for i, (le_x, z_pos, chord_m) in enumerate([
        (root_le_x, 0.0,                fin["root_chord_m"]),
        (tip_le_x,  fin["semi_span_m"], fin["tip_chord_m"]),
    ]):
        x, yu, yl = naca4_coords(0.0, 0.0, 0.10, n=30)
        x, yu, yl = blunt_te(x, yu, yl, 0.6, chord_m * 1000)
        pts = airfoil_loop(x, yu, yl, chord_m)
        L.append(_fs_pts(pts, f"finPts{i}"))
        # normal=Z, xDir=X → yDir=Y (fin thickness in Y) ✓
        pl = _plane_call((le_x, 0.0, z_pos), "Z_DIRECTION", "X_DIRECTION")
        L.append(f"    var finProf{i} = addProfile(context, id+\"fiP{i}\", {pl}, finPts{i});")
    L += [
        '    opLoft(context, id+"finDorsal", {',
        '        "profileSubqueries" : [finProf0, finProf1]',
        '    });',
        '    opPattern(context, id+"finVentral", {',
        '        "entities"      : qCreatedBy(id+"finDorsal", EntityType.BODY),',
        '        "transforms"    : [ mirrorAcross(symPlaneZ) ],',
        '        "instanceNames" : ["1"]',
        '    });',
        "",
        "}, {});",
    ]

    return "\n".join(L)


# ---------------------------------------------------------------------------
# Onshape API helpers
# ---------------------------------------------------------------------------

def onshape_api(method, path, **kwargs):
    r = _session.request(method, f"{BASE_URL}{path}", timeout=120, **kwargs)
    if not r.ok:
        print(f"  API {r.status_code}: {r.text[:400]}")
        r.raise_for_status()
    return r.json() if r.content else {}


def get_element_microversion(did, wid, eid):
    """Return the element microversionId from the elements list."""
    elements = onshape_api("GET", f"/api/v6/documents/d/{did}/w/{wid}/elements")
    for el in elements:
        if el["id"] == eid:
            return el.get("microversionId", "")
    raise RuntimeError(f"Element {eid} not found in document")


def get_or_create_document():
    if not args.rebuild and DOC_STATE_FILE.exists():
        state = json.loads(DOC_STATE_FILE.read_text())
        did, wid = state["did"], state["wid"]
        print(f"Reusing doc: {did}")
        return did, wid, state.get("ps_eid"), state.get("fs_eid"), state.get("fs_el_mv")
    print("Creating Onshape document...")
    doc = onshape_api("POST", "/api/v6/documents",
                      json={"name": "CanardTailsitterVTOL_FDM", "isPublic": True})
    did, wid = doc["id"], doc["defaultWorkspace"]["id"]
    DOC_STATE_FILE.write_text(json.dumps({"did": did, "wid": wid}, indent=2))
    print(f"  Created: {did}")
    return did, wid, None, None, None


def save_state(did, wid, ps_eid, fs_eid, fs_el_mv):
    DOC_STATE_FILE.write_text(json.dumps(
        {"did": did, "wid": wid, "ps_eid": ps_eid, "fs_eid": fs_eid, "fs_el_mv": fs_el_mv},
        indent=2))


def find_or_create_feature_studio(did, wid, name="VTOL Generator"):
    """Return existing Feature Studio eid, or create a new one."""
    elements = onshape_api("GET", f"/api/v6/documents/d/{did}/w/{wid}/elements")
    for el in elements:
        if el.get("elementType") == "FEATURESTUDIO" and el.get("name") == name:
            eid = el["id"]
            print(f"Reusing Feature Studio: {eid}")
            return eid
    print(f"Creating Feature Studio '{name}'...")
    r = _session.post(f"{BASE_URL}/api/v6/featurestudios/d/{did}/w/{wid}",
                      json={"name": name}, timeout=30)
    r.raise_for_status()
    eid = r.json()["id"]
    print(f"  Feature Studio: {eid}")
    return eid


def upload_featurescript(did, wid, fs_eid, code):
    size_kb = len(code) / 1e3
    print(f"Uploading FeatureScript ({size_kb:.1f} KB)...")
    onshape_api("POST", f"/api/v6/featurestudios/d/{did}/w/{wid}/e/{fs_eid}",
                json={"contents": code})
    print("  Uploaded.")


def instantiate_feature(did, wid, ps_eid, fs_eid, fs_el_mv):
    """Add the vtolAircraft feature to the Part Studio using the correct namespace format."""
    namespace = f"e{fs_eid}::m{fs_el_mv}"
    print(f"Instantiating vtolAircraft (namespace={namespace[:40]}...)...")
    body = {
        "feature": {
            "btType": "BTMFeature-134",
            "featureType": "vtolAircraft",
            "name": "VTOL Aircraft",
            "suppressed": False,
            "parameters": [],
            "namespace": namespace,
            "returnAfterSubfeatures": False,
            "subFeatures": [],
            "parameterLibraries": [],
        },
        "libraryVersion": 2931,
        "serializationVersion": "1.2.17",
        "microversionSkew": False,
    }
    r = onshape_api("POST",
                    f"/api/v6/partstudios/d/{did}/w/{wid}/e/{ps_eid}/features",
                    json=body)
    states = r.get("featureStates", {})
    all_ok = True
    for fid, st in states.items():
        status = st.get("featureStatus", "?")
        err    = st.get("errorMessage", "")
        if status != "OK":
            print(f"  [{fid}] {status}: {err[:200]}")
            all_ok = False
    if all_ok:
        print("  Feature added: OK")
    return r


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    did, wid, ps_eid_cached, fs_eid_cached, fs_el_mv_cached = get_or_create_document()

    # Find the default Part Studio
    elements = onshape_api("GET", f"/api/v6/documents/d/{did}/w/{wid}/elements")
    ps_list = [e for e in elements if e.get("elementType") == "PARTSTUDIO"]
    if not ps_list:
        raise RuntimeError("No Part Studio in document")
    ps_eid = ps_eid_cached or ps_list[0]["id"]
    print(f"Part Studio: {ps_eid}")

    # Create or reuse Feature Studio
    fs_eid = find_or_create_feature_studio(did, wid)

    # Generate FeatureScript code
    fs_code = generate_featurescript(geo)
    fs_path = RESULTS_DIR / "vtol_aircraft.fs"
    fs_path.write_text(fs_code)
    print(f"FeatureScript: {fs_path.name} ({len(fs_code)/1e3:.1f} KB)")

    # Upload to Onshape Feature Studio
    upload_featurescript(did, wid, fs_eid, fs_code)
    time.sleep(3)   # wait for Onshape to compile the FeatureScript

    # Get element microversionId (used to build the feature namespace)
    fs_el_mv = get_element_microversion(did, wid, fs_eid)
    print(f"Feature Studio microversionId: {fs_el_mv}")
    save_state(did, wid, ps_eid, fs_eid, fs_el_mv)

    # Instantiate custom feature → geometry appears in Part Studio
    instantiate_feature(did, wid, ps_eid, fs_eid, fs_el_mv)

    # Verify bodies
    time.sleep(8)
    parts = onshape_api("GET", f"/api/v6/parts/d/{did}/w/{wid}/e/{ps_eid}")
    print(f"\nBodies in Part Studio: {len(parts)}")
    for p in parts:
        print(f"  - {p.get('name','?')} ({p.get('bodyType','?')})")

    url = f"{BASE_URL}/documents/{did}/w/{wid}/e/{ps_eid}"
    print(f"\nOnshape URL:\n  {url}")
    try:
        import qrcode
        qr = qrcode.QRCode(border=1)
        qr.add_data(url)
        qr.make(fit=True)
        print()
        qr.print_ascii(invert=True)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
