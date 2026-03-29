"""
upload_onshape.py — Upload per-component STL files to Onshape.

Combines per-component STLs into a single binary STL, then uploads to
one Onshape document. Each disconnected mesh region becomes a separate
body in a single Part Studio.

Reads credentials from .env (ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY).
"""
import os
import struct
import time
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
from pathlib import Path

REPO = Path(__file__).parent
BASE_URL = "https://cad.onshape.com"
RESULTS_DIR = REPO / "v3" / "results"

# Load credentials from .env
env_path = REPO / ".env"
if env_path.exists():
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ[k.strip()] = v.strip()

ACCESS_KEY = os.environ["ONSHAPE_ACCESS_KEY"]
SECRET_KEY = os.environ["ONSHAPE_SECRET_KEY"]

# Session with retries
session = requests.Session()
session.auth = (ACCESS_KEY, SECRET_KEY)
retries = Retry(total=3, backoff_factor=2, status_forcelist=[502, 503, 504])
session.mount("https://", HTTPAdapter(max_retries=retries))


def parse_ascii_stl(path):
    """Parse ASCII STL, return list of (normal, [v0, v1, v2]) tuples."""
    triangles = []
    with open(path) as f:
        normal = None
        verts = []
        for line in f:
            line = line.strip()
            if line.startswith("facet normal"):
                parts = line.split()
                normal = [float(parts[2]), float(parts[3]), float(parts[4])]
                verts = []
            elif line.startswith("vertex"):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("endfacet"):
                if normal and len(verts) == 3:
                    triangles.append((normal, verts))
    return triangles


def write_binary_stl(triangles, path):
    """Write triangles to binary STL."""
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(triangles)))
        for normal, verts in triangles:
            f.write(struct.pack("<3f", *normal))
            for v in verts:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


def api(method, path, **kwargs):
    url = f"{BASE_URL}{path}"
    r = session.request(method, url, timeout=300, **kwargs)
    if not r.ok:
        print(f"  API error {r.status_code}: {r.text[:500]}")
        r.raise_for_status()
    return r.json() if r.content else {}


# Find per-component STL files
component_stls = sorted(RESULTS_DIR.glob("fixed_wing_tailsitter_optimized_*.stl"))
component_stls = [p for p in component_stls if ".bin." not in p.name and "_compgeom" not in p.name]

if not component_stls:
    print("No per-component STL files found. Run gen_vsp3_api.py first.")
    raise SystemExit(1)

# Combine all component STLs into one binary STL
print("Combining per-component STLs:")
all_triangles = []
for stl_path in component_stls:
    comp_name = stl_path.stem.split("_")[-1]
    tris = parse_ascii_stl(stl_path)
    all_triangles.extend(tris)
    print(f"  {comp_name:10s} {len(tris):6d} triangles")

combined_path = RESULTS_DIR / "fixed_wing_tailsitter_optimized_combined.bin.stl"
write_binary_stl(all_triangles, combined_path)
print(f"  Total:     {len(all_triangles):6d} triangles ({combined_path.stat().st_size / 1e6:.1f} MB)")

# Create Onshape document
print("\nCreating Onshape document...")
doc = api("POST", "/api/v6/documents", json={
    "name": "CanardTailsitterVTOL",
    "isPublic": True,
})
did = doc["id"]
wid = doc["defaultWorkspace"]["id"]
print(f"  Document ID: {did}")

# Upload combined binary STL
print("Uploading to Onshape...")
with open(combined_path, "rb") as f:
    r = session.post(
        f"{BASE_URL}/api/v6/translations/d/{did}/w/{wid}",
        files={"file": ("CanardTailsitterVTOL.stl", f, "application/octet-stream")},
        data={
            "storeInDocument": "true",
            "allowFaultyParts": "true",
            "flattenAssemblies": "true",
            "yAxisIsUp": "false",
        },
        timeout=300,
    )
    r.raise_for_status()

tid = r.json().get("id")
print(f"  Translation ID: {tid}")

# Poll until done
for _ in range(30):
    time.sleep(3)
    st = api("GET", f"/api/v6/translations/{tid}")
    state = st.get("requestState")
    if state == "DONE":
        print("  Translation complete!")
        break
    elif state == "FAILED":
        print(f"  Translation FAILED: {st.get('failureReason')}")
        break
    print(f"  {state}...")

# Verify
time.sleep(2)
print("\nOnshape document contents:")
elements = api("GET", f"/api/v6/documents/d/{did}/w/{wid}/elements")
for el in elements:
    if el.get("elementType") == "PARTSTUDIO":
        eid = el["id"]
        parts = api("GET", f"/api/v6/parts/d/{did}/w/{wid}/e/{eid}")
        if parts:
            print(f"  Part Studio '{el['name']}': {len(parts)} body/bodies")
            for p in parts:
                print(f"    - {p.get('name')} ({p.get('bodyType')})")

url = f"{BASE_URL}/documents/{did}/w/{wid}"
print(f"\nOpen in Onshape: {url}")
