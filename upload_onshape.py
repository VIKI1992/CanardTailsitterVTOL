"""
upload_onshape.py — Upload model to Onshape via REST API.

Converts the STL to binary format and imports it into a new Onshape document.
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

STL_FILE = REPO / "v3" / "results" / "fixed_wing_tailsitter_optimized.stl"
BINARY_STL = STL_FILE.with_suffix(".bin.stl")

# Session with retries
session = requests.Session()
session.auth = (ACCESS_KEY, SECRET_KEY)
retries = Retry(total=3, backoff_factor=2, status_forcelist=[502, 503, 504])
session.mount("https://", HTTPAdapter(max_retries=retries))


def ascii_to_binary_stl(ascii_path, binary_path):
    """Convert ASCII STL to binary STL (required for Onshape import)."""
    triangles = []
    with open(ascii_path) as f:
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
    with open(binary_path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(triangles)))
        for normal, verts in triangles:
            f.write(struct.pack("<3f", *normal))
            for v in verts:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))
    return len(triangles)


def api(method, path, **kwargs):
    url = f"{BASE_URL}{path}"
    r = session.request(method, url, timeout=300, **kwargs)
    if not r.ok:
        print(f"  API error {r.status_code}: {r.text[:500]}")
        r.raise_for_status()
    return r.json() if r.content else {}


# 1. Convert ASCII STL to binary
print(f"Converting {STL_FILE.name} to binary STL...")
n_tris = ascii_to_binary_stl(STL_FILE, BINARY_STL)
print(f"  {n_tris} triangles, {BINARY_STL.stat().st_size / 1e6:.1f} MB")

# 2. Create Onshape document
print("Creating Onshape document...")
doc = api("POST", "/api/v6/documents", json={
    "name": "CanardTailsitterVTOL",
    "isPublic": True,
})
did = doc["id"]
wid = doc["defaultWorkspace"]["id"]
print(f"  Document ID: {did}")

# 3. Upload binary STL — omit formatName to let Onshape auto-detect
print(f"Uploading to Onshape...")
with open(BINARY_STL, "rb") as f:
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

result = r.json()
tid = result.get("id")
print(f"  Translation ID: {tid}")

# 4. Poll until translation completes
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

# 5. Verify parts exist
time.sleep(2)
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
