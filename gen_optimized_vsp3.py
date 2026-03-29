"""
Generate the optimized design .vsp3 file from the best optimized params JSON.
"""
import json
import sys
from pathlib import Path

REPO = Path(__file__).parent
sys.path.insert(0, str(REPO))

from v3.model.spec import load_parameters
from v3.model.openvsp_builder import build_vspscript
from v3.optimization.design_space import encode, PARAM_NAMES, _LOWERS, _UPPERS
import numpy as np

# --- Load baseline params ---
base_params = load_parameters(REPO / "spec" / "parameters_v3.json")

# --- Load optimized raw values ---
result_path = REPO / "results" / "optimized_params_20260329_135310.json"
with open(result_path) as f:
    data = json.load(f)

raw_dict = data["best_candidate"]["design_vector_raw"]
raw = np.array([raw_dict[name] for name in PARAM_NAMES], dtype=np.float64)
x_norm = encode(np.clip(raw, _LOWERS, _UPPERS))

# --- Reconstruct DesignParameters ---
from v3.optimization.design_space import to_design_parameters
opt_params = to_design_parameters(x_norm, base_params)

# --- Generate VSPScript ---
out_vsp3 = "v3/results/fixed_wing_tailsitter_optimized.vsp3"
script_text = build_vspscript(opt_params, out_vsp3)

script_path = REPO / "v3" / "results" / "fixed_wing_tailsitter_optimized.vspscript"
script_path.write_text(script_text)
print(f"VSPScript written to: {script_path}")
print(f"Target .vsp3: {REPO / out_vsp3}")
