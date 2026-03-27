"""V3 canard tailsitter model package."""

from .spec import DesignParameters, load_parameters, load_validation_targets, load_control_model
from .geometry import DerivedGeometry, derive_geometry, write_derived_geometry
from .analysis import evaluate_validation, write_validation_outputs, build_airplane, run_aero
from .openvsp_builder import build_vspscript, export_vsp3, find_script_runner
