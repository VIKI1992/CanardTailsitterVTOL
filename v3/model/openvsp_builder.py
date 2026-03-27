from __future__ import annotations

import math
from pathlib import Path
import subprocess

from .spec import DesignParameters, FuselageStation, SurfaceStation


def _fmt(value: float) -> str:
    return f"{value:.6f}"


def _section_geometry(left: SurfaceStation, right: SurfaceStation) -> tuple[float, float, float]:
    dy = right.y_m - left.y_m
    dz = right.z_m - left.z_m
    dx = right.leading_edge_x_m - left.leading_edge_x_m
    span = math.hypot(dy, dz)
    dihedral_deg = math.degrees(math.atan2(dz, max(dy, 1e-9)))
    sweep_deg = math.degrees(math.atan2(dx, max(dy, 1e-9)))
    return span, sweep_deg, dihedral_deg


def _naca4(identifier: str) -> tuple[float, float, float] | None:
    clean = identifier.strip().upper().replace("NACA", "")
    if len(clean) != 4 or not clean.isdigit():
        return None
    camber = int(clean[0]) / 100.0
    camber_loc = int(clean[1]) / 10.0
    thickness = int(clean[2:]) / 100.0
    return camber, camber_loc, thickness


def _set_airfoil_lines(geom_id: str, xsec_surf: str, index: int, identifier: str) -> list[str]:
    lines = [f'    string xsec_{geom_id}_{index} = GetXSec( {xsec_surf}, {index} );']
    naca = _naca4(identifier)
    if naca is None:
        lines.append(f"    ChangeXSecShape( {xsec_surf}, {index}, XS_FILE_AIRFOIL );")
        lines.append("    Update();")
        lines.append(f'    xsec_{geom_id}_{index} = GetXSec( {xsec_surf}, {index} );')
        lines.append(f'    ReadFileAirfoil( xsec_{geom_id}_{index}, "airfoils/sd7037.dat" );')
        return lines
    camber, camber_loc, thickness = naca
    lines.append(f"    ChangeXSecShape( {xsec_surf}, {index}, XS_FOUR_SERIES );")
    lines.append("    Update();")
    lines.append(f'    SetParmVal( {geom_id}, "Camber", "XSecCurve_{index}", {_fmt(camber)} );')
    lines.append(f'    SetParmVal( {geom_id}, "CamberLoc", "XSecCurve_{index}", {_fmt(camber_loc)} );')
    lines.append(f'    SetParmVal( {geom_id}, "ThickChord", "XSecCurve_{index}", {_fmt(thickness)} );')
    return lines


def _wing_lines(parameters: DesignParameters, geom_name: str, geom_id: str, stations: list[SurfaceStation], airfoils: list[str]) -> list[str]:
    root = stations[0]
    lines = [
        f'    string {geom_id} = AddGeom( "WING", "" );',
        f'    SetGeomName( {geom_id}, "{geom_name}" );',
        f'    SetParmVal( {geom_id}, "X_Rel_Location", "XForm", {_fmt(root.leading_edge_x_m)} );',
        f'    SetParmVal( {geom_id}, "Y_Rel_Location", "XForm", {_fmt(root.y_m)} );',
        f'    SetParmVal( {geom_id}, "Z_Rel_Location", "XForm", {_fmt(root.z_m)} );',
        f'    SetParmVal( {geom_id}, "Y_Rel_Rotation", "XForm", {_fmt(root.twist_deg)} );',
        f'    SetParmVal( {geom_id}, "RotateAirfoilMatchDideralFlag", "WingGeom", 1.0 );',
        f'    SetParmVal( {geom_id}, "Sym_Planar_Flag", "Sym", 2 );',
    ]
    # Default wing has 2 XSecs (1 section). We need len(stations) XSecs
    # (len(stations)-1 sections), so insert len(stations)-2 more.
    for i in range(len(stations) - 2):
        lines.append(f"    InsertXSec( {geom_id}, {i + 1}, XS_FOUR_SERIES );")
    lines.append("    Update();")
    lines.append(f"    string {geom_id}_xsec_surf = GetXSecSurf( {geom_id}, 0 );")
    for index, airfoil in enumerate(airfoils):
        lines.extend(_set_airfoil_lines(geom_id, f"{geom_id}_xsec_surf", index, airfoil))
    for index, (left, right) in enumerate(zip(stations[:-1], stations[1:]), start=1):
        span, sweep, dihedral = _section_geometry(left, right)
        lines.extend(
            [
                f"    SetDriverGroup( {geom_id}, {index}, SPAN_WSECT_DRIVER, ROOTC_WSECT_DRIVER, TIPC_WSECT_DRIVER );",
                f'    SetParmVal( {geom_id}, "Span", "XSec_{index}", {_fmt(span)} );',
                f'    SetParmVal( {geom_id}, "Root_Chord", "XSec_{index}", {_fmt(left.chord_m)} );',
                f'    SetParmVal( {geom_id}, "Tip_Chord", "XSec_{index}", {_fmt(right.chord_m)} );',
                f'    SetParmVal( {geom_id}, "Sweep", "XSec_{index}", {_fmt(sweep)} );',
                f'    SetParmVal( {geom_id}, "Sweep_Location", "XSec_{index}", 0.0 );',
                f'    SetParmVal( {geom_id}, "Dihedral", "XSec_{index}", {_fmt(dihedral)} );',
                f'    SetParmVal( {geom_id}, "Twist", "XSec_{index}", {_fmt(right.twist_deg - root.twist_deg)} );',
                f'    SetParmVal( {geom_id}, "SectTess_U", "XSec_{index}", 12 );',
            ]
        )
    lines.append("    Update();")
    return lines


def _y_to_eta(y_m: float, stations: list[SurfaceStation]) -> float:
    """Convert a y-position to OpenVSP eta (equal U-range per section)."""
    n_sections = len(stations) - 1
    for i, (a, b) in enumerate(zip(stations[:-1], stations[1:])):
        if y_m <= b.y_m:
            t = (y_m - a.y_m) / max(b.y_m - a.y_m, 1e-9)
            return (i + t) / n_sections
    return 1.0


def _control_surface_lines(parameters: DesignParameters) -> list[str]:
    elevon = parameters.control_surfaces["elevon_left"]
    eta_start = _y_to_eta(elevon.start_y_m, parameters.wing.stations)
    eta_end = _y_to_eta(elevon.end_y_m, parameters.wing.stations)
    u_start = elevon.hinge_fraction_from_le
    lines = [
        "    string elevon_ss = AddSubSurf( wing_id, SS_CONTROL, 0 );",
        "    Update();",
        '    SetParmVal( wing_id, "EtaStart", "SS_Control_1", ' + _fmt(eta_start) + " );",
        '    SetParmVal( wing_id, "EtaEnd", "SS_Control_1", ' + _fmt(eta_end) + " );",
        '    SetParmVal( wing_id, "UStart", "SS_Control_1", ' + _fmt(u_start) + " );",
        '    SetParmVal( wing_id, "UEnd", "SS_Control_1", 1.0 );',
        '    SetParmVal( wing_id, "SE_Const_Flag", "SS_Control_1", 1.0 );',
        "    Update();",
        "    int elevon_sym_group = CreateVSPAEROControlSurfaceGroup();",
        '    SetVSPAEROControlGroupName( "ElevonSymmetric", elevon_sym_group );',
        "    array<int> elevon_indices;",
        "    elevon_indices.push_back( 1 );",
        "    elevon_indices.push_back( 2 );",
        "    AddSelectedToCSGroup( elevon_indices, elevon_sym_group );",
        "    string cs_group_container_id = FindContainer( \"VSPAEROSettings\", 0 );",
        '    SetParmVal( FindParm( cs_group_container_id, "Surf_" + elevon_ss + "_0_Gain", "ControlSurfaceGroup_0" ), 1 );',
        '    SetParmVal( FindParm( cs_group_container_id, "Surf_" + elevon_ss + "_1_Gain", "ControlSurfaceGroup_0" ), 1 );',
        "    int elevon_diff_group = CreateVSPAEROControlSurfaceGroup();",
        '    SetVSPAEROControlGroupName( "ElevonDifferential", elevon_diff_group );',
        "    AddSelectedToCSGroup( elevon_indices, elevon_diff_group );",
        '    SetParmVal( FindParm( cs_group_container_id, "Surf_" + elevon_ss + "_0_Gain", "ControlSurfaceGroup_1" ), 1 );',
        '    SetParmVal( FindParm( cs_group_container_id, "Surf_" + elevon_ss + "_1_Gain", "ControlSurfaceGroup_1" ), -1 );',
        "    Update();",
    ]
    return lines


def _fuselage_shape_lines(fuselage_id: str, stations: list[FuselageStation]) -> list[str]:
    base_x = stations[0].x_m
    length = stations[-1].x_m - base_x
    lines = [
        f'    string {fuselage_id} = AddGeom( "FUSELAGE", "" );',
        f'    SetGeomName( {fuselage_id}, "Fuselage" );',
        f'    SetParmVal( {fuselage_id}, "X_Rel_Location", "XForm", {_fmt(base_x)} );',
        f'    SetParmVal( {fuselage_id}, "Length", "Design", {_fmt(length)} );',
        f'    SetParmVal( {fuselage_id}, "Tess_W", "Shape", {max(25, len(stations) * 6 + 1)} );',
    ]
    for _ in range(len(stations) - 5):
        lines.append(f"    InsertXSec( {fuselage_id}, 1, XS_ELLIPSE );")
    lines.append("    Update();")
    lines.append(f"    string {fuselage_id}_xsec_surf = GetXSecSurf( {fuselage_id}, 0 );")
    for index, station in enumerate(stations):
        shape = station.shape.lower()
        if shape == "point":
            lines.append(f"    ChangeXSecShape( {fuselage_id}_xsec_surf, {index}, XS_POINT );")
        elif shape == "ellipse":
            lines.append(f"    ChangeXSecShape( {fuselage_id}_xsec_surf, {index}, XS_ELLIPSE );")
        elif shape == "rounded_rectangle":
            lines.append(f"    ChangeXSecShape( {fuselage_id}_xsec_surf, {index}, XS_ROUNDED_RECTANGLE );")
        else:
            raise ValueError(f"Unsupported fuselage shape: {station.shape}")
    lines.append("    Update();")
    for index, station in enumerate(stations):
        xsec_ref = f"xsec_{fuselage_id}_{index}"
        lines.append(f"    string {xsec_ref} = GetXSec( {fuselage_id}_xsec_surf, {index} );")
        if index > 0:
            lines.append(
                f'    SetParmVal( {fuselage_id}, "XLocPercent", "XSec_{index}", {_fmt((station.x_m - base_x) / length)} );'
            )
            lines.append(f'    SetParmVal( {fuselage_id}, "ZLocPercent", "XSec_{index}", 0.0 );')
        if station.shape.lower() == "ellipse":
            lines.append(
                f'    SetParmVal( {fuselage_id}, "Ellipse_Width", "XSecCurve_{index}", {_fmt(station.width_m)} );'
            )
            lines.append(
                f'    SetParmVal( {fuselage_id}, "Ellipse_Height", "XSecCurve_{index}", {_fmt(station.height_m)} );'
            )
        elif station.shape.lower() == "rounded_rectangle":
            lines.append(
                f'    SetParmVal( {fuselage_id}, "RoundedRect_Width", "XSecCurve_{index}", {_fmt(station.width_m)} );'
            )
            lines.append(
                f'    SetParmVal( {fuselage_id}, "RoundedRect_Height", "XSecCurve_{index}", {_fmt(station.height_m)} );'
            )
            lines.append(
                f'    SetParmVal( {fuselage_id}, "RoundRectXSec_Radius", "XSecCurve_{index}", {_fmt(station.corner_radius_m or 0.0)} );'
            )
        lines.extend(
            [
                f"    SetXSecContinuity( {xsec_ref}, 1 );",
                f"    SetXSecTanAngles( {xsec_ref}, XSEC_BOTH_SIDES, 0, 0, 0, 0 );",
                f"    SetXSecTanStrengths( {xsec_ref}, XSEC_BOTH_SIDES, 0.5, 0.5, 0.5, 0.5 );",
            ]
        )
    lines.append(f"    SetXSecTanAngles( xsec_{fuselage_id}_0, XSEC_BOTH_SIDES, 90, 90, 90, 90 );")
    lines.append(
        f"    SetXSecTanAngles( xsec_{fuselage_id}_{len(stations)-1}, XSEC_BOTH_SIDES, -90, -90, -90, -90 );"
    )
    lines.append("    Update();")
    return lines


def _prop_lines(parameters: DesignParameters) -> list[str]:
    lines = []
    for side, y_sign, rotation in (
        ("LeftPropDisk", 1.0, parameters.propulsion.left_rotation),
        ("RightPropDisk", -1.0, parameters.propulsion.right_rotation),
    ):
        geom_var = side.lower()
        lines.extend(
            [
                f'    string {geom_var} = AddGeom( "PROP", "" );',
                f'    SetGeomName( {geom_var}, "{side}" );',
                f'    SetParmVal( {geom_var}, "PropMode", "Design", PROP_DISK );',
                f'    SetParmVal( {geom_var}, "Diameter", "Design", {_fmt(parameters.propulsion.prop_diameter_m)} );',
                f'    SetParmVal( {geom_var}, "X_Rel_Location", "XForm", {_fmt(parameters.propulsion.motor_x_m)} );',
                f'    SetParmVal( {geom_var}, "Y_Rel_Location", "XForm", {_fmt(y_sign * parameters.propulsion.motor_y_m)} );',
                f'    SetParmVal( {geom_var}, "Z_Rel_Location", "XForm", {_fmt(parameters.propulsion.motor_z_m)} );',
                f'    SetParmVal( {geom_var}, "ReverseFlag", "Design", {1 if rotation.upper() == "CCW" else 0} );',
            ]
        )
    lines.append("    Update();")
    return lines


def build_vspscript(parameters: DesignParameters, output_vsp3_relative: str) -> str:
    n_wing = len(parameters.wing.stations)
    n_main = max(1, n_wing - 3)
    wing_airfoils = [parameters.wing.airfoil] * n_main + ["NACA0009"] * (n_wing - n_main)
    canard_airfoils = [parameters.canard.airfoil] * len(parameters.canard.stations)
    lines = ["void main()", "{"]
    lines.extend(
        _wing_lines(
            parameters,
            geom_name="MainWing",
            geom_id="wing_id",
            stations=parameters.wing.stations,
            airfoils=wing_airfoils,
        )
    )
    lines.extend(
        _wing_lines(
            parameters,
            geom_name="Canard",
            geom_id="canard_id",
            stations=parameters.canard.stations,
            airfoils=canard_airfoils,
        )
    )
    lines.extend(_control_surface_lines(parameters))
    lines.extend(_fuselage_shape_lines("fuselage_id", parameters.fuselage.stations))
    lines.extend(_prop_lines(parameters))
    lines.append(f'    WriteVSPFile( "{output_vsp3_relative}", SET_ALL );')
    lines.append("}")
    return "\n".join(lines) + "\n"


def find_script_runner(repo_root: Path) -> Path | None:
    patterns = [
        "tools/OpenVSP-*/*/vspscript.exe",
        "tools/OpenVSP-*/*/vsp.exe",
    ]
    for pattern in patterns:
        candidates = sorted(repo_root.glob(pattern), reverse=True)
        for candidate in candidates:
            if candidate.is_file():
                return candidate
    return None


def export_vsp3(repo_root: Path, script_runner: Path, script_path: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(script_runner), "-script", str(script_path)],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )
