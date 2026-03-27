from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

from .spec import DesignParameters, SurfaceStation


@dataclass(frozen=True)
class DerivedGeometry:
    wing_area_m2: float
    wing_aspect_ratio: float
    wing_mac_m: float
    wing_mac_y_m: float
    wing_mac_le_x_m: float
    wing_quarter_chord_sweep_deg: float
    wing_projected_span_m: float
    canard_area_m2: float
    canard_area_ratio: float
    canard_quarter_chord_x_m: float
    canard_tip_y_m: float
    canard_tip_trailing_edge_x_m: float
    main_wing_x_at_canard_tip_m: float
    main_wing_z_at_canard_tip_m: float
    canard_to_prop_radial_clearance_m: float
    canard_to_prop_x_clearance_m: float
    elevon_span_in_propwash_fraction: float
    wake_clearance_m: float
    required_cruise_cl: float


def _surface_area(stations: list[SurfaceStation]) -> float:
    return 2.0 * sum(
        0.5 * (left.chord_m + right.chord_m) * (right.y_m - left.y_m)
        for left, right in zip(stations[:-1], stations[1:])
    )


def _surface_mac(stations: list[SurfaceStation]) -> tuple[float, float, float]:
    area_half = 0.0
    mac_weighted = 0.0
    mac_y_weighted = 0.0
    mac_x_weighted = 0.0
    for left, right in zip(stations[:-1], stations[1:]):
        dy = right.y_m - left.y_m
        c0 = left.chord_m
        c1 = right.chord_m
        area = 0.5 * (c0 + c1) * dy
        if area <= 0:
            continue
        taper = c1 / c0
        mac = (2.0 / 3.0) * c0 * ((1.0 + taper + taper * taper) / (1.0 + taper))
        eta = (1.0 + 2.0 * taper) / (3.0 * (1.0 + taper))
        y = left.y_m + eta * dy
        x = left.leading_edge_x_m + eta * (right.leading_edge_x_m - left.leading_edge_x_m)
        area_half += area
        mac_weighted += mac * area
        mac_y_weighted += y * area
        mac_x_weighted += x * area
    return (
        mac_weighted / area_half,
        mac_y_weighted / area_half,
        mac_x_weighted / area_half,
    )


def _quarter_chord_sweep(stations: list[SurfaceStation]) -> float:
    root = stations[0]
    tip = stations[-1]
    dx = tip.quarter_chord_x_m - root.quarter_chord_x_m
    dy = tip.y_m - root.y_m
    return math.degrees(math.atan2(dx, dy))


def _interp_surface(stations: list[SurfaceStation], y_m: float) -> tuple[float, float, float]:
    if y_m <= stations[0].y_m:
        station = stations[0]
        return station.leading_edge_x_m, station.z_m, station.chord_m
    for left, right in zip(stations[:-1], stations[1:]):
        if left.y_m <= y_m <= right.y_m:
            t = (y_m - left.y_m) / (right.y_m - left.y_m)
            x = left.leading_edge_x_m + t * (right.leading_edge_x_m - left.leading_edge_x_m)
            z = left.z_m + t * (right.z_m - left.z_m)
            chord = left.chord_m + t * (right.chord_m - left.chord_m)
            return x, z, chord
    station = stations[-1]
    return station.leading_edge_x_m, station.z_m, station.chord_m


def _wake_clearance(parameters: DesignParameters, wing_x: float, wing_z: float) -> float:
    canard_tip = parameters.canard.stations[-1]
    effective_angle_deg = (
        parameters.analysis_conditions.transition_alpha_deg
        + parameters.canard.deflection_limits_deg.transition_abs
        + 4.0
    )
    delta_x = wing_x - canard_tip.trailing_edge_x_m
    wake_z = canard_tip.z_m - math.tan(math.radians(effective_angle_deg)) * max(delta_x, 0.0)
    return wake_z - wing_z


def derive_geometry(parameters: DesignParameters) -> DerivedGeometry:
    wing_area = _surface_area(parameters.wing.stations)
    wing_mac, wing_mac_y, wing_mac_le_x = _surface_mac(parameters.wing.stations)
    wing_span = 2.0 * parameters.wing.stations[-1].y_m
    wing_ar = wing_span * wing_span / wing_area
    canard_area = _surface_area(parameters.canard.stations)
    canard_tip = parameters.canard.stations[-1]
    prop_radius = parameters.propulsion.prop_radius_m
    prop_inboard_edge = parameters.propulsion.motor_y_m - prop_radius
    main_wing_x, main_wing_z, _ = _interp_surface(parameters.wing.stations, canard_tip.y_m)
    elevon = parameters.control_surfaces["elevon_left"]
    elevon_span = elevon.end_y_m - elevon.start_y_m
    overlap_start = max(elevon.start_y_m, prop_inboard_edge)
    overlap_end = min(elevon.end_y_m, parameters.propulsion.motor_y_m + prop_radius)
    overlap = max(0.0, overlap_end - overlap_start)
    q = 0.5 * parameters.analysis_conditions.air_density_kg_m3 * parameters.analysis_conditions.cruise_speed_m_s**2
    weight = parameters.mass_properties.analysis_mass_kg * 9.80665
    required_cl = weight / (q * wing_area)
    return DerivedGeometry(
        wing_area_m2=wing_area,
        wing_aspect_ratio=wing_ar,
        wing_mac_m=wing_mac,
        wing_mac_y_m=wing_mac_y,
        wing_mac_le_x_m=wing_mac_le_x,
        wing_quarter_chord_sweep_deg=_quarter_chord_sweep(parameters.wing.stations),
        wing_projected_span_m=wing_span,
        canard_area_m2=canard_area,
        canard_area_ratio=canard_area / wing_area,
        canard_quarter_chord_x_m=parameters.canard.stations[0].quarter_chord_x_m,
        canard_tip_y_m=canard_tip.y_m,
        canard_tip_trailing_edge_x_m=canard_tip.trailing_edge_x_m,
        main_wing_x_at_canard_tip_m=main_wing_x,
        main_wing_z_at_canard_tip_m=main_wing_z,
        canard_to_prop_radial_clearance_m=prop_inboard_edge - canard_tip.y_m,
        canard_to_prop_x_clearance_m=parameters.propulsion.motor_x_m - canard_tip.trailing_edge_x_m,
        elevon_span_in_propwash_fraction=overlap / elevon_span,
        wake_clearance_m=_wake_clearance(parameters, main_wing_x, main_wing_z),
        required_cruise_cl=required_cl,
    )


def derived_to_dict(derived: DerivedGeometry) -> dict[str, float]:
    return asdict(derived)


def write_derived_geometry(path: Path, derived: DerivedGeometry) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(derived_to_dict(derived), indent=2), encoding="utf-8")
