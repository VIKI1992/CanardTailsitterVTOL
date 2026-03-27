from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Metadata:
    name: str
    version: str
    units: str


@dataclass(frozen=True)
class SurfaceStation:
    name: str
    leading_edge_x_m: float
    y_m: float
    z_m: float
    chord_m: float
    twist_deg: float

    @property
    def quarter_chord_x_m(self) -> float:
        return self.leading_edge_x_m + 0.25 * self.chord_m

    @property
    def trailing_edge_x_m(self) -> float:
        return self.leading_edge_x_m + self.chord_m


@dataclass(frozen=True)
class WingDefinition:
    name: str
    airfoil: str
    stations: list[SurfaceStation]


@dataclass(frozen=True)
class CanardLimits:
    trim_target_abs: float
    forward_command_abs: float
    transition_abs: float
    hard_stop_abs: float


@dataclass(frozen=True)
class CanardDefinition:
    name: str
    airfoil: str
    pivot_fraction: float
    baseline_incidence_deg: float
    stations: list[SurfaceStation]
    deflection_limits_deg: CanardLimits


@dataclass(frozen=True)
class ControlSurfaceDefinition:
    surface_name: str
    start_y_m: float
    end_y_m: float
    chord_fraction: float
    hinge_fraction_from_le: float
    cruise_limit_deg: float
    hover_limit_deg: float
    hard_stop_deg: float


@dataclass(frozen=True)
class FuselageStation:
    name: str
    x_m: float
    width_m: float
    height_m: float
    shape: str
    corner_radius_m: float | None = None


@dataclass(frozen=True)
class FuselageDefinition:
    name: str
    stations: list[FuselageStation]
    tessellation_w: int

    @property
    def length_m(self) -> float:
        return self.stations[-1].x_m - self.stations[0].x_m


@dataclass(frozen=True)
class PropulsionDefinition:
    motor_count: int
    prop_diameter_m: float
    motor_x_m: float
    motor_y_m: float
    motor_z_m: float
    left_rotation: str
    right_rotation: str

    @property
    def prop_radius_m(self) -> float:
        return 0.5 * self.prop_diameter_m


@dataclass(frozen=True)
class MassProperties:
    analysis_mass_kg: float
    cg_x_m: float


@dataclass(frozen=True)
class AnalysisConditions:
    cruise_speed_m_s: float
    transition_speed_m_s: float
    air_density_kg_m3: float
    cruise_alpha_deg: float
    transition_alpha_deg: float


@dataclass(frozen=True)
class PowerEffects:
    elevon_dynamic_pressure_gain_powered: float
    canard_dynamic_pressure_gain_powered: float


@dataclass(frozen=True)
class DesignParameters:
    metadata: Metadata
    wing: WingDefinition
    canard: CanardDefinition
    control_surfaces: dict[str, ControlSurfaceDefinition]
    fuselage: FuselageDefinition
    propulsion: PropulsionDefinition
    mass_properties: MassProperties
    analysis_conditions: AnalysisConditions
    power_effects: PowerEffects


def _station(data: dict[str, Any]) -> SurfaceStation:
    return SurfaceStation(
        name=data["name"],
        leading_edge_x_m=float(data["leading_edge_x_m"]),
        y_m=float(data["y_m"]),
        z_m=float(data["z_m"]),
        chord_m=float(data["chord_m"]),
        twist_deg=float(data["twist_deg"]),
    )


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_parameters(path: Path) -> DesignParameters:
    data = _load_json(path)
    return DesignParameters(
        metadata=Metadata(**data["metadata"]),
        wing=WingDefinition(
            name=data["wing"]["name"],
            airfoil=data["wing"]["airfoil"],
            stations=[_station(item) for item in data["wing"]["stations"]],
        ),
        canard=CanardDefinition(
            name=data["canard"]["name"],
            airfoil=data["canard"]["airfoil"],
            pivot_fraction=float(data["canard"]["pivot_fraction"]),
            baseline_incidence_deg=float(data["canard"]["baseline_incidence_deg"]),
            stations=[_station(item) for item in data["canard"]["stations"]],
            deflection_limits_deg=CanardLimits(
                trim_target_abs=float(data["canard"]["deflection_limits_deg"]["trim_target_abs"]),
                forward_command_abs=float(data["canard"]["deflection_limits_deg"]["forward_command_abs"]),
                transition_abs=float(data["canard"]["deflection_limits_deg"]["transition_abs"]),
                hard_stop_abs=float(data["canard"]["deflection_limits_deg"]["hard_stop_abs"]),
            ),
        ),
        control_surfaces={
            name: ControlSurfaceDefinition(
                surface_name=item["surface_name"],
                start_y_m=float(item["start_y_m"]),
                end_y_m=float(item["end_y_m"]),
                chord_fraction=float(item["chord_fraction"]),
                hinge_fraction_from_le=float(item["hinge_fraction_from_le"]),
                cruise_limit_deg=float(item["cruise_limit_deg"]),
                hover_limit_deg=float(item["hover_limit_deg"]),
                hard_stop_deg=float(item["hard_stop_deg"]),
            )
            for name, item in data["control_surfaces"].items()
        },
        fuselage=FuselageDefinition(
            name=data["fuselage"]["name"],
            stations=[
                FuselageStation(
                    name=item["name"],
                    x_m=float(item["x_m"]),
                    width_m=float(item["width_m"]),
                    height_m=float(item["height_m"]),
                    shape=item["shape"],
                    corner_radius_m=(
                        float(item["corner_radius_m"]) if "corner_radius_m" in item else None
                    ),
                )
                for item in data["fuselage"]["stations"]
            ],
            tessellation_w=int(data["fuselage"]["tessellation_w"]),
        ),
        propulsion=PropulsionDefinition(**data["propulsion"]),
        mass_properties=MassProperties(**data["mass_properties"]),
        analysis_conditions=AnalysisConditions(**data["analysis_conditions"]),
        power_effects=PowerEffects(**data["power_effects"]),
    )


def load_validation_targets(path: Path) -> dict[str, Any]:
    return _load_json(path)


def load_control_model(path: Path) -> dict[str, Any]:
    return _load_json(path)

