"""Milestone 1 & 3 Propulsion, Transmission Limit, and Fuel-Flow Model."""

from dataclasses import dataclass
from typing import Any, Dict
import numpy as np
from .atmosphere import AtmosphereState, isa_atmosphere


@dataclass(frozen=True)
class PropulsionEvaluation:
    p_rotor_aero_kW: float
    p_shaft_req_kW: float
    p_eng_thermo_avail_kW: float
    p_trans_limit_kW: float
    p_avail_effective_kW: float
    power_margin_kW: float
    load_fraction: float
    sfc_kg_per_kWh: float
    fuel_flow_kg_s: float
    fuel_flow_kg_h: float
    governing_power_cap: str


def evaluate_propulsion(
    p_rotor_pair_aero_kW: float,
    aircraft_cfg: Dict[str, Any],
    atm: AtmosphereState | None = None,
    altitude_m: float = 0.0,
    delta_t_K: float = 0.0,
) -> PropulsionEvaluation:
    """Compute shaft power demand, available power (engine lapse vs gearbox), and fuel burn."""
    if atm is None:
        atm = isa_atmosphere(altitude_m, delta_t_K)

    prop = aircraft_cfg["propulsion"]
    eta_gb = float(prop["gearbox_efficiency"])
    p_idle = float(prop["accessory_idle_power_kW"])
    p_sl = float(prop["installed_power_sl_total_kW"])
    p_trans = float(prop["transmission_shaft_limit_kW"])
    lapse_exp = float(prop["altitude_density_lapse_exponent"])

    rho_ratio = atm.rho_kgm3 / 1.225
    temp_ratio = atm.temperature_K / 288.15
    p_eng_thermo = p_sl * (rho_ratio ** lapse_exp) * float(np.sqrt(temp_ratio))

    if p_eng_thermo <= p_trans:
        p_avail = p_eng_thermo
        cap_source = "Engine ISA Lapse"
    else:
        p_avail = p_trans
        cap_source = "Transmission Limit"

    p_shaft_req = max(p_idle, float(p_rotor_pair_aero_kW) / eta_gb + 0.35 * p_idle)
    load_frac = float(np.clip(p_shaft_req / max(p_eng_thermo, 1.0), float(prop["sfc_min_load_fraction"]), 1.15))

    sfc_ref = float(prop["sfc_design_kg_per_kWh"])
    a_sfc = float(prop["sfc_part_load_a"])
    b_sfc = float(prop["sfc_part_load_b"])
    sfc_eff = sfc_ref * (a_sfc + b_sfc / load_frac)

    mdot_kgh = sfc_eff * p_shaft_req
    mdot_kgs = mdot_kgh / 3600.0

    return PropulsionEvaluation(
        p_rotor_aero_kW=float(p_rotor_pair_aero_kW),
        p_shaft_req_kW=float(p_shaft_req),
        p_eng_thermo_avail_kW=float(p_eng_thermo),
        p_trans_limit_kW=float(p_trans),
        p_avail_effective_kW=float(p_avail),
        power_margin_kW=float(p_avail - p_shaft_req),
        load_fraction=load_frac,
        sfc_kg_per_kWh=float(sfc_eff),
        fuel_flow_kg_s=float(mdot_kgs),
        fuel_flow_kg_h=float(mdot_kgh),
        governing_power_cap=cap_source,
    )
