"""Milestone 1 Module: International Standard Atmosphere (ISA) & Wind Kinematics."""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class AtmosphereState:
    altitude_m: float
    delta_t_K: float
    rho_kgm3: float
    a_sound_ms: float
    temperature_K: float
    pressure_Pa: float
    mu_Pas: float


def isa_atmosphere(altitude_m: float, delta_t_K: float = 0.0) -> AtmosphereState:
    """Evaluate tropospheric ISA state with non-standard temperature offset."""
    alt = float(np.clip(altitude_m, 0.0, 11000.0))
    t_std = 288.15 - 0.0065 * alt
    p_std = 101325.0 * ((t_std / 288.15) ** 5.25588)
    temp = max(t_std + float(delta_t_K), 180.0)
    rho = p_std / (287.058 * temp)
    a_sound = float(np.sqrt(1.4 * 287.058 * temp))
    mu = float(1.458e-6 * (temp ** 1.5) / (temp + 110.4))
    return AtmosphereState(
        altitude_m=alt,
        delta_t_K=float(delta_t_K),
        rho_kgm3=float(rho),
        a_sound_ms=a_sound,
        temperature_K=float(temp),
        pressure_Pa=float(p_std),
        mu_Pas=mu,
    )


def ground_speed_from_tas(tas_ms: float, headwind_ms: float = 0.0, gamma_deg: float = 0.0) -> float:
    """Horizontal ground speed accounting for flight-path angle and headwind."""
    horiz_tas = float(tas_ms) * float(np.cos(np.radians(gamma_deg)))
    if tas_ms < 0.5:
        return 0.0
    return float(max(0.0, horiz_tas - float(headwind_ms)))
