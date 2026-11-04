"""Milestone 2 & 3 Six-DOF Aircraft Trim Solver & Feasibility Margin Evaluator."""

from typing import Any, Dict, Optional
import numpy as np
from scipy.optimize import least_squares
from .airfoil_bemt import AirfoilModel
from .aircraft_aero import evaluate_aircraft_loads, scheduled_rpm


def solve_aircraft_trim(
    v_tas_ms: float,
    theta_nac_deg: float,
    aircraft_cfg: Dict[str, Any],
    airfoil: AirfoilModel,
    rpm: Optional[float] = None,
    altitude_m: float = 0.0,
    delta_t_K: float = 0.0,
    gamma_deg: float = 0.0,
    gross_mass_kg: Optional[float] = None,
    dv_dt_ms2: float = 0.0,
    u_init: Optional[np.ndarray] = None,
    verify_high_res: bool = True,
) -> Dict[str, Any]:
    """Solve full 6-DOF symmetric aircraft equilibrium with smooth FCC control allocation."""
    if rpm is None:
        rpm = scheduled_rpm(theta_nac_deg, aircraft_cfg)
    if gross_mass_kg is None:
        gross_mass_kg = float(aircraft_cfg["mass_breakdown_kg"]["mtow_kg"])

    lim = aircraft_cfg["controls_and_limits"]
    th0_lo, th0_hi = map(float, lim["theta0_deg_bounds"])
    th1s_lo, th1s_hi = map(float, lim["theta1s_lon_deg_bounds"])
    pitch_lo, pitch_hi = map(float, lim["pitch_deg_bounds"])
    de_lo, de_hi = map(float, lim["elevator_deg_bounds"])

    if v_tas_ms < 6.0:
        de_lo_eff, de_hi_eff = -4.0, 4.0
        pitch_lo_eff, pitch_hi_eff = -10.0, 10.0
    else:
        de_lo_eff, de_hi_eff = de_lo, de_hi
        pitch_lo_eff, pitch_hi_eff = pitch_lo, pitch_hi

    lo = np.array([th0_lo, th1s_lo, pitch_lo_eff, de_lo_eff], dtype=float)
    hi = np.array([th0_hi, th1s_hi, pitch_hi_eff, de_hi_eff], dtype=float)

    tn_rad = float(np.radians(theta_nac_deg))
    # Smooth FCC cyclic washout weight: low penalty in helo mode, strong washout in airplane mode
    w_cyc = 0.008 + 0.12 * (float(np.cos(tn_rad)) ** 2)
    w_elev = 0.12 * float(np.exp(-v_tas_ms / 14.0)) + 0.008
    w_pitch = 0.0 if v_tas_ms < 10.0 else 0.003

    def _full_u(x4: np.ndarray) -> np.ndarray:
        return np.array([x4[0], x4[1], 0.0, x4[2], x4[3], 0.0], dtype=float)

    def _residual_vec(x4: np.ndarray, nr: int, npsi: int) -> np.ndarray:
        ev = evaluate_aircraft_loads(
            _full_u(x4),
            v_tas_ms=v_tas_ms,
            theta_nac_deg=theta_nac_deg,
            aircraft_cfg=aircraft_cfg,
            airfoil=airfoil,
            rpm=rpm,
            altitude_m=altitude_m,
            delta_t_K=delta_t_K,
            gamma_deg=gamma_deg,
            gross_mass_kg=gross_mass_kg,
            dv_dt_ms2=dv_dt_ms2,
            num_radial=nr,
            num_azimuth=npsi,
        )
        F = ev["F_net"]
        M = ev["M_net"]
        if not (np.all(np.isfinite(F)) and np.all(np.isfinite(M))):
            return np.full(6, 1e4)
        return np.array(
            [
                F[0] / 25.0,
                F[2] / 25.0,
                M[1] / 35.0,
                w_cyc * x4[1],
                w_elev * x4[3],
                w_pitch * (x4[2] - float(gamma_deg)),
            ],
            dtype=float,
        )

    seeds = []
    if u_init is not None:
        u_arr = np.asarray(u_init, dtype=float)
        seeds.append(np.array([u_arr[0], u_arr[1], u_arr[3], u_arr[4]], dtype=float))

    th0_est = 10.0 + 26.0 * (v_tas_ms / 115.0) * float(np.cos(tn_rad))
    seeds.append(np.array([np.clip(th0_est, th0_lo + 1.0, th0_hi - 1.0), 0.0, float(gamma_deg) + 1.5, -1.5]))
    if theta_nac_deg < 45.0:
        seeds.append(np.array([32.0, 0.0, float(gamma_deg) + 2.5, -3.0]))
    else:
        seeds.append(np.array([11.0, -1.0, float(gamma_deg), 0.0]))

    best_sol = None
    for s in seeds:
        s_clip = np.clip(s, lo + 1e-4, hi - 1e-4)
        sol = least_squares(
            lambda x: _residual_vec(x, 14, 24),
            s_clip,
            bounds=(lo, hi),
            diff_step=2e-3,
            xtol=1e-6,
            ftol=1e-6,
            max_nfev=80,
        )
        if best_sol is None or sol.cost < best_sol.cost:
            best_sol = sol
        if best_sol.cost < 1e-2:
            break

    x_opt = best_sol.x
    if verify_high_res:
        sol_hi = least_squares(
            lambda x: _residual_vec(x, 24, 48),
            np.clip(x_opt, lo + 1e-4, hi - 1e-4),
            bounds=(lo, hi),
            diff_step=2e-3,
            xtol=1e-6,
            ftol=1e-6,
            max_nfev=35,
        )
        if sol_hi.cost <= best_sol.cost * 1.5 + 1e-2:
            x_opt = sol_hi.x

    u_trim = _full_u(x_opt)
    ev = evaluate_aircraft_loads(
        u_trim,
        v_tas_ms=v_tas_ms,
        theta_nac_deg=theta_nac_deg,
        aircraft_cfg=aircraft_cfg,
        airfoil=airfoil,
        rpm=rpm,
        altitude_m=altitude_m,
        delta_t_K=delta_t_K,
        gamma_deg=gamma_deg,
        gross_mass_kg=gross_mass_kg,
        dv_dt_ms2=dv_dt_ms2,
        num_radial=24 if verify_high_res else 14,
        num_azimuth=48 if verify_high_res else 24,
    )

    f_res_max = float(np.max(np.abs(ev["F_net"])))
    m_res_max = float(np.max(np.abs(ev["M_net"])))
    converged = (f_res_max <= float(lim["force_residual_tol_N"])) and (
        m_res_max <= float(lim["moment_residual_tol_Nm"])
    )

    failures = []
    if not converged:
        failures.append("Excessive Trim Residual")
    if ev["power_margin_kW"] < 0.0:
        failures.append("Power Limit")
    if ev["tip_mach_margin"] < 0.0:
        failures.append("Tip-Mach Limit")
    if theta_nac_deg < 80.0 and v_tas_ms > 18.0 and ev["wing_stall_margin_deg"] < float(lim["wing_stall_margin_min_deg"]):
        failures.append("Wing Stall")
    if ev["rotor_stall_margin_deg"] < float(lim["rotor_stall_margin_min_deg"]):
        failures.append("Rotor Stall")
    if ev["control_margin_pct"] < 2.0:
        failures.append("Control Saturation")

    feasible = len(failures) == 0
    return {
        "feasible": feasible,
        "converged": converged,
        "status": "FEASIBLE" if feasible else "; ".join(failures),
        "failure_codes": failures,
        "u_trim_deg": u_trim,
        "theta0_deg": float(u_trim[0]),
        "theta1s_deg": float(u_trim[1]),
        "theta1c_deg": float(u_trim[2]),
        "pitch_deg": float(u_trim[3]),
        "elevator_deg": float(u_trim[4]),
        "roll_deg": float(u_trim[5]),
        "rpm": float(rpm),
        "v_tas_ms": float(v_tas_ms),
        "theta_nac_deg": float(theta_nac_deg),
        "altitude_m": float(altitude_m),
        "gamma_deg": float(gamma_deg),
        "gross_mass_kg": float(gross_mass_kg),
        "force_res_max_N": f_res_max,
        "moment_res_max_Nm": m_res_max,
        "eval": ev,
    }
