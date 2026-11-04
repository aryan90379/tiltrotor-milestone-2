"""Milestone 2 & 3 Whole-Aircraft Aerodynamics, Rotor-Wing Interaction & CG Shift Model."""

from typing import Any, Dict
import numpy as np
from .atmosphere import AtmosphereState, isa_atmosphere
from .airfoil_bemt import AirfoilModel, euler_body_from_inertial, run_edgewise_bemt
from .propulsion import evaluate_propulsion


def scheduled_rpm(theta_nac_deg: float, aircraft_cfg: Dict[str, Any]) -> float:
    """Piecewise-smooth proprotor RPM schedule: 360 rpm in airplane/low-tilt (<=25 deg), 535 rpm in helo (90 deg)."""
    rot = aircraft_cfg["geometry"]["rotor"]
    rpm_cr = float(rot["rpm_cruise"])
    rpm_hov = float(rot["rpm_hover"])
    frac = float(np.clip((float(theta_nac_deg) - 25.0) / 65.0, 0.0, 1.0))
    return float(rpm_cr + (rpm_hov - rpm_cr) * (frac ** 0.90))


def instantaneous_cg_station_m(theta_nac_deg: float, aircraft_cfg: Dict[str, Any]) -> float:
    """Compute longitudinal CG station [m aft of nose] accounting for wingtip nacelle tilt."""
    coords = aircraft_cfg["geometry"]["coordinates_body_m"]
    x_helo = float(coords["x_cg_helo_m"])
    x_air = float(coords["x_cg_airplane_m"])
    s = float(np.clip(np.sin(np.radians(np.clip(theta_nac_deg, 0.0, 90.0))), 0.0, 1.0))
    return float(x_air + (x_helo - x_air) * s)


def _body_rel_cg(x_sta: float, y_sta: float, z_sta: float, x_cg: float) -> np.ndarray:
    """Convert nose-aft station to right-handed body vector (+x fwd, +y right, +z down) from CG."""
    return np.array([x_cg - x_sta, y_sta, z_sta], dtype=float)


def evaluate_aircraft_loads(
    u_controls_deg: np.ndarray,
    v_tas_ms: float,
    theta_nac_deg: float,
    aircraft_cfg: Dict[str, Any],
    airfoil: AirfoilModel,
    rpm: float | None = None,
    altitude_m: float = 0.0,
    delta_t_K: float = 0.0,
    gamma_deg: float = 0.0,
    gross_mass_kg: float | None = None,
    dv_dt_ms2: float = 0.0,
    num_radial: int = 16,
    num_azimuth: int = 36,
) -> Dict[str, Any]:
    """Evaluate 6-DOF rotor + airframe + gravity + inertial loads about instantaneous CG.

    Control vector u_controls_deg = [theta0, theta1s_lon, theta1c_lat, pitch_theta, delta_e, phi_roll].
    """
    th0, th1s, th1c, pitch_deg, elev_deg, roll_deg = map(float, u_controls_deg)
    if rpm is None:
        rpm = scheduled_rpm(theta_nac_deg, aircraft_cfg)
    if gross_mass_kg is None:
        gross_mass_kg = float(aircraft_cfg["mass_breakdown_kg"]["mtow_kg"])

    atm: AtmosphereState = isa_atmosphere(altitude_m, delta_t_K)
    rot_cfg = aircraft_cfg["geometry"]["rotor"]
    wing_cfg = aircraft_cfg["geometry"]["wing"]
    emp_cfg = aircraft_cfg["geometry"]["empennage"]
    coords = aircraft_cfg["geometry"]["coordinates_body_m"]

    x_cg = instantaneous_cg_station_m(theta_nac_deg, aircraft_cfg)
    r_hub_port = _body_rel_cg(float(coords["x_hub_pivot_m"]), -float(coords["y_hub_m"]), float(coords["z_hub_m"]), x_cg)
    r_hub_stbd = _body_rel_cg(float(coords["x_hub_pivot_m"]), +float(coords["y_hub_m"]), float(coords["z_hub_m"]), x_cg)
    r_wing_ac = _body_rel_cg(float(coords["x_ac_wing_m"]), 0.0, float(coords["z_wing_m"]), x_cg)
    r_htail_ac = _body_rel_cg(float(coords["x_ac_htail_m"]), 0.0, float(coords["z_htail_m"]), x_cg)

    alpha_body_deg = pitch_deg - float(gamma_deg)
    a_b = np.radians(alpha_body_deg)
    ca, sa = float(np.cos(a_b)), float(np.sin(a_b))
    tn_rad = float(np.radians(theta_nac_deg))

    common_bemt = dict(
        radius=float(rot_cfg["radius_m"]),
        root_cutout=float(rot_cfg["root_cutout_m"]),
        num_blades=int(rot_cfg["num_blades"]),
        c_root=float(rot_cfg["root_chord_m"]),
        taper=float(rot_cfg["taper_ratio"]),
        collective_deg=th0,
        twist_deg=float(rot_cfg["twist_deg"]),
        rpm=float(rpm),
        v_inf_ms=float(v_tas_ms),
        theta_nac_deg=float(theta_nac_deg),
        airfoil=airfoil,
        cyclic_1c_deg=th1s,
        alpha_body_deg=alpha_body_deg,
        rho=atm.rho_kgm3,
        a_sound=atm.a_sound_ms,
        mu_visc=atm.mu_Pas,
        num_radial=int(num_radial),
        num_azimuth=int(num_azimuth),
        flapping_mode="steady_1st_harmonic",
        spanwise_flow_correction=bool(rot_cfg.get("spanwise_flow_correction", True)),
    )

    left_rot = run_edgewise_bemt(**common_bemt, cyclic_1s_deg=+th1c, rotation_dir=+1, r_hub_body=r_hub_port)
    right_rot = run_edgewise_bemt(**common_bemt, cyclic_1s_deg=-th1c, rotation_dir=-1, r_hub_body=r_hub_stbd)

    F_rot = left_rot.F_body_N + right_rot.F_body_N
    M_rot = left_rot.M_body_Nm + right_rot.M_body_Nm

    v_tip = float(rpm) * 2.0 * np.pi / 60.0 * float(rot_cfg["radius_m"])
    vi_mean_ms = 0.5 * (left_rot.lambda_i_mean + right_rot.lambda_i_mean) * v_tip
    p_rotor_pair_kW = float(left_rot.power_kW + right_rot.power_kW)

    q_inf = 0.5 * atm.rho_kgm3 * (float(v_tas_ms) ** 2)
    S_w = float(wing_cfg["area_m2"])
    AR_w = float(wing_cfg["aspect_ratio"])
    e_w = float(wing_cfg["oswald_e"])
    b_w = float(wing_cfg["span_m"])
    taper_w = float(wing_cfg["taper_ratio"])
    c_root_w = 2.0 * S_w / (b_w * (1.0 + taper_w))
    mac_w = (2.0 / 3.0) * c_root_w * (1.0 + taper_w + taper_w ** 2) / (1.0 + taper_w)

    # Scheduled conversion flaperons: delta_f = 20 deg * sin(theta_nac)
    flap_frac = float(np.clip(np.sin(np.clip(tn_rad, 0.0, 0.5 * np.pi)), 0.0, 1.0))
    flaperon_deg = float(wing_cfg["flaperon_max_deg"]) * flap_frac
    cl_max_eff = float(wing_cfg["cl_max_clean"]) + flap_frac * (
        float(wing_cfg["cl_max_flaperon"]) - float(wing_cfg["cl_max_clean"])
    )
    alpha_stall_eff_deg = float(wing_cfg["alpha_stall_clean_deg"]) + flap_frac * (
        float(wing_cfg["alpha_stall_flaperon_deg"]) - float(wing_cfg["alpha_stall_clean_deg"])
    )
    dcl_flaperon = 0.22 * flap_frac * float(np.clip(v_tas_ms / 25.0, 0.0, 1.0))

    # Rotor-wake downwash on wing during forward conversion (fades out at hover V->0 where vertical download applies instead)
    fwd_blend = float(np.clip((v_tas_ms - 8.0) / 16.0, 0.0, 1.0))
    downwash_rot_deg = fwd_blend * float(np.degrees(np.arctan2(0.30 * vi_mean_ms * np.sin(tn_rad), max(v_tas_ms, 12.0))))
    alpha_wing_deg = alpha_body_deg + float(wing_cfg["incidence_deg"]) - downwash_rot_deg
    alpha_w_rad = np.radians(alpha_wing_deg)

    cl_unclipped = float(wing_cfg["cl_alpha_per_rad"]) * alpha_w_rad + dcl_flaperon
    if abs(alpha_wing_deg) <= alpha_stall_eff_deg:
        cl_wing = float(np.clip(cl_unclipped, -cl_max_eff, cl_max_eff))
        cd_stall_inc = 0.0
    else:
        sgn_a = float(np.sign(alpha_wing_deg))
        cl_wing = float(
            sgn_a * cl_max_eff * np.cos(alpha_w_rad - sgn_a * np.radians(alpha_stall_eff_deg))
        )
        cd_stall_inc = 0.35 * float(np.sin(alpha_w_rad) ** 2)

    cd_flap_inc = 0.008 * (flap_frac ** 2)
    cd_wing_ind = (cl_wing ** 2) / (np.pi * AR_w * e_w) + cd_flap_inc + cd_stall_inc
    L_wing_N = q_inf * S_w * cl_wing
    D_wing_N = q_inf * S_w * cd_wing_ind

    F_wing = np.array([-D_wing_N * ca + L_wing_N * sa, 0.0, -D_wing_N * sa - L_wing_N * ca])
    M_wing = np.cross(r_wing_ac, F_wing) + np.array([0.0, q_inf * S_w * mac_w * float(wing_cfg["cm0_wing"]), 0.0])

    # Rotor fountain / vertical wake download on wing in hover & low-speed helicopter flight
    download_frac = 0.052 * (float(np.sin(tn_rad)) ** 2) * float(np.exp(-v_tas_ms / 22.0))
    F_download_N = download_frac * max(0.0, left_rot.thrust_N + right_rot.thrust_N)
    F_download = np.array([0.0, 0.0, F_download_N])

    # Fuselage + tilted-nacelle bluff-body parasite drag
    cd_fuse_nac = float(wing_cfg["cd0_clean"]) + float(wing_cfg["cd_nacelle_tilt_increment"]) * (float(np.sin(tn_rad)) ** 2)
    D_fuse_N = q_inf * S_w * cd_fuse_nac
    F_fuse = np.array([-D_fuse_N * ca, 0.0, -D_fuse_N * sa])

    # Horizontal tail (with wing downwash & partial wake dynamic pressure recovery)
    eps_wing_deg = float(np.degrees(2.0 * cl_wing / (np.pi * AR_w))) if v_tas_ms > 2.0 else 0.0
    alpha_ht_deg = alpha_body_deg - eps_wing_deg - 0.35 * downwash_rot_deg
    q_tail = float(emp_cfg["tail_dynamic_pressure_ratio"]) * q_inf + 0.25 * atm.rho_kgm3 * (vi_mean_ms ** 2) * float(np.cos(tn_rad) ** 2)
    cl_ht = float(
        np.clip(
            float(emp_cfg["cl_alpha_htail_per_rad"])
            * (np.radians(alpha_ht_deg) + float(emp_cfg["elevator_effectiveness"]) * np.radians(elev_deg)),
            -float(emp_cfg["cl_max_htail"]),
            float(emp_cfg["cl_max_htail"]),
        )
    )
    L_ht_N = q_tail * float(emp_cfg["s_htail_m2"]) * cl_ht
    F_tail = np.array([L_ht_N * sa, 0.0, -L_ht_N * ca])
    M_tail = np.cross(r_htail_ac, F_tail)

    F_aero = F_wing + F_fuse + F_tail + F_download
    M_aero = M_wing + M_tail

    weight_N = float(gross_mass_kg) * 9.80665
    R_BI = euler_body_from_inertial(np.radians(roll_deg), np.radians(pitch_deg), 0.0)
    F_grav = R_BI @ np.array([0.0, 0.0, weight_N])
    F_inert = np.array([-float(gross_mass_kg) * float(dv_dt_ms2), 0.0, 0.0])

    F_net = F_rot + F_aero + F_grav + F_inert
    M_net = M_rot + M_aero

    prop_eval = evaluate_propulsion(p_rotor_pair_kW, aircraft_cfg, atm=atm)

    L_total_aero_N = L_wing_N + L_ht_N
    D_total_aero_N = D_wing_N + D_fuse_N
    aircraft_LD = float(L_total_aero_N / max(D_total_aero_N, 1.0)) if v_tas_ms > 5.0 else 0.0

    rotor_vert_support_N = float(-F_rot[2] - F_download_N)
    wing_vert_support_N = float(L_wing_N * np.cos(a_b))

    lim_cfg = aircraft_cfg["controls_and_limits"]
    tip_mach = float(max(left_rot.max_tip_mach, right_rot.max_tip_mach))
    rotor_stall_margin_deg = float(min(left_rot.stall_margin_deg, right_rot.stall_margin_deg))
    wing_stall_margin_deg = float(alpha_stall_eff_deg - abs(alpha_wing_deg))
    tip_mach_margin = float(lim_cfg["advancing_tip_mach_limit"]) - tip_mach

    # Normalized control travel margin: min distance to actuator stops across [theta0, theta1s, delta_e]
    th0_lo, th0_hi = map(float, lim_cfg["theta0_deg_bounds"])
    th1s_lo, th1s_hi = map(float, lim_cfg["theta1s_lon_deg_bounds"])
    de_lo, de_hi = map(float, lim_cfg["elevator_deg_bounds"])
    m_th0 = min(th0 - th0_lo, th0_hi - th0) / (0.5 * (th0_hi - th0_lo))
    m_th1s = min(th1s - th1s_lo, th1s_hi - th1s) / (0.5 * (th1s_hi - th1s_lo))
    m_de = min(elev_deg - de_lo, de_hi - elev_deg) / (0.5 * (de_hi - de_lo))
    control_margin_frac = float(min(m_th0, m_th1s, m_de))

    return {
        "F_net": F_net,
        "M_net": M_net,
        "F_rot": F_rot,
        "M_rot": M_rot,
        "F_aero": F_aero,
        "M_aero": M_aero,
        "left_rotor": left_rot,
        "right_rotor": right_rot,
        "propulsion": prop_eval,
        "atmosphere": atm,
        "x_cg_m": x_cg,
        "rpm": float(rpm),
        "flaperon_deg": flaperon_deg,
        "alpha_body_deg": alpha_body_deg,
        "alpha_wing_deg": alpha_wing_deg,
        "alpha_stall_eff_deg": alpha_stall_eff_deg,
        "CL_wing": cl_wing,
        "CL_htail": cl_ht,
        "L_wing_N": float(L_wing_N),
        "D_wing_N": float(D_wing_N),
        "D_fuse_N": float(D_fuse_N),
        "F_download_N": float(F_download_N),
        "L_htail_N": float(L_ht_N),
        "aircraft_LD": aircraft_LD,
        "rotor_support_pct": 100.0 * rotor_vert_support_N / max(weight_N, 1.0),
        "wing_support_pct": 100.0 * wing_vert_support_N / max(weight_N, 1.0),
        "vi_mean_ms": float(vi_mean_ms),
        "tip_mach": tip_mach,
        "tip_mach_margin": tip_mach_margin,
        "rotor_stall_margin_deg": rotor_stall_margin_deg,
        "wing_stall_margin_deg": wing_stall_margin_deg,
        "power_margin_kW": prop_eval.power_margin_kW,
        "control_margin_pct": 100.0 * control_margin_frac,
    }
