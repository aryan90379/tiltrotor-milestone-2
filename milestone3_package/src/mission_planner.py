"""Milestone 3 Integrated End-to-End Tiltrotor Mission Simulator (VTOL -> Conversion -> Cruise -> Reconversion -> Landing)."""

from typing import Any, Dict, List
import numpy as np
import pandas as pd
from .atmosphere import ground_speed_from_tas
from .airfoil_bemt import AirfoilModel
from .aircraft_aero import scheduled_rpm
from .trim_solver import solve_aircraft_trim


def build_mission_timeline(aircraft_cfg: Dict[str, Any], mission_cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Construct smooth, time-ordered kinematic mission profile from user mission_final.yaml."""
    atm_cfg = mission_cfg["atmosphere"]
    headwind = float(atm_cfg["headwind_ms"])
    segs = {int(s["id"]): s for s in mission_cfg["segments"]}

    steps: List[Dict[str, Any]] = []
    t_cur = 0.0

    # --- Segment 1: Vertical takeoff and hover climb ---
    s1 = segs[1]
    dt1 = float(s1["dt_s"])
    t_hov = float(s1["hover_hold_s"])
    h_hov = float(s1["hover_alt_m"])
    n_hov = max(2, int(round(t_hov / dt1)))
    for i in range(n_hov + 1):
        frac = i / n_hov
        alt = h_hov * min(1.0, frac * 1.5)
        roc = (h_hov / (t_hov / 1.5)) if frac < (1.0 / 1.5) else 0.0
        steps.append(
            dict(
                t_s=t_cur + frac * t_hov,
                segment_id=1,
                segment="Takeoff / hover",
                subphase="Hover Takeoff",
                alt_m=alt,
                tas_ms=0.0,
                roc_ms=roc,
                dv_dt_ms2=0.0,
                nacelle_deg=float(s1["nacelle_deg"]),
            )
        )
    t_cur += t_hov

    t_clb1 = float(s1["climb_duration_s"])
    h_end1 = float(s1["end_alt_m"])
    v_end1 = float(s1["end_tas_ms"])
    n_clb1 = max(3, int(round(t_clb1 / dt1)))
    for i in range(1, n_clb1 + 1):
        frac = i / n_clb1
        s_smooth = 0.5 * (1.0 - np.cos(np.pi * frac))
        ds_dt = 0.5 * (np.pi / t_clb1) * np.sin(np.pi * frac)
        alt = h_hov + (h_end1 - h_hov) * s_smooth
        roc = (h_end1 - h_hov) * ds_dt
        tas = v_end1 * s_smooth
        dv_dt = v_end1 * ds_dt
        nac_clb = 90.0 - 2.0 * s_smooth
        steps.append(
            dict(
                t_s=t_cur + frac * t_clb1,
                segment_id=1,
                segment="Takeoff / hover",
                subphase="Helo Climb",
                alt_m=float(alt),
                tas_ms=float(tas),
                roc_ms=float(roc),
                dv_dt_ms2=float(dv_dt),
                nacelle_deg=float(nac_clb),
            )
        )
    t_cur += t_clb1

    # --- Segment 2: Conversion to airplane mode ---
    s2 = segs[2]
    t_conv = float(s2["duration_s"])
    n_conv = int(s2["n_steps"])
    v0_2, v1_2 = float(s2["start_tas_ms"]), float(s2["end_tas_ms"])
    h0_2, h1_2 = float(s2["start_alt_m"]), float(s2["end_alt_m"])
    nac0_2, nac1_2 = float(s2["start_nacelle_deg"]), float(s2["end_nacelle_deg"])
    for i in range(1, n_conv + 1):
        frac = i / n_conv
        base_s = 0.5 * (1.0 - np.cos(np.pi * frac))
        s_v = base_s ** 0.90
        # Tilt nacelles smoothly ahead of peak speed so forward thrust vector drives acceleration
        s_nac = base_s ** 0.78
        tas = v0_2 + (v1_2 - v0_2) * s_v
        nac = nac0_2 + (nac1_2 - nac0_2) * s_nac
        alt = h0_2 + (h1_2 - h0_2) * base_s
        roc = (h1_2 - h0_2) * 0.5 * (np.pi / t_conv) * np.sin(np.pi * frac)
        dv_dt = (v1_2 - v0_2) * 0.90 * (max(base_s, 1e-4) ** -0.10) * 0.5 * (np.pi / t_conv) * np.sin(np.pi * frac)
        steps.append(
            dict(
                t_s=t_cur + frac * t_conv,
                segment_id=2,
                segment="Conversion",
                subphase="Outbound Conversion",
                alt_m=float(alt),
                tas_ms=float(tas),
                roc_ms=float(roc),
                dv_dt_ms2=float(dv_dt),
                nacelle_deg=float(nac),
            )
        )
    t_cur += t_conv

    # Pre-calculate distance covered in Segments 1, 2, 3a (climb), 3c (descent), 4, 5 to size Segment 3b level cruise
    s3 = segs[3]
    s4 = segs[4]
    s5 = segs[5]

    t_clb3 = float(s3["climb_duration_s"])
    dt_cd3 = float(s3["dt_climb_descent_s"])
    h_cr = float(s3["cruise_alt_m"])
    v_cr = float(s3["cruise_tas_ms"])
    n_clb3 = max(4, int(round(t_clb3 / dt_cd3)))

    clb3_steps = []
    for i in range(1, n_clb3 + 1):
        frac = i / n_clb3
        s_sm = 0.5 * (1.0 - np.cos(np.pi * frac))
        ds_dt = 0.5 * (np.pi / t_clb3) * np.sin(np.pi * frac)
        alt = h1_2 + (h_cr - h1_2) * s_sm
        roc = (h_cr - h1_2) * ds_dt
        tas = v1_2 + (v_cr - v1_2) * s_sm
        dv_dt = (v_cr - v1_2) * ds_dt
        clb3_steps.append((frac * t_clb3, float(alt), float(tas), float(roc), float(dv_dt)))

    t_des3 = float(s3["descent_duration_s"])
    h_des_end = float(s3["end_descent_alt_m"])
    v_des_end = float(s3["end_descent_tas_ms"])
    n_des3 = max(4, int(round(t_des3 / dt_cd3)))
    des3_steps = []
    for i in range(1, n_des3 + 1):
        frac = i / n_des3
        s_sm = 0.5 * (1.0 - np.cos(np.pi * frac))
        ds_dt = 0.5 * (np.pi / t_des3) * np.sin(np.pi * frac)
        alt = h_cr + (h_des_end - h_cr) * s_sm
        roc = (h_des_end - h_cr) * ds_dt
        tas = v_cr + (v_des_end - v_cr) * s_sm
        dv_dt = (v_des_end - v_cr) * ds_dt
        des3_steps.append((frac * t_des3, float(alt), float(tas), float(roc), float(dv_dt)))

    # Estimate non-level-cruise ground distance [m]
    def _seg_dist_est() -> float:
        d_m = 0.0
        for idx_s in range(1, len(steps)):
            dt_i = steps[idx_s]["t_s"] - steps[idx_s - 1]["t_s"]
            v_m = 0.5 * (steps[idx_s]["tas_ms"] + steps[idx_s - 1]["tas_ms"])
            d_m += max(0.0, v_m - (headwind if v_m > 0.5 else 0.0)) * dt_i
        # Climb 3a + Descent 3c
        d_m += max(0.0, 0.5 * (v1_2 + v_cr) - headwind) * t_clb3
        d_m += max(0.0, 0.5 * (v_cr + v_des_end) - headwind) * t_des3
        # Reconversion 4 + Landing 5
        d_m += max(0.0, 0.5 * (float(s4["start_tas_ms"]) + float(s4["end_tas_ms"])) - headwind) * float(s4["duration_s"])
        d_m += max(0.0, 0.5 * float(s4["end_tas_ms"]) - 0.5 * headwind) * float(s5["approach_duration_s"])
        return d_m

    non_cr_dist_m = _seg_dist_est()
    target_dist_m = float(s3["target_total_mission_distance_km"]) * 1000.0
    rem_cruise_dist_m = max(50000.0, target_dist_m - non_cr_dist_m)
    gs_cr = ground_speed_from_tas(v_cr, headwind, 0.0)
    t_level_cr = rem_cruise_dist_m / max(gs_cr, 10.0)

    # Append 3a (Airplane Climb)
    for dt_rel, alt, tas, roc, dv_dt in clb3_steps:
        steps.append(
            dict(
                t_s=t_cur + dt_rel,
                segment_id=3,
                segment="Cruise",
                subphase="Airplane Climb",
                alt_m=alt,
                tas_ms=tas,
                roc_ms=roc,
                dv_dt_ms2=dv_dt,
                nacelle_deg=0.0,
            )
        )
    t_cur += t_clb3

    # Append 3b (Level High-Speed Cruise)
    dt_cr = float(s3["dt_cruise_s"])
    n_cr = max(10, int(round(t_level_cr / dt_cr)))
    for i in range(1, n_cr + 1):
        frac = i / n_cr
        steps.append(
            dict(
                t_s=t_cur + frac * t_level_cr,
                segment_id=3,
                segment="Cruise",
                subphase="Level Cruise",
                alt_m=h_cr,
                tas_ms=v_cr,
                roc_ms=0.0,
                dv_dt_ms2=0.0,
                nacelle_deg=0.0,
            )
        )
    t_cur += t_level_cr

    # Append 3c (Airplane Descent)
    for dt_rel, alt, tas, roc, dv_dt in des3_steps:
        steps.append(
            dict(
                t_s=t_cur + dt_rel,
                segment_id=3,
                segment="Cruise",
                subphase="Airplane Descent",
                alt_m=alt,
                tas_ms=tas,
                roc_ms=roc,
                dv_dt_ms2=dv_dt,
                nacelle_deg=0.0,
            )
        )
    t_cur += t_des3

    # --- Segment 4: Reconversion to helicopter mode ---
    t_rec = float(s4["duration_s"])
    n_rec = int(s4["n_steps"])
    v0_4, v1_4 = float(s4["start_tas_ms"]), float(s4["end_tas_ms"])
    h0_4, h1_4 = float(s4["start_alt_m"]), float(s4["end_alt_m"])
    nac0_4, nac1_4 = float(s4["start_nacelle_deg"]), float(s4["end_nacelle_deg"])
    for i in range(1, n_rec + 1):
        frac = i / n_rec
        base_s = 0.5 * (1.0 - np.cos(np.pi * frac))
        ds_dt = 0.5 * (np.pi / t_rec) * np.sin(np.pi * frac)
        # Follow conversion corridor: smooth deceleration while tilting to >=82 deg before V drops below Vs
        s_v = base_s
        s_nac = 1.0 - ((1.0 - base_s) ** 2.15)
        tas = v0_4 + (v1_4 - v0_4) * s_v
        nac = nac0_4 + (nac1_4 - nac0_4) * s_nac
        alt = h0_4 + (h1_4 - h0_4) * base_s
        roc = (h1_4 - h0_4) * ds_dt
        dv_dt = (v1_4 - v0_4) * ds_dt
        steps.append(
            dict(
                t_s=t_cur + frac * t_rec,
                segment_id=4,
                segment="Reconversion",
                subphase="Inbound Reconversion",
                alt_m=float(alt),
                tas_ms=float(tas),
                roc_ms=float(roc),
                dv_dt_ms2=float(dv_dt),
                nacelle_deg=float(nac),
            )
        )
    t_cur += t_rec

    # --- Segment 5: Descent and vertical landing ---
    dt5 = float(s5["dt_s"])
    t_app = float(s5["approach_duration_s"])
    h_flare = float(s5["flare_hover_alt_m"])
    n_app = max(3, int(round(t_app / dt5)))
    for i in range(1, n_app + 1):
        frac = i / n_app
        s_sm = 0.5 * (1.0 - np.cos(np.pi * frac))
        ds_dt = 0.5 * (np.pi / t_app) * np.sin(np.pi * frac)
        alt = h1_4 + (h_flare - h1_4) * s_sm
        roc = (h_flare - h1_4) * ds_dt
        tas = v1_4 * (1.0 - s_sm)
        dv_dt = -v1_4 * ds_dt
        nac_app = float(nac1_4) + (90.0 - float(nac1_4)) * s_sm
        steps.append(
            dict(
                t_s=t_cur + frac * t_app,
                segment_id=5,
                segment="Landing",
                subphase="Helo Approach",
                alt_m=float(alt),
                tas_ms=float(tas),
                roc_ms=float(roc),
                dv_dt_ms2=float(dv_dt),
                nacelle_deg=float(nac_app),
            )
        )
    t_cur += t_app

    t_td = float(s5["touchdown_hover_s"])
    h_end5 = float(s5["end_alt_m"])
    n_td = max(2, int(round(t_td / dt5)))
    for i in range(1, n_td + 1):
        frac = i / n_td
        s_sm = 0.5 * (1.0 - np.cos(np.pi * frac))
        ds_dt = 0.5 * (np.pi / t_td) * np.sin(np.pi * frac)
        alt = h_flare + (h_end5 - h_flare) * s_sm
        roc = (h_end5 - h_flare) * ds_dt
        steps.append(
            dict(
                t_s=t_cur + frac * t_td,
                segment_id=5,
                segment="Landing",
                subphase="Vertical Touchdown",
                alt_m=float(alt),
                tas_ms=0.0,
                roc_ms=float(roc),
                dv_dt_ms2=0.0,
                nacelle_deg=float(s5["nacelle_deg"]),
            )
        )

    return steps


def run_end_to_end_mission(
    aircraft_cfg: Dict[str, Any], mission_cfg: Dict[str, Any], airfoil: AirfoilModel | None = None
) -> Dict[str, Any]:
    """Simulate full end-to-end VTOL mission with continuous 6-DOF trim, mass/fuel integration, and limit checks."""
    if airfoil is None:
        airfoil = AirfoilModel(aircraft_cfg["geometry"]["rotor"]["airfoil"])

    timeline = build_mission_timeline(aircraft_cfg, mission_cfg)
    atm_cfg = mission_cfg["atmosphere"]
    load_cfg = mission_cfg["loading"]
    mass_cfg = aircraft_cfg["mass_breakdown_kg"]

    delta_t_K = float(atm_cfg["isa_delta_t_degC"])
    headwind_ms = float(atm_cfg["headwind_ms"])

    empty_kg = float(mass_cfg["empty_mass_kg"])
    payload_kg = float(load_cfg["initial_payload_kg"])
    fuel_init_kg = float(load_cfg["initial_fuel_kg"])
    fuel_cap_kg = float(mass_cfg["fuel_capacity_usable_kg"])
    min_reserve_tank_kg = float(load_cfg["reserve_fuel_min_fraction"]) * fuel_cap_kg

    # Pre-evaluate 20-min airplane loiter reserve fuel requirement at end-of-mission weight
    est_landing_mass_kg = empty_kg + payload_kg + min_reserve_tank_kg + 80.0
    loiter_trim = solve_aircraft_trim(
        v_tas_ms=float(load_cfg["reserve_loiter_speed_ms"]),
        theta_nac_deg=0.0,
        aircraft_cfg=aircraft_cfg,
        airfoil=airfoil,
        altitude_m=float(load_cfg["reserve_loiter_altitude_m"]),
        delta_t_K=delta_t_K,
        gamma_deg=0.0,
        gross_mass_kg=est_landing_mass_kg,
        verify_high_res=True,
    )
    loiter_burn_kg = (
        loiter_trim["eval"]["propulsion"].fuel_flow_kg_s * float(load_cfg["reserve_loiter_duration_min"]) * 60.0
    )
    protected_reserve_kg = float(max(min_reserve_tank_kg, loiter_burn_kg))

    fuel_rem_kg = fuel_init_kg
    cum_dist_m = 0.0
    u_prev = None
    rows: List[Dict[str, Any]] = []

    for k, st in enumerate(timeline):
        t_s = float(st["t_s"])
        dt_s = 0.0 if k == 0 else (t_s - float(timeline[k - 1]["t_s"]))
        tas_ms = float(st["tas_ms"])
        roc_ms = float(st["roc_ms"])
        alt_m = float(st["alt_m"])
        nac_deg = float(st["nacelle_deg"])
        dv_dt = float(st["dv_dt_ms2"])

        if tas_ms > 5.0:
            gamma_deg = float(np.degrees(np.arctan2(roc_ms, tas_ms)))
        else:
            gamma_deg = 0.0

        v_eff_bemt = tas_ms
        gross_kg = empty_kg + payload_kg + fuel_rem_kg

        # During steady level cruise, reuse previous trim seed & fast high-accuracy solve
        high_res = (st["subphase"] != "Level Cruise") or (k % 5 == 0)
        tr = solve_aircraft_trim(
            v_tas_ms=v_eff_bemt,
            theta_nac_deg=nac_deg,
            aircraft_cfg=aircraft_cfg,
            airfoil=airfoil,
            altitude_m=alt_m,
            delta_t_K=delta_t_K,
            gamma_deg=gamma_deg,
            gross_mass_kg=gross_kg,
            dv_dt_ms2=dv_dt,
            u_init=u_prev,
            verify_high_res=high_res,
        )
        u_prev = tr["u_trim_deg"]
        ev = tr["eval"]
        prop = ev["propulsion"]

        # Add potential-energy climb / descent power increment for vertical helicopter climb/descent when V_TAS ~ 0
        if tas_ms < 5.0 and abs(roc_ms) > 0.05:
            p_climb_kW = (gross_kg * 9.80665 * roc_ms) / 1000.0 / float(aircraft_cfg["propulsion"]["gearbox_efficiency"])
            p_req_kW = float(np.clip(prop.p_shaft_req_kW + (p_climb_kW if roc_ms > 0 else 0.45 * p_climb_kW), 140.0, 5000.0))
        else:
            p_req_kW = float(prop.p_shaft_req_kW)

        p_avail_kW = float(prop.p_avail_effective_kW)
        p_margin_kW = p_avail_kW - p_req_kW
        mdot_kgs = float(prop.sfc_kg_per_kWh) * p_req_kW / 3600.0

        if dt_s > 0.0:
            gs_prev = rows[-1]["gs_ms"]
            gs_now = ground_speed_from_tas(tas_ms, headwind_ms, gamma_deg)
            cum_dist_m += 0.5 * (gs_prev + gs_now) * dt_s
            mdot_prev = rows[-1]["fuel_flow_kg_h"] / 3600.0
            fuel_rem_kg = max(0.0, fuel_rem_kg - 0.5 * (mdot_prev + mdot_kgs) * dt_s)
        else:
            gs_now = ground_speed_from_tas(tas_ms, headwind_ms, gamma_deg)

        gross_kg = empty_kg + payload_kg + fuel_rem_kg
        fuel_margin_kg = fuel_rem_kg - protected_reserve_kg

        # Normalized margins (0.0 = at limit, >0 = safe margin)
        norm_margins = {
            "Fuel Reserve": fuel_margin_kg / max(fuel_init_kg - protected_reserve_kg, 1.0),
            "Power Margin": p_margin_kW / max(p_avail_kW, 1.0),
            "Rotor Stall": ev["rotor_stall_margin_deg"] / 15.75,
            "Tip Mach": ev["tip_mach_margin"] / 0.25,
            "Control Travel": ev["control_margin_pct"] / 100.0,
        }
        if tas_ms >= 18.0 or nac_deg < 80.0:
            norm_margins["Wing Stall"] = ev["wing_stall_margin_deg"] / 16.0
            wing_marg_active = ev["wing_stall_margin_deg"]
        else:
            norm_margins["Wing Stall"] = 1.0
            wing_marg_active = 16.0

        gov_name = min(norm_margins, key=lambda name: norm_margins[name])
        min_norm = float(norm_margins[gov_name])
        step_feasible = bool(tr["feasible"] and (fuel_margin_kg >= 0.0) and (p_margin_kW >= 0.0))

        nac_rate = 0.0 if k == 0 or dt_s <= 0 else abs(nac_deg - rows[-1]["nacelle_deg"]) / dt_s

        rows.append(
            dict(
                step=k,
                time_s=t_s,
                time_min=t_s / 60.0,
                segment_id=int(st["segment_id"]),
                segment=st["segment"],
                subphase=st["subphase"],
                altitude_m=alt_m,
                tas_ms=tas_ms,
                gs_ms=gs_now,
                roc_ms=roc_ms,
                gamma_deg=gamma_deg,
                cum_dist_km=cum_dist_m / 1000.0,
                gross_mass_kg=gross_kg,
                payload_kg=payload_kg,
                fuel_rem_kg=fuel_rem_kg,
                fuel_burned_kg=fuel_init_kg - fuel_rem_kg,
                fuel_margin_kg=fuel_margin_kg,
                nacelle_deg=nac_deg,
                nacelle_rate_deg_s=nac_rate,
                rpm=tr["rpm"],
                flaperon_deg=ev["flaperon_deg"],
                x_cg_m=ev["x_cg_m"],
                theta0_deg=tr["theta0_deg"],
                theta1s_deg=tr["theta1s_deg"],
                pitch_deg=tr["pitch_deg"],
                elevator_deg=tr["elevator_deg"],
                p_req_kW=p_req_kW,
                p_avail_kW=p_avail_kW,
                p_eng_thermo_kW=prop.p_eng_thermo_avail_kW,
                p_trans_limit_kW=prop.p_trans_limit_kW,
                power_margin_kW=p_margin_kW,
                sfc_kg_kWh=prop.sfc_kg_per_kWh,
                fuel_flow_kg_h=mdot_kgs * 3600.0,
                rotor_support_pct=ev["rotor_support_pct"],
                wing_support_pct=ev["wing_support_pct"],
                aircraft_LD=ev["aircraft_LD"],
                FM_hover=ev["left_rotor"].FM_hover,
                eta_prop=ev["left_rotor"].eta_prop,
                tip_mach=ev["tip_mach"],
                tip_mach_margin=ev["tip_mach_margin"],
                rotor_stall_margin_deg=ev["rotor_stall_margin_deg"],
                wing_stall_margin_deg=wing_marg_active,
                control_margin_pct=ev["control_margin_pct"],
                norm_fuel_margin=norm_margins["Fuel Reserve"],
                norm_power_margin=norm_margins["Power Margin"],
                norm_rotor_stall=norm_margins["Rotor Stall"],
                norm_tip_mach=norm_margins["Tip Mach"],
                norm_wing_stall=norm_margins["Wing Stall"],
                norm_control_margin=norm_margins["Control Travel"],
                min_norm_margin=min_norm,
                governing_margin=gov_name,
                force_res_N=tr["force_res_max_N"],
                moment_res_Nm=tr["moment_res_max_Nm"],
                feasible=step_feasible,
                status=tr["status"],
            )
        )

    df = pd.DataFrame(rows)

    # Re-evaluate exact Reserve segment starting from actual touchdown mass
    landing_mass_kg = float(df["gross_mass_kg"].iloc[-1])
    landing_fuel_kg = float(df["fuel_rem_kg"].iloc[-1])
    res_trim = solve_aircraft_trim(
        v_tas_ms=float(load_cfg["reserve_loiter_speed_ms"]),
        theta_nac_deg=0.0,
        aircraft_cfg=aircraft_cfg,
        airfoil=airfoil,
        altitude_m=float(load_cfg["reserve_loiter_altitude_m"]),
        delta_t_K=delta_t_K,
        gamma_deg=0.0,
        gross_mass_kg=landing_mass_kg,
        verify_high_res=True,
    )
    res_ev = res_trim["eval"]
    res_p_req = float(res_ev["propulsion"].p_shaft_req_kW)
    res_p_avail = float(res_ev["propulsion"].p_avail_effective_kW)
    res_burn_20min_kg = float(res_ev["propulsion"].fuel_flow_kg_s) * float(load_cfg["reserve_loiter_duration_min"]) * 60.0

    # Build segment-end summary table (matching Slide 21)
    seg_summary_rows = []
    for seg_name in ["Takeoff / hover", "Conversion", "Cruise", "Reconversion", "Landing"]:
        sub_df = df[df["segment"] == seg_name]
        end_r = sub_df.iloc[-1]
        min_idx = sub_df["min_norm_margin"].idxmin()
        min_r = df.loc[min_idx]
        seg_summary_rows.append(
            dict(
                Segment=seg_name,
                Time_min=float(end_r["time_min"]),
                Duration_min=float(sub_df["time_min"].iloc[-1] - sub_df["time_min"].iloc[0])
                if seg_name == "Takeoff / hover"
                else float(end_r["time_min"] - df.loc[sub_df.index[0] - 1, "time_min"]),
                Altitude_m=float(end_r["altitude_m"]),
                GS_ms=float(end_r["gs_ms"]),
                TAS_ms=float(end_r["tas_ms"]),
                Cum_dist_km=float(end_r["cum_dist_km"]),
                Gross_mass_kg=float(end_r["gross_mass_kg"]),
                Fuel_left_kg=float(end_r["fuel_rem_kg"]),
                Fuel_burned_seg_kg=float(
                    (fuel_init_kg if seg_name == "Takeoff / hover" else df.loc[sub_df.index[0] - 1, "fuel_rem_kg"])
                    - end_r["fuel_rem_kg"]
                ),
                Nacelle_deg=float(end_r["nacelle_deg"]),
                RPM=float(end_r["rpm"]),
                P_req_end_kW=float(end_r["p_req_kW"]),
                P_req_peak_kW=float(sub_df["p_req_kW"].max()),
                P_avail_end_kW=float(end_r["p_avail_kW"]),
                Min_norm_margin=float(min_r["min_norm_margin"]),
                Min_margin_label=f"{min_r['governing_margin']} ({100.0*min_r['min_norm_margin']:.1f}%)",
            )
        )

    # Segment 6: Reserve row
    fuel_after_loiter_kg = landing_fuel_kg - res_burn_20min_kg
    seg_summary_rows.append(
        dict(
            Segment="Reserve",
            Time_min=float(df["time_min"].iloc[-1] + float(load_cfg["reserve_loiter_duration_min"])),
            Duration_min=float(load_cfg["reserve_loiter_duration_min"]),
            Altitude_m=float(load_cfg["reserve_loiter_altitude_m"]),
            GS_ms=ground_speed_from_tas(float(load_cfg["reserve_loiter_speed_ms"]), headwind_ms, 0.0),
            TAS_ms=float(load_cfg["reserve_loiter_speed_ms"]),
            Cum_dist_km=float(df["cum_dist_km"].iloc[-1]),
            Gross_mass_kg=landing_mass_kg - res_burn_20min_kg,
            Fuel_left_kg=fuel_after_loiter_kg,
            Fuel_burned_seg_kg=res_burn_20min_kg,
            Nacelle_deg=0.0,
            RPM=float(res_trim["rpm"]),
            P_req_end_kW=res_p_req,
            P_req_peak_kW=res_p_req,
            P_avail_end_kW=res_p_avail,
            Min_norm_margin=float(landing_fuel_kg - protected_reserve_kg) / max(fuel_init_kg - protected_reserve_kg, 1.0),
            Min_margin_label=f"Surplus +{landing_fuel_kg - protected_reserve_kg:.1f} kg",
        )
    )

    seg_df = pd.DataFrame(seg_summary_rows)

    return {
        "telemetry": df,
        "segment_summary": seg_df,
        "protected_reserve_kg": protected_reserve_kg,
        "min_tank_reserve_kg": min_reserve_tank_kg,
        "loiter_20min_burn_kg": res_burn_20min_kg,
        "landing_fuel_kg": landing_fuel_kg,
        "landing_mass_kg": landing_mass_kg,
        "total_fuel_burned_kg": fuel_init_kg - landing_fuel_kg,
        "total_time_min": float(df["time_min"].iloc[-1]),
        "total_distance_km": float(df["cum_dist_km"].iloc[-1]),
        "all_steps_feasible": bool(df["feasible"].all()),
        "reserve_trim": res_trim,
    }
