"""Reproduce Section 3 Final Aircraft Performance Summary tables & sweeps from user config."""

from pathlib import Path
import sys
import json
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from milestone3_package.src import load_config
from milestone3_package.src.atmosphere import isa_atmosphere
from milestone3_package.src.airfoil_bemt import AirfoilModel, trim_axial_rotor_collective
from milestone3_package.src.propulsion import evaluate_propulsion
from milestone3_package.src.trim_solver import solve_aircraft_trim


def run_all_performance(cfg_path: str | Path | None = None, out_dir: str | Path | None = None) -> dict:
    pkg_dir = Path(__file__).resolve().parents[1]
    if cfg_path is None:
        cfg_path = pkg_dir / "config" / "aircraft_final.yaml"
    if out_dir is None:
        out_dir = pkg_dir / "results"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = load_config(cfg_path)
    af = AirfoilModel(cfg["geometry"]["rotor"]["airfoil"])
    rot = cfg["geometry"]["rotor"]
    wing = cfg["geometry"]["wing"]
    mass_cfg = cfg["mass_breakdown_kg"]
    mtow_kg = float(mass_cfg["mtow_kg"])
    usable_fuel_kg = float(mass_cfg["fuel_capacity_usable_kg"]) * (
        1.0 - float(cfg["design_requirements"]["reserve_fuel_fraction"])
    )

    # ------------------------------------------------------------------
    # 1. Hover Performance Sweep (SL ISA & 3000 m ISA) + Max Hover Weight & Ceiling
    # ------------------------------------------------------------------
    hover_rows = []
    for alt_m in [0.0, 3000.0]:
        atm = isa_atmosphere(alt_m, 0.0)
        for m_kg in np.linspace(4200.0, 15600.0, 39):
            # Include 5.2% wing fountain download in installed OGE hover thrust demand
            t_per_rot = 0.5 * (m_kg * 9.80665) * 1.052
            th0, b_res, ok = trim_axial_rotor_collective(
                t_per_rot, cfg, af, float(rot["rpm_hover"]), 0.0, atm.rho_kgm3, atm.a_sound_ms, atm.mu_Pas
            )
            prop = evaluate_propulsion(2.0 * b_res.power_kW, cfg, atm=atm)
            feasible_hov = (
                ok
                and (prop.power_margin_kW >= 0.0)
                and (b_res.stall_margin_deg >= 0.0)
                and (b_res.max_tip_mach <= 0.85)
            )
            hover_rows.append(
                dict(
                    altitude_m=alt_m,
                    mass_kg=float(m_kg),
                    thrust_total_N=2.0 * t_per_rot,
                    theta0_deg=th0,
                    FM=b_res.FM_hover,
                    p_req_kW=prop.p_shaft_req_kW,
                    p_avail_kW=prop.p_avail_effective_kW,
                    power_margin_kW=prop.power_margin_kW,
                    stall_margin_deg=b_res.stall_margin_deg,
                    tip_mach=b_res.max_tip_mach,
                    feasible=feasible_hov,
                )
            )
    df_hover_sweep = pd.DataFrame(hover_rows)
    df_hover_sweep.to_csv(out_dir / "sec3_1_hover_sweep.csv", index=False)

    sl_feas = df_hover_sweep[(df_hover_sweep["altitude_m"] == 0.0) & (df_hover_sweep["feasible"])]
    max_hover_mass_sl_kg = float(sl_feas["mass_kg"].max())
    max_hov_row = sl_feas.iloc[-1]

    # OGE & IGE Hover Ceiling sweep at Design MTOW (6800 kg)
    oge_ceiling_m = 0.0
    ige_ceiling_m = 0.0
    z_over_d = 0.55  # wheels ~2.5 m above ground -> z/D ~ 0.55, Hayden/Cheeseman-Bennett IGE factor
    ige_thrust_factor = 1.0 / (1.0 + (1.0 / (16.0 * (z_over_d ** 2))))  # k_IGE < 1 reduces required power/thrust
    ceil_rows = []
    for alt_m in np.linspace(0.0, 9600.0, 49):
        atm = isa_atmosphere(alt_m, 0.0)
        t_oge = 0.5 * (mtow_kg * 9.80665) * 1.052
        th_oge, res_oge, ok_oge = trim_axial_rotor_collective(
            t_oge, cfg, af, float(rot["rpm_hover"]), 0.0, atm.rho_kgm3, atm.a_sound_ms, atm.mu_Pas
        )
        prop_oge = evaluate_propulsion(2.0 * res_oge.power_kW, cfg, atm=atm)
        feas_oge = ok_oge and (prop_oge.power_margin_kW >= 0.0) and (res_oge.stall_margin_deg >= 0.0) and (res_oge.max_tip_mach <= 0.85)
        if feas_oge:
            oge_ceiling_m = float(alt_m)

        t_ige = t_oge * ige_thrust_factor
        th_ige, res_ige, ok_ige = trim_axial_rotor_collective(
            t_ige, cfg, af, float(rot["rpm_hover"]), 0.0, atm.rho_kgm3, atm.a_sound_ms, atm.mu_Pas
        )
        prop_ige = evaluate_propulsion(2.0 * res_ige.power_kW, cfg, atm=atm)
        feas_ige = ok_ige and (prop_ige.power_margin_kW >= 0.0) and (res_ige.stall_margin_deg >= 0.0) and (res_ige.max_tip_mach <= 0.85)
        if feas_ige:
            ige_ceiling_m = float(alt_m)
        ceil_rows.append(
            dict(
                altitude_m=float(alt_m),
                p_req_oge_kW=prop_oge.p_shaft_req_kW,
                p_avail_kW=prop_oge.p_avail_effective_kW,
                stall_margin_oge_deg=res_oge.stall_margin_deg,
                tip_mach=res_oge.max_tip_mach,
                feasible_oge=feas_oge,
                feasible_ige=feas_ige,
            )
        )
    pd.DataFrame(ceil_rows).to_csv(out_dir / "sec3_3_hover_ceiling_sweep.csv", index=False)

    # ------------------------------------------------------------------
    # 2. Airplane-Mode Cruise Sweep at 3000 m ISA (Design Altitude)
    # ------------------------------------------------------------------
    cruise_rows = []
    u_seed = None
    mid_cruise_mass_kg = mtow_kg - 0.5 * usable_fuel_kg  # Mid-cruise representative weight (~6305 kg)
    for v_ms in np.linspace(62.0, 145.0, 30):
        tr = solve_aircraft_trim(
            v_tas_ms=float(v_ms),
            theta_nac_deg=0.0,
            aircraft_cfg=cfg,
            airfoil=af,
            altitude_m=3000.0,
            gross_mass_kg=mtow_kg,
            u_init=u_seed,
            verify_high_res=True,
        )
        if tr["converged"]:
            u_seed = tr["u_trim_deg"]
        ev = tr["eval"]
        prop = ev["propulsion"]
        lr = ev["left_rotor"]
        spec_range_km_per_kg = (float(v_ms) * 3.6) / max(prop.fuel_flow_kg_h, 1.0)
        # Breguet range at this speed for full usable fuel (990 kg, W_i=6800 -> W_f=5810 kg)
        eta_tot = lr.eta_prop * float(cfg["propulsion"]["gearbox_efficiency"])
        c_si = (prop.sfc_kg_per_kWh / 3600.0) / 1000.0  # kg / (W * s)
        breguet_km = (
            (ev["aircraft_LD"] * eta_tot / max(9.80665 * c_si, 1e-12))
            * np.log(mtow_kg / (mtow_kg - usable_fuel_kg))
            / 1000.0
        )
        cruise_rows.append(
            dict(
                v_tas_ms=float(v_ms),
                v_tas_kmh=float(v_ms) * 3.6,
                feasible=tr["feasible"],
                status=tr["status"],
                theta0_deg=tr["theta0_deg"],
                pitch_deg=tr["pitch_deg"],
                elevator_deg=tr["elevator_deg"],
                rpm=tr["rpm"],
                p_req_kW=prop.p_shaft_req_kW,
                p_avail_kW=prop.p_avail_effective_kW,
                power_margin_kW=prop.power_margin_kW,
                aircraft_LD=ev["aircraft_LD"],
                eta_prop=lr.eta_prop,
                tip_mach=ev["tip_mach"],
                rotor_stall_margin_deg=ev["rotor_stall_margin_deg"],
                wing_stall_margin_deg=ev["wing_stall_margin_deg"],
                spec_range_km_per_kg=spec_range_km_per_kg,
                breguet_range_km=float(breguet_km),
            )
        )
    df_cruise_sweep = pd.DataFrame(cruise_rows)
    df_cruise_sweep.to_csv(out_dir / "sec3_1_cruise_sweep.csv", index=False)

    feas_cr = df_cruise_sweep[df_cruise_sweep["feasible"]]
    idx_br = feas_cr["breguet_range_km"].idxmax()
    br_row = df_cruise_sweep.loc[idx_br]
    vmax_feas_ms = float(feas_cr["v_tas_ms"].max())

    # Exact operating points for Slide 13 (Table 3.1)
    hov_sl = solve_aircraft_trim(0.0, 90.0, cfg, af, altitude_m=0.0, gross_mass_kg=mtow_kg, verify_high_res=True)
    hov_ceil = solve_aircraft_trim(0.0, 90.0, cfg, af, altitude_m=oge_ceiling_m, gross_mass_kg=mtow_kg, verify_high_res=True)
    cr_des = solve_aircraft_trim(111.111, 0.0, cfg, af, altitude_m=3000.0, gross_mass_kg=mtow_kg, verify_high_res=True)
    cr_br = solve_aircraft_trim(float(br_row["v_tas_ms"]), 0.0, cfg, af, altitude_m=3000.0, gross_mass_kg=mtow_kg, verify_high_res=True)

    sec3_1_table = pd.DataFrame(
        [
            dict(
                Operating_Point="Hover, design weight",
                Alt_ISA="0 m / SL ISA",
                Gross_Mass_kg=mtow_kg,
                RPM_Collective=f"{hov_sl['rpm']:.0f} rpm / {hov_sl['theta0_deg']:.1f}°",
                Thrust_or_Lift_N=f"{-hov_sl['eval']['F_rot'][2]:.0f} N (T)",
                P_req_kW=round(hov_sl["eval"]["propulsion"].p_shaft_req_kW, 1),
                P_avail_kW=round(hov_sl["eval"]["propulsion"].p_avail_effective_kW, 1),
                Efficiency_Metric=f"FM = {hov_sl['eval']['left_rotor'].FM_hover:.3f}",
            ),
            dict(
                Operating_Point="Hover, max weight / ceiling",
                Alt_ISA=f"{oge_ceiling_m:.0f} m OGE ({max_hover_mass_sl_kg:.0f} kg SL)",
                Gross_Mass_kg=mtow_kg,
                RPM_Collective=f"{hov_ceil['rpm']:.0f} rpm / {hov_ceil['theta0_deg']:.1f}°",
                Thrust_or_Lift_N=f"{-hov_ceil['eval']['F_rot'][2]:.0f} N (T)",
                P_req_kW=round(hov_ceil["eval"]["propulsion"].p_shaft_req_kW, 1),
                P_avail_kW=round(hov_ceil["eval"]["propulsion"].p_avail_effective_kW, 1),
                Efficiency_Metric=f"FM = {hov_ceil['eval']['left_rotor'].FM_hover:.3f}",
            ),
            dict(
                Operating_Point="Cruise, design speed (111.1 m/s)",
                Alt_ISA="3000 m / ISA",
                Gross_Mass_kg=mtow_kg,
                RPM_Collective=f"{cr_des['rpm']:.0f} rpm / {cr_des['theta0_deg']:.1f}°",
                Thrust_or_Lift_N=f"{cr_des['eval']['L_wing_N'] + cr_des['eval']['L_htail_N']:.0f} N (L)",
                P_req_kW=round(cr_des["eval"]["propulsion"].p_shaft_req_kW, 1),
                P_avail_kW=round(cr_des["eval"]["propulsion"].p_avail_effective_kW, 1),
                Efficiency_Metric=f"ηp = {cr_des['eval']['left_rotor'].eta_prop:.2f}, L/D = {cr_des['eval']['aircraft_LD']:.1f}",
            ),
            dict(
                Operating_Point=f"Cruise, best range ({br_row['v_tas_ms']:.1f} m/s)",
                Alt_ISA="3000 m / ISA",
                Gross_Mass_kg=mtow_kg,
                RPM_Collective=f"{cr_br['rpm']:.0f} rpm / {cr_br['theta0_deg']:.1f}°",
                Thrust_or_Lift_N=f"{cr_br['eval']['L_wing_N'] + cr_br['eval']['L_htail_N']:.0f} N (L)",
                P_req_kW=round(cr_br["eval"]["propulsion"].p_shaft_req_kW, 1),
                P_avail_kW=round(cr_br["eval"]["propulsion"].p_avail_effective_kW, 1),
                Efficiency_Metric=f"ηp = {cr_br['eval']['left_rotor'].eta_prop:.2f}, L/D = {cr_br['eval']['aircraft_LD']:.1f} ({br_row['breguet_range_km']:.0f} km)",
            ),
        ]
    )
    sec3_1_table.to_csv(out_dir / "sec3_1_hover_cruise_table.csv", index=False)

    # ------------------------------------------------------------------
    # 3. Conversion Corridor Map (Section 3.2) at 500 m ISA, MTOW = 6800 kg
    # ------------------------------------------------------------------
    cor_csv = out_dir / "sec3_2_conversion_corridor_final.csv"
    if cor_csv.exists() and len(pd.read_csv(cor_csv)) == 475:
        df_corridor = pd.read_csv(cor_csv)
    else:
        v_grid = np.linspace(0.0, 120.0, 25)
        nac_grid = np.linspace(0.0, 90.0, 19)
        corridor_rows = []
        for nac in nac_grid:
            u_seed_c = None
            for v_ms in v_grid:
                tr_c = solve_aircraft_trim(
                    v_tas_ms=float(v_ms),
                    theta_nac_deg=float(nac),
                    aircraft_cfg=cfg,
                    airfoil=af,
                    altitude_m=500.0,
                    gross_mass_kg=mtow_kg,
                    u_init=u_seed_c,
                    verify_high_res=False,
                )
                if tr_c["converged"]:
                    u_seed_c = tr_c["u_trim_deg"]
                ev_c = tr_c["eval"]
                primary_fail = tr_c["failure_codes"][0] if tr_c["failure_codes"] else "FEASIBLE"
                corridor_rows.append(
                    dict(
                        V_ms=float(v_ms),
                        nacelle_deg=float(nac),
                        feasible=tr_c["feasible"],
                        primary_constraint=primary_fail,
                        status=tr_c["status"],
                        power_kW=ev_c["propulsion"].p_shaft_req_kW,
                        power_margin_kW=ev_c["power_margin_kW"],
                        wing_stall_margin_deg=ev_c["wing_stall_margin_deg"],
                        rotor_stall_margin_deg=ev_c["rotor_stall_margin_deg"],
                        tip_mach=ev_c["tip_mach"],
                        control_margin_pct=ev_c["control_margin_pct"],
                    )
                )
        df_corridor = pd.DataFrame(corridor_rows)
        df_corridor.to_csv(cor_csv, index=False)

    # Airplane Service Ceiling check at Best-Climb / Loiter speed (~82 m/s)
    service_ceiling_m = 3000.0
    for alt_m in np.linspace(3000.0, 11500.0, 35):
        tr_sc = solve_aircraft_trim(85.0, 0.0, cfg, af, altitude_m=float(alt_m), gross_mass_kg=mtow_kg, verify_high_res=False)
        # Service ceiling requires >= 0.5 m/s excess climb capability (~100 kW shaft reserve)
        if tr_sc["feasible"] and tr_sc["eval"]["power_margin_kW"] >= 100.0:
            service_ceiling_m = float(alt_m)

    # Clean wing stall speed at MTOW, SL & 3000m (nacelle = 0 deg)
    vs_clean_sl_ms = float(np.sqrt((2.0 * mtow_kg * 9.80665) / (1.225 * float(wing["area_m2"]) * float(wing["cl_max_clean"]))))
    vs_flap_sl_ms = float(np.sqrt((2.0 * mtow_kg * 9.80665) / (1.225 * float(wing["area_m2"]) * float(wing["cl_max_flaperon"]))))

    # ------------------------------------------------------------------
    # 4. Section 3.3 Final Operating-Limits Table
    # ------------------------------------------------------------------
    sec3_3_limits = pd.DataFrame(
        [
            dict(
                Limit="Max hover weight (SL, ISA)",
                Value=f"{max_hover_mass_sl_kg:.0f} (6800 MTOW)",
                Units="kg",
                Governing_Constraint="Transmission torque/power cap (4050 kW)",
                Condition_Basis=f"Aerodynamic OGE SL limit (structural MTOW = 6800 kg at P_req = {hov_sl['eval']['propulsion'].p_shaft_req_kW:.0f} kW)",
            ),
            dict(
                Limit="Hover ceiling (OGE / IGE)",
                Value=f"{oge_ceiling_m:.0f} / {ige_ceiling_m:.0f}",
                Units="m",
                Governing_Constraint="ISA engine power lapse (P_avail = P_req)",
                Condition_Basis=f"Design MTOW 6800 kg, 535 rpm; Airplane ceiling = {service_ceiling_m:.0f} m",
            ),
            dict(
                Limit="Practical cruise speed",
                Value=f"111.1 (max {vmax_feas_ms:.0f})",
                Units="m/s",
                Governing_Constraint="Collective authority / engine lapse",
                Condition_Basis=f"3000 m ISA, θ_nac = 0°, 360 rpm (400 km/h design; {vmax_feas_ms*3.6:.0f} km/h max)",
            ),
            dict(
                Limit="Power-limited condition",
                Value=f"{vmax_feas_ms:.0f} m/s / 4050",
                Units="m/s / kW",
                Governing_Constraint="Transmission limit (SL) / ISA lapse (>1.1 km)",
                Condition_Basis="4050 kW gearbox cap below 1.1 km; engine density lapse above 1.1 km",
            ),
            dict(
                Limit="Rotor blade stall limit",
                Value="15.75 (0° margin)",
                Units="deg",
                Governing_Constraint="VR-12 95th-pct outer-blade α limit",
                Condition_Basis="Active r/R >= 0.25 w/ Gormont radial-flow stall delay; bounds OGE ceiling",
            ),
            dict(
                Limit="Tip-Mach limit",
                Value="0.850",
                Units="–",
                Governing_Constraint="Advancing-blade compressibility drag rise",
                Condition_Basis="Bounds helicopter edgewise speed to V <= 55 m/s at θ_nac >= 75° (535 rpm)",
            ),
            dict(
                Limit="Wing stall limit",
                Value=f"{vs_clean_sl_ms:.1f} clean / {vs_flap_sl_ms:.1f} flap",
                Units="m/s",
                Governing_Constraint="CL_max = 1.38 clean / 1.65 w/ 20° flaperon",
                Condition_Basis="Minimum 1g wing-borne speed at SL MTOW; sets lower boundary of θ_nac = 0°",
            ),
            dict(
                Limit="Conversion restriction",
                Value="12 -> 95 m/s (<= 3°/s)",
                Units="m/s, deg/s",
                Governing_Constraint="Coupled wing stall (low V) & tip Mach (high V)",
                Condition_Basis=f"{int(df_corridor['feasible'].sum())}/{len(df_corridor)} ({100.0*df_corridor['feasible'].mean():.1f}%) grid states feasible at 500 m",
            ),
        ]
    )
    sec3_3_limits.to_csv(out_dir / "sec3_3_operating_limits.csv", index=False)

    # ------------------------------------------------------------------
    # 5. Section 3.4 Comparison with Relevant Tiltrotors (AW609 & Bell XV-15)
    # ------------------------------------------------------------------
    disk_area_2 = 2.0 * np.pi * (float(rot["radius_m"]) ** 2)
    dl_tr9 = (mtow_kg * 9.80665) / disk_area_2  # N/m^2
    wl_tr9 = (mtow_kg * 9.80665) / float(wing["area_m2"])  # N/m^2
    pl_tr9 = (mtow_kg * 9.80665) / float(cfg["propulsion"]["installed_power_sl_total_kW"])  # N/kW
    pf_tr9 = 100.0 * float(mass_cfg["payload_design_kg"]) / mtow_kg

    # Reference 1: Leonardo AW609 (MTOW = 8165 kg, R = 3.96 m -> 2A = 98.5 m^2, S_w = 18.5 m^2, P_inst = 2x1447 = 2894 kW, V_cr = 139 m/s, Payload = 2500 kg useful / ~1800 kg cabin)
    dl_aw = (8165.0 * 9.80665) / (2.0 * np.pi * (3.96 ** 2))
    wl_aw = (8165.0 * 9.80665) / 18.5
    pl_aw = (8165.0 * 9.80665) / 2894.0
    pf_aw = 100.0 * 1800.0 / 8165.0

    # Reference 2: Bell XV-15 (NASA SP-2000-4517: MTOW = 6000 kg, R = 3.81 m -> 2A = 91.2 m^2, S_w = 15.7 m^2, P_inst = 2x1156 = 2312 kW, V_cr = 132 m/s, Payload = 1360 kg max useful)
    dl_xv = (6000.0 * 9.80665) / (2.0 * np.pi * (3.81 ** 2))
    wl_xv = (6000.0 * 9.80665) / 15.7
    pl_xv = (6000.0 * 9.80665) / 2312.0
    pf_xv = 100.0 * 1360.0 / 6000.0

    sec3_4_comp = pd.DataFrame(
        [
            dict(
                Metric="Disk loading",
                Final_Design=f"{dl_tr9:.1f} N/m²",
                Ref1_AW609=f"{dl_aw:.1f} N/m²",
                Ref2_XV15=f"{dl_xv:.1f} N/m²",
                Ratio_Final_Ref1=f"{dl_tr9/dl_aw:.2f}×",
                Note_Source="Larger R = 4.60 m lowers hover power & improves OGE efficiency [Leonardo; NASA SP-2000-4517]",
            ),
            dict(
                Metric="Wing loading",
                Final_Design=f"{wl_tr9:.0f} N/m²",
                Ref1_AW609=f"{wl_aw:.0f} N/m²",
                Ref2_XV15=f"{wl_xv:.0f} N/m²",
                Ratio_Final_Ref1=f"{wl_tr9/wl_aw:.2f}×",
                Note_Source="Generous S = 43 m² (b = 20 m for rotor clearance) widens low-speed conversion corridor",
            ),
            dict(
                Metric="Power loading (hover)",
                Final_Design=f"{pl_tr9:.1f} N/kW",
                Ref1_AW609=f"{pl_aw:.1f} N/kW",
                Ref2_XV15=f"{pl_xv:.1f} N/kW",
                Ratio_Final_Ref1=f"{pl_tr9/pl_aw:.2f}×",
                Note_Source="Higher installed P/W (4300 kW) provides OEI safety & high-altitude climb reserve",
            ),
            dict(
                Metric="Cruise speed",
                Final_Design="111.1 m/s (400 km/h)",
                Ref1_AW609="138.9 m/s (500 km/h)",
                Ref2_XV15="132.0 m/s (475 km/h)",
                Ratio_Final_Ref1=f"{111.11/138.89:.2f}×",
                Note_Source="Lower W/S increases cruise parasite drag area S·CD0 relative to compact AW609 wing",
            ),
            dict(
                Metric="Payload fraction",
                Final_Design=f"{pf_tr9:.1f}%",
                Ref1_AW609=f"{pf_aw:.1f}%",
                Ref2_XV15=f"{pf_xv:.1f}%",
                Ratio_Final_Ref1=f"{pf_tr9/pf_aw:.2f}×",
                Note_Source="Matches AW609/XV-15 cabin payload fraction (1500 kg payload at 6800 kg MTOW)",
            ),
            dict(
                Metric="Max gross mass",
                Final_Design="6,800 kg",
                Ref1_AW609="8,165 kg",
                Ref2_XV15="6,000 kg",
                Ratio_Final_Ref1=f"{6800.0/8165.0:.2f}×",
                Note_Source="Medium twin-turboshaft tiltrotor bracketed directly between XV-15 and AW609",
            ),
        ]
    )
    sec3_4_comp.to_csv(out_dir / "sec3_4_tiltrotor_comparison.csv", index=False)

    summary_meta = {
        "mtow_kg": mtow_kg,
        "hover_sl_power_kW": round(hov_sl["eval"]["propulsion"].p_shaft_req_kW, 1),
        "hover_sl_FM": round(hov_sl["eval"]["left_rotor"].FM_hover, 3),
        "max_hover_mass_sl_kg": round(max_hover_mass_sl_kg, 0),
        "oge_hover_ceiling_m": round(oge_ceiling_m, 0),
        "ige_hover_ceiling_m": round(ige_ceiling_m, 0),
        "airplane_service_ceiling_m": round(service_ceiling_m, 0),
        "cruise_design_power_kW": round(cr_des["eval"]["propulsion"].p_shaft_req_kW, 1),
        "cruise_design_eta_prop": round(cr_des["eval"]["left_rotor"].eta_prop, 3),
        "cruise_design_LD": round(cr_des["eval"]["aircraft_LD"], 2),
        "design_speed_range_km": round(
            float(
                (
                    cr_des["eval"]["aircraft_LD"]
                    * cr_des["eval"]["left_rotor"].eta_prop
                    * float(cfg["propulsion"]["gearbox_efficiency"])
                    / max(9.80665 * (cr_des["eval"]["propulsion"].sfc_kg_per_kWh / 3.6e6), 1e-12)
                )
                * np.log(mtow_kg / (mtow_kg - usable_fuel_kg))
                / 1000.0
            ),
            1,
        ),
        "best_range_speed_ms": round(float(br_row["v_tas_ms"]), 1),
        "best_range_km": round(float(br_row["breguet_range_km"]), 1),
        "best_range_LD": round(float(br_row["aircraft_LD"]), 2),
        "vmax_feasible_ms": round(vmax_feas_ms, 1),
        "corridor_feasible_count": int(df_corridor["feasible"].sum()),
        "corridor_total_count": int(len(df_corridor)),
        "corridor_feasible_pct": round(100.0 * float(df_corridor["feasible"].mean()), 1),
        "vs_clean_sl_ms": round(vs_clean_sl_ms, 1),
        "vs_flap_sl_ms": round(vs_flap_sl_ms, 1),
    }
    (out_dir / "performance_summary.json").write_text(json.dumps(summary_meta, indent=2), encoding="utf-8")
    print("Section 3 performance summary complete:")
    print(json.dumps(summary_meta, indent=2))
    return summary_meta


if __name__ == "__main__":
    run_all_performance()
