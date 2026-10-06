# ==============================================================================
# CELL 5: TILTROTOR PRE-COMPUTED CSV DATABASE ENGINE (`tiltrotor_rotor_database.csv`)
#         6D HYPERCUBE INTERPOLATOR + SECTION 4 CONTROL-RESPONSE STUDY FROM CSV
# ==============================================================================

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator

CSV_DB_FILE = "tiltrotor_rotor_database.csv"

class TiltrotorCSVDatabaseEngine:
    """
    6-Dimensional Multilinear Interpolation Engine backed directly by `tiltrotor_rotor_database.csv`
    (5,292 pre-computed BEMT rows completed to a 5 x 5 x 4 x 7 x 3 x 3 = 6,300 6D Cartesian grid).
    Input axes:
      1) V_ms           : [0.0, 30.0, 60.0, 90.0, 120.0] m/s
      2) flow_angle_deg : [0.0, 22.5, 45.0, 67.5, 90.0] deg  (where flow_angle = 90 - (theta_nac + alpha_B))
      3) rpm            : [200.0, 300.0, 400.0, 535.0] RPM
      4) theta_0_deg    : [0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0] deg
      5) theta_1c_deg   : [-6.0, 0.0, +6.0] deg
      6) theta_1s_deg   : [-6.0, 0.0, +6.0] deg
    """
    def __init__(self, csv_path: str = CSV_DB_FILE):
        self.csv_path = csv_path
        self.raw_df = pd.read_csv(csv_path)

        # At V_ms == 0.0, flow_angle_deg is physically invariant (V_inplane = V_axial = 0).
        # Replicate V_ms == 0.0 slice across flow_angle_deg in [0, 22.5, 45, 67.5] to form a dense 6D hypercube.
        v0_slice = self.raw_df[self.raw_df["V_ms"] == 0.0].copy()
        extra_slices = []
        for fa in [0.0, 22.5, 45.0, 67.5]:
            s = v0_slice.copy()
            s["flow_angle_deg"] = fa
            extra_slices.append(s)
        self.full_df = pd.concat([self.raw_df] + extra_slices, ignore_index=True)

        self.axis_cols = ["V_ms", "flow_angle_deg", "rpm", "theta_0_deg", "theta_1c_deg", "theta_1s_deg"]
        self.full_df = self.full_df.sort_values(self.axis_cols).reset_index(drop=True)

        self.axes = [np.sort(self.full_df[c].unique()).astype(float) for c in self.axis_cols]
        self.grid_shape = tuple(len(a) for a in self.axes)

        self.output_cols = [
            "Fx_shaft_N", "Fy_shaft_N", "Fz_shaft_N",
            "Mx_shaft_Nm", "My_shaft_Nm", "Mz_shaft_Nm",
            "torque_Nm", "power_kW", "CT", "CP", "CT_disk", "CQ_disk",
            "mu_advance", "lambda_G", "vi_glauert_ms",
            "max_Mach", "advancing_tip_Mach", "max_alpha_deg", "min_alpha_deg",
            "stall_fraction", "reverse_flow_fraction"
        ]
        # Stack all 21 output channels into a single (5, 5, 4, 7, 3, 3, 21) tensor for 1-call vectorized interpolation!
        tensor_data = np.stack(
            [self.full_df[col].values.astype(float).reshape(self.grid_shape) for col in self.output_cols],
            axis=-1
        )
        self.vector_interp = RegularGridInterpolator(
            self.axes, tensor_data, method="linear", bounds_error=False, fill_value=None
        )
        self.col_idx = {col: i for i, col in enumerate(self.output_cols)}
        print(f"[CSV DATABASE LOADED] '{csv_path}' -> {len(self.raw_df):,} raw rows ({len(self.full_df):,} 6D grid nodes, {len(self.output_cols)} channels).")
        for col, ax_vals in zip(self.axis_cols, self.axes):
            print(f"  Axis {col:16s}: {ax_vals.tolist()}")

    def query_rotor(
        self,
        v_inf_ms: float,
        theta_nac_deg: float,
        rpm: float,
        theta_0_deg: float,
        theta_1c_deg: float = 0.0,
        theta_1s_deg: float = 0.0,
        alpha_body_deg: float = 0.0,
        rotation_dir: int = 1,  # +1 = CCW (Port Rotor), -1 = CW (Starboard Rotor)
        r_hub_body: np.ndarray = None,
    ) -> dict:
        """
        Query single-rotor forces, moments, power, inflow, Mach, stall, and reverse-flow metrics
        directly from `tiltrotor_rotor_database.csv` and transform to Aircraft Body Axes about CG.
        """
        if r_hub_body is None:
            r_hub_body = np.array([0.0, 0.0, 0.0])

        # Map (theta_nac_deg, alpha_body_deg) to CSV flow_angle_deg:
        # In CSV: flow_angle_deg = 0 deg is pure edgewise (theta_nac = 90 deg), 90 deg is pure axial (theta_nac = 0 deg)
        flow_angle_deg = float(np.clip(90.0 - (theta_nac_deg + alpha_body_deg), 0.0, 90.0))

        v_q   = float(np.clip(v_inf_ms, 0.0, 120.0))
        rpm_q = float(np.clip(rpm, 200.0, 535.0))
        th0_q = float(np.clip(theta_0_deg, 0.0, 24.0))
        # For CW starboard rotor (rotation_dir = -1), lateral cyclic sign flips symmetrically
        th1c_eff = float(np.clip(rotation_dir * theta_1c_deg, -6.0, 6.0))
        th1s_q   = float(np.clip(theta_1s_deg, -6.0, 6.0))

        pt = np.array([[v_q, flow_angle_deg, rpm_q, th0_q, th1c_eff, th1s_q]])
        vec = self.vector_interp(pt)[0]

        # Extract interpolated channels + linear extrapolation slope if collective exceeds 24 deg in high-speed cruise
        d_th0_extrap = max(0.0, theta_0_deg - 24.0)
        if d_th0_extrap > 0.0:
            pt_20 = np.array([[v_q, flow_angle_deg, rpm_q, 20.0, th1c_eff, th1s_q]])
            vec_20 = self.vector_interp(pt_20)[0]
            slope_th0 = (vec - vec_20) / 4.0
            vec = vec + slope_th0 * d_th0_extrap

        Fx_csv = float(vec[self.col_idx["Fx_shaft_N"]])      # + forward in disk plane
        Fy_csv = float(vec[self.col_idx["Fy_shaft_N"]]) * rotation_dir
        Fz_csv = float(vec[self.col_idx["Fz_shaft_N"]])      # + thrust along shaft axis
        Mx_csv = float(vec[self.col_idx["Mx_shaft_Nm"]]) * rotation_dir
        My_csv = float(vec[self.col_idx["My_shaft_Nm"]])
        Mz_csv = float(vec[self.col_idx["Mz_shaft_Nm"]]) * rotation_dir

        # Convert from CSV shaft axes (where +Fx_csv is forward in-plane and +Fz_csv is thrust)
        # to our F_S convention (where +x_s is aft H-force = -Fx_csv and +z_s is thrust = +Fz_csv):
        F_shaft = np.array([-Fx_csv, Fy_csv, Fz_csv])
        M_shaft = np.array([Mx_csv, My_csv, Mz_csv])
        F_body, M_body, _ = shaft_to_body_transform(F_shaft, M_shaft, theta_nac_deg, r_hub_body)

        max_alpha = float(vec[self.col_idx["max_alpha_deg"]])
        stall_frac = float(np.clip(vec[self.col_idx["stall_fraction"]], 0.0, 1.0))
        stall_margin_deg = 15.8 - max_alpha * (0.65 + 0.35 * stall_frac)

        return {
            "thrust_N": Fz_csv,
            "fx_shaft_N": Fx_csv,
            "fy_shaft_N": Fy_csv,
            "h_force_N": -Fx_csv,
            "mx_hub_Nm": Mx_csv,
            "my_hub_Nm": My_csv,
            "mz_hub_Nm": Mz_csv,
            "torque_Nm": float(vec[self.col_idx["torque_Nm"]]),
            "power_kW": max(5.0, float(vec[self.col_idx["power_kW"]])),
            "CT": float(vec[self.col_idx["CT"]]),
            "CP": float(vec[self.col_idx["CP"]]),
            "mu_advance": float(vec[self.col_idx["mu_advance"]]),
            "lambda_G": float(vec[self.col_idx["lambda_G"]]),
            "vi_glauert_ms": float(vec[self.col_idx["vi_glauert_ms"]]),
            "max_tip_mach": float(vec[self.col_idx["advancing_tip_Mach"]]),
            "max_alpha_deg": max_alpha,
            "stall_fraction": stall_frac,
            "stall_margin_deg": stall_margin_deg,
            "reverse_flow_fraction": float(np.clip(vec[self.col_idx["reverse_flow_fraction"]], 0.0, 1.0)),
            "F_body_N": F_body,
            "M_body_Nm": M_body,
        }


# Instantiate the Global CSV Database Lookup Engine
CSV_ROTOR_DB = TiltrotorCSVDatabaseEngine("tiltrotor_rotor_database.csv")


def run_section4_control_response_from_csv(show_plots: bool = True):
    """
    Executes Section 4.1 (Collective Sweep), 4.2 (Longitudinal Cyclic Sweep), and 4.3 (Lateral Cyclic Sweep)
    directly by querying `CSV_ROTOR_DB` (`tiltrotor_rotor_database.csv`).
    """
    b_wing = float(SIZED_VEHICLE.get("Wingspan", 20.0))
    r_hub_port = np.array([0.0, -0.5 * b_wing, -0.80])

    # Two conversion states directly aligned with `tiltrotor_rotor_database.csv` grid:
    # Condition A: Helicopter-Like Edgewise Flight (V = 30 m/s, theta_nac = 90 deg -> flow_angle = 0 deg, RPM = 535)
    # Condition B: Intermediate Conversion Flight  (V = 60 m/s, theta_nac = 45 deg -> flow_angle = 45 deg, RPM = 400)
    conditions = [
        {"name": "Helicopter Edgewise (V=30 m/s, θ_nac=90°, RPM=535)", "V": 30.0, "tn": 90.0, "rpm": 535.0, "th0_nom": 12.0, "col": "#0d6efd", "ls": "-"},
        {"name": "Intermediate Conversion (V=60 m/s, θ_nac=45°, RPM=400)", "V": 60.0, "tn": 45.0, "rpm": 400.0, "th0_nom": 16.0, "col": "#dc3545", "ls": "--"},
    ]

    th0_sweep = np.linspace(0.0, 24.0, 13)
    th1s_sweep = np.linspace(-6.0, 6.0, 13)
    th1c_sweep = np.linspace(-6.0, 6.0, 13)

    results_41, results_42, results_43 = {}, {}, {}
    for cond in conditions:
        cname = cond["name"]
        rows_41, rows_42, rows_43 = [], [], []
        for th0 in th0_sweep:
            r = CSV_ROTOR_DB.query_rotor(cond["V"], cond["tn"], cond["rpm"], th0, 0.0, 0.0, rotation_dir=1, r_hub_body=r_hub_port)
            rows_41.append({"val": th0, "FX": r["F_body_N"][0]*1e-3, "FY": r["F_body_N"][1]*1e-3, "FZ": r["F_body_N"][2]*1e-3, "MX": r["M_body_Nm"][0]*1e-3, "MY": r["M_body_Nm"][1]*1e-3, "MZ": r["M_body_Nm"][2]*1e-3, "P_kW": r["power_kW"], "stall_pct": 100.0 * r["stall_fraction"]})
        for th1s in th1s_sweep:
            r = CSV_ROTOR_DB.query_rotor(cond["V"], cond["tn"], cond["rpm"], cond["th0_nom"], 0.0, th1s, rotation_dir=1, r_hub_body=r_hub_port)
            rows_42.append({"val": th1s, "FX": r["F_body_N"][0]*1e-3, "FY": r["F_body_N"][1]*1e-3, "FZ": r["F_body_N"][2]*1e-3, "MX": r["M_body_Nm"][0]*1e-3, "MY": r["M_body_Nm"][1]*1e-3, "MZ": r["M_body_Nm"][2]*1e-3, "P_kW": r["power_kW"], "stall_pct": 100.0 * r["stall_fraction"]})
        for th1c in th1c_sweep:
            r = CSV_ROTOR_DB.query_rotor(cond["V"], cond["tn"], cond["rpm"], cond["th0_nom"], th1c, 0.0, rotation_dir=1, r_hub_body=r_hub_port)
            rows_43.append({"val": th1c, "FX": r["F_body_N"][0]*1e-3, "FY": r["F_body_N"][1]*1e-3, "FZ": r["F_body_N"][2]*1e-3, "MX": r["M_body_Nm"][0]*1e-3, "MY": r["M_body_Nm"][1]*1e-3, "MZ": r["M_body_Nm"][2]*1e-3, "P_kW": r["power_kW"], "stall_pct": 100.0 * r["stall_fraction"]})
        results_41[cname] = pd.DataFrame(rows_41)
        results_42[cname] = pd.DataFrame(rows_42)
        results_43[cname] = pd.DataFrame(rows_43)

    if show_plots:
        fig, axes = plt.subplots(3, 4, figsize=(18.5, 12.5), dpi=140)
        fig.suptitle(
            "Milestone 2 — Section 4: Single-Rotor Control-Response Sweeps (Directly Interpolated from `tiltrotor_rotor_database.csv`)\n"
            "Blue Solid: Helicopter Edgewise (V=30 m/s, θ_nac=90°, RPM=535)  |  Red Dashed: Intermediate Conversion (V=60 m/s, θ_nac=45°, RPM=400)",
            fontsize=13.0, fontweight="bold", color="#162d4c", y=0.99
        )
        sweep_configs = [
            (0, results_41, "Collective Pitch θ_0 [deg]", "4.1 Collective Sweep"),
            (1, results_42, "Longitudinal Cyclic θ_1s [deg]", "4.2 Longitudinal Cyclic Sweep"),
            (2, results_43, "Lateral Cyclic θ_1c [deg]", "4.3 Lateral Cyclic Sweep"),
        ]
        for row_idx, res_dict, xlabel, row_title in sweep_configs:
            ax_f, ax_m, ax_p, ax_s = axes[row_idx]
            for cond in conditions:
                df = res_dict[cond["name"]]
                ls = cond["ls"]
                ax_f.plot(df["val"], df["FX"], ls=ls, color="#0d6efd", lw=2.0, label="F_X (Forward)" if ls=="-" else "_nolegend_")
                ax_f.plot(df["val"], df["FY"], ls=ls, color="#198754", lw=1.8, label="F_Y (Side)" if ls=="-" else "_nolegend_")
                ax_f.plot(df["val"], df["FZ"], ls=ls, color="#dc3545", lw=2.2, label="F_Z (Vertical)" if ls=="-" else "_nolegend_")
                ax_m.plot(df["val"], df["MX"], ls=ls, color="#6f42c1", lw=2.0, label="M_X (Roll)" if ls=="-" else "_nolegend_")
                ax_m.plot(df["val"], df["MY"], ls=ls, color="#fd7e14", lw=2.0, label="M_Y (Pitch)" if ls=="-" else "_nolegend_")
                ax_m.plot(df["val"], df["MZ"], ls=ls, color="#20c997", lw=1.8, label="M_Z (Yaw)" if ls=="-" else "_nolegend_")
                ax_p.plot(df["val"], df["P_kW"], ls=ls, color=cond["col"], lw=2.2, label=cond["name"][:26])
                ax_s.plot(df["val"], 100.0 - df["stall_pct"], ls=ls, color=cond["col"], lw=2.2, label=cond["name"][:26])
            ax_f.set_title(f"{row_title}: Body Forces [kN]", fontsize=10, fontweight="bold"); ax_f.set_xlabel(xlabel); ax_f.set_ylabel("Force [kN]"); ax_f.grid(True, alpha=0.35); ax_f.legend(fontsize=8)
            ax_m.set_title(f"{row_title}: Body Moments about CG [kN·m]", fontsize=10, fontweight="bold"); ax_m.set_xlabel(xlabel); ax_m.set_ylabel("Moment [kN·m]"); ax_m.grid(True, alpha=0.35); ax_m.legend(fontsize=8)
            ax_p.set_title(f"{row_title}: Rotor Shaft Power [kW]", fontsize=10, fontweight="bold"); ax_p.set_xlabel(xlabel); ax_p.set_ylabel("Shaft Power [kW]"); ax_p.grid(True, alpha=0.35); ax_p.legend(fontsize=7.5)
            ax_s.axhline(85.0, color="black", ls=":", lw=1.4, label="Unstalled Limit (85%)")
            ax_s.set_title(f"{row_title}: Unstalled Disk Area [100 - Stall %]", fontsize=10, fontweight="bold"); ax_s.set_xlabel(xlabel); ax_s.set_ylabel("Unstalled Disk [%]"); ax_s.grid(True, alpha=0.35); ax_s.legend(fontsize=7.5)
        plt.tight_layout(rect=[0, 0, 1, 0.94])
        plt.show()
    return {"4.1": results_41, "4.2": results_42, "4.3": results_43}

SEC4_RESULTS = run_section4_control_response_from_csv(show_plots=True)

