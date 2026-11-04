"""Milestone 1 & 2 Rotor Aeromechanics Module: Airfoil Polars, Axial BEMT & Edgewise BEMT w/ Gormont Radial Flow."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
import glob
import os
import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator
from scipy.optimize import brentq


def _find_airfoils_dir() -> Path:
    candidates = [
        Path("airfoils"),
        Path.cwd().parent / "airfoils",
    ]
    if "__file__" in globals():
        candidates.insert(1, Path(__file__).resolve().parents[2] / "airfoils")
    for c in candidates:
        if c.exists() and c.is_dir():
            return c
    return Path("airfoils")


def to_airfoiltools_slug(name: str) -> str:
    cleaned = name.lower().replace(" ", "").replace("-", "").replace("_", "")
    if "vr12" in cleaned or "boeing" in cleaned:
        return "vr12-il"
    if "0012" in cleaned or "naca0012" in cleaned:
        return "n0012-il"
    if "sc1095" in cleaned:
        return "sc1095-il"
    return cleaned + "-il"


def extract_airfoil_thickness_ratio(airfoil_name: str) -> float:
    slug = to_airfoiltools_slug(airfoil_name).replace("-il", "")
    af_dir = _find_airfoils_dir()
    for fpath in glob.glob(str(af_dir / "**" / "*.dat"), recursive=True):
        if slug in os.path.basename(fpath).lower():
            coords = []
            with open(fpath, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 2:
                        try:
                            coords.append([float(parts[0]), float(parts[1])])
                        except ValueError:
                            pass
            if len(coords) > 10:
                arr = np.array(coords)
                min_x_idx = int(np.argmin(arr[:, 0]))
                upper = arr[: min_x_idx + 1]
                lower = arr[min_x_idx:]
                upper = upper[np.argsort(upper[:, 0])]
                lower = lower[np.argsort(lower[:, 0])]
                x_common = np.linspace(0.01, 0.99, 200)
                y_u = np.interp(x_common, upper[:, 0], upper[:, 1])
                y_l = np.interp(x_common, lower[:, 0], lower[:, 1])
                return float(np.max(y_u - y_l))
    cleaned = airfoil_name.lower()
    if "vr12" in cleaned or "vr-12" in cleaned:
        return 0.1056
    if "sc1095" in cleaned:
        return 0.0950
    return 0.1200


class AirfoilModel:
    """Multi-Reynolds airfoil polar table interpolator with Prandtl-Glauert & Viterna-Corrigan post-stall."""

    def __init__(self, airfoil_name: str = "Boeing-Vertol VR-12", ncrit: int = 9):
        self.airfoil_name = airfoil_name
        self.ncrit = ncrit
        self.has_polars = False
        self.re_list: list[float] = []
        self.raw_data: Dict[float, pd.DataFrame] = {}
        self.alpha_stall_deg = 15.5
        self.cl_max = 1.49
        self.thickness_ratio = extract_airfoil_thickness_ratio(airfoil_name)
        self.dcl_dalpha_linear = 2.0 * np.pi
        self._build_polar_interpolators()

    def _build_polar_interpolators(self) -> None:
        slug = to_airfoiltools_slug(self.airfoil_name).replace("-il", "")
        af_dir = _find_airfoils_dir()
        for fpath in glob.glob(str(af_dir / "**" / "*.csv"), recursive=True):
            if slug in os.path.basename(fpath).lower():
                try:
                    df = pd.read_csv(fpath, skiprows=10)
                    df.columns = [c.strip().lower() for c in df.columns]
                    if {"alpha", "cl", "cd"}.issubset(set(df.columns)):
                        re_val = float(
                            os.path.basename(fpath).split("-")[-1].replace(".csv", "").replace("n5", "")
                        )
                        self.raw_data[re_val] = df[["alpha", "cl", "cd"]].rename(
                            columns={"cl": "CL", "cd": "CD"}
                        )
                except Exception:
                    pass
        if self.raw_data:
            self.re_list = sorted(self.raw_data.keys())
            ref_re = min(self.re_list, key=lambda x: abs(x - 500000.0))
            ref_df = self.raw_data[ref_re]
            idx_cl_max = ref_df["CL"].astype(float).idxmax()
            self.alpha_stall_deg = float(ref_df.loc[idx_cl_max, "alpha"])
            self.cl_max = float(ref_df.loc[idx_cl_max, "CL"])

            alpha_common = np.linspace(np.radians(-15.0), np.radians(20.0), 71)
            cl_grid = np.zeros((len(alpha_common), len(self.re_list)))
            cd_grid = np.zeros((len(alpha_common), len(self.re_list)))
            for j, re_v in enumerate(self.re_list):
                df = self.raw_data[re_v]
                a_rad = np.radians(df["alpha"].values.astype(float))
                cl_v = df["CL"].values.astype(float)
                cd_v = df["CD"].values.astype(float)
                cl_grid[:, j] = np.interp(alpha_common, a_rad, cl_v)
                cd_grid[:, j] = np.interp(alpha_common, a_rad, cd_v)
            self._interp_cl = RegularGridInterpolator(
                (alpha_common, np.log10(self.re_list)), cl_grid, bounds_error=False, fill_value=None
            )
            self._interp_cd = RegularGridInterpolator(
                (alpha_common, np.log10(self.re_list)), cd_grid, bounds_error=False, fill_value=None
            )
            self.has_polars = True
            a_p, a_m = np.radians(2.0), np.radians(-2.0)
            log_re_mid = np.log10(500000.0)
            cl_p = float(self._interp_cl(np.array([[a_p, log_re_mid]]))[0])
            cl_m = float(self._interp_cl(np.array([[a_m, log_re_mid]]))[0])
            self.dcl_dalpha_linear = (cl_p - cl_m) / (a_p - a_m)

    def evaluate(
        self, alpha_rad: np.ndarray, re_arr: np.ndarray, mach_arr: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        alpha_flat = np.asarray(alpha_rad, dtype=float).ravel()
        re_flat = np.asarray(re_arr, dtype=float).ravel()
        mach_flat = np.asarray(mach_arr, dtype=float).ravel()

        if self.has_polars:
            re_clamped = np.clip(re_flat, 5e4, 1e6)
            log_re = np.log10(re_clamped)
            pts = np.column_stack((np.clip(alpha_flat, np.radians(-15.0), np.radians(20.0)), log_re))
            cl_tab = self._interp_cl(pts)
            cd_tab = self._interp_cd(pts)
        else:
            a_st = np.radians(self.alpha_stall_deg)
            cl_tab = np.clip(self.dcl_dalpha_linear * alpha_flat, -1.25, self.cl_max)
            cd_tab = 0.0085 + 0.012 * (alpha_flat / max(a_st, 0.1)) ** 2

        # Smooth Viterna-Corrigan post-stall blending for |alpha| > 18.5 deg
        abs_a = np.abs(alpha_flat)
        a_blend_start = np.radians(18.5)
        high_mask = abs_a > a_blend_start
        if np.any(high_mask):
            sgn = np.sign(alpha_flat[high_mask])
            aa = np.clip(abs_a[high_mask], a_blend_start, np.radians(89.0))
            cd_max = 1.98
            cl_vc = sgn * (0.95 * np.sin(2.0 * aa) + 0.25 * (np.cos(aa) ** 2) / np.maximum(np.sin(aa), 0.1))
            cd_vc = 0.02 + cd_max * (np.sin(aa) ** 2)
            w = np.clip((aa - a_blend_start) / np.radians(6.0), 0.0, 1.0)
            cl_tab[high_mask] = (1.0 - w) * cl_tab[high_mask] + w * cl_vc
            cd_tab[high_mask] = (1.0 - w) * cd_tab[high_mask] + w * cd_vc

        prandtl = 1.0 / np.sqrt(np.maximum(1.0 - np.clip(mach_flat, 0.0, 0.72) ** 2, 0.2))
        cl_comp = np.where(abs_a <= np.radians(18.0), cl_tab * prandtl, cl_tab)
        dm = np.maximum(0.0, mach_flat - 0.70)
        cd_comp = cd_tab + 20.0 * (dm ** 4)
        valid = (alpha_flat >= np.radians(-12.0)) & (alpha_flat <= np.radians(18.0)) & (mach_flat <= 0.85)
        return cl_comp, cd_comp, valid


def euler_body_from_inertial(phi_rad: float, theta_rad: float, psi_rad: float = 0.0) -> np.ndarray:
    cp, sp = np.cos(phi_rad), np.sin(phi_rad)
    ct, st = np.cos(theta_rad), np.sin(theta_rad)
    cy, sy = np.cos(psi_rad), np.sin(psi_rad)
    return np.array(
        [
            [ct * cy, ct * sy, -st],
            [sp * st * cy - cp * sy, sp * st * sy + cp * cy, sp * ct],
            [cp * st * cy + sp * sy, cp * st * sy - sp * cy, cp * ct],
        ]
    )


def shaft_to_body_transform(
    F_shaft: np.ndarray, M_shaft: np.ndarray, theta_nac_deg: float, r_hub_body: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform shaft forces/moments (+z_s along thrust) to body frame (+x fwd, +y stbd, +z down)."""
    tn = np.radians(theta_nac_deg)
    R_BS = np.array(
        [
            [-np.sin(tn), 0.0, np.cos(tn)],
            [0.0, 1.0, 0.0],
            [np.cos(tn), 0.0, -np.sin(tn)],
        ]
    )
    F_body = R_BS @ F_shaft
    M_hub_body = R_BS @ M_shaft
    M_cg_body = M_hub_body + np.cross(r_hub_body, F_body)
    return F_body, M_cg_body, R_BS


@dataclass
class EdgewiseRotorResult:
    thrust_N: float
    h_force_N: float
    y_force_N: float
    mx_hub_Nm: float
    my_hub_Nm: float
    torque_Nm: float
    mz_hub_Nm: float
    power_kW: float
    F_body_N: np.ndarray
    M_body_Nm: np.ndarray
    CT: float
    CP: float
    FM_hover: float
    eta_prop: float
    mu_edge: float
    mu_axial: float
    lambda_G: float
    lambda_i_mean: float
    kx_inflow: float
    max_tip_mach: float
    min_ut_ms: float
    reverse_flow_area_frac: float
    stall_area_frac: float
    stall_margin_deg: float
    valid_fraction: float
    beta_flap_deg: Tuple[float, float, float]


def apply_gormont_radial_flow_correction(
    alpha_rad: np.ndarray,
    re_arr: np.ndarray,
    UT_grid: np.ndarray,
    UR_grid: np.ndarray,
    CL_2D: np.ndarray,
    CD_2D: np.ndarray,
    t_over_c: float,
    airfoil: AirfoilModel,
    rho: float,
    CHORD_grid: np.ndarray,
    min_ut_ms: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Gormont (1973) / Harris radial-flow stall-delay & yawed skin-friction model (USAAMRDL TR 72-67)."""
    U_TOT_grid = np.sqrt(UT_grid ** 2 + UR_grid ** 2)
    invalid_mask = UT_grid < min_ut_ms
    UT_safe = np.where(invalid_mask, 1.0, UT_grid)
    A_rad = np.arctan2(UR_grid, UT_safe)
    cos_A = np.cos(A_rad)
    sec_A = np.where(invalid_mask, 1.0, 1.0 / np.maximum(cos_A, 1e-6))

    re_safe = np.maximum(re_arr, 1.0e4)
    CD_SF = (0.088 / (re_safe ** (1.0 / 6.0))) * (1.0 + 2.0 * t_over_c)
    CD_corrected = np.where(invalid_mask, CD_2D, CD_2D + CD_SF * (sec_A - 1.0))
    D_radial = np.where(invalid_mask, 0.0, 0.5 * rho * CHORD_grid * U_TOT_grid * UR_grid * CD_SF)

    alpha_stall_rad = np.radians(airfoil.alpha_stall_deg)
    stall_mask = (alpha_rad > alpha_stall_rad) & (~invalid_mask)
    CL_yawed = CL_2D / np.maximum(cos_A, 0.1)
    CL_linear_cap = airfoil.dcl_dalpha_linear * alpha_rad
    CL_corrected = np.where(stall_mask, np.minimum(CL_yawed, CL_linear_cap), CL_2D)
    return CL_corrected, CD_corrected, D_radial


def run_edgewise_bemt(
    radius: float,
    root_cutout: float,
    num_blades: int,
    c_root: float,
    taper: float,
    collective_deg: float,
    twist_deg: float,
    rpm: float,
    v_inf_ms: float,
    theta_nac_deg: float,
    airfoil: AirfoilModel,
    cyclic_1c_deg: float = 0.0,
    cyclic_1s_deg: float = 0.0,
    alpha_body_deg: float = 0.0,
    rho: float = 1.225,
    a_sound: float = 340.3,
    mu_visc: float = 1.789e-5,
    num_radial: int = 24,
    num_azimuth: int = 48,
    rotation_dir: int = 1,
    r_hub_body: Optional[np.ndarray] = None,
    flapping_mode: str = "steady_1st_harmonic",
    spanwise_flow_correction: bool = True,
) -> EdgewiseRotorResult:
    """Unified Axial + Edgewise BEMT rotor solver (Milestones 1 & 2)."""
    if r_hub_body is None:
        r_hub_body = np.zeros(3)

    r_edges = np.linspace(root_cutout, radius, num_radial + 1)
    r_stations = 0.5 * (r_edges[:-1] + r_edges[1:])
    dr = np.diff(r_edges)
    r_norm = r_stations / radius
    chords = c_root + (c_root * taper - c_root) * (
        (r_stations - root_cutout) / max(1e-4, radius - root_cutout)
    )
    sigma_r = (num_blades * chords) / (np.pi * radius)

    psi_rad = np.linspace(0.0, 2.0 * np.pi, num_azimuth, endpoint=False)
    R_grid, PSI_grid = np.meshgrid(r_stations, psi_rad, indexing="ij")
    RNORM_grid = R_grid / radius
    CHORD_grid = chords[:, None]
    DR_grid = dr[:, None]

    alpha_b_rad = np.radians(alpha_body_deg)
    tn_rad = np.radians(theta_nac_deg)
    v_axial_ms = v_inf_ms * (np.cos(alpha_b_rad) * np.cos(tn_rad) - np.sin(alpha_b_rad) * np.sin(tn_rad))
    v_edge_ms = v_inf_ms * (np.cos(alpha_b_rad) * np.sin(tn_rad) + np.sin(alpha_b_rad) * np.cos(tn_rad))

    omega = rpm * 2.0 * np.pi / 60.0
    v_tip = max(omega * radius, 1e-3)
    mu_edge = v_edge_ms / v_tip
    mu_axial = v_axial_ms / v_tip

    theta_base_r = np.radians(collective_deg + twist_deg * (r_norm - 0.75))
    mach_base_r = np.sqrt((omega * r_stations) ** 2 + v_axial_ms ** 2 + v_edge_ms ** 2) / a_sound
    lambda_tot_r = np.full_like(r_stations, max(mu_axial, 1e-4) + 0.025)
    relax = 0.25

    for _ in range(85):
        u_t_0 = omega * r_stations
        u_p_0 = lambda_tot_r * v_tip
        phi_0 = np.arctan2(u_p_0, np.maximum(u_t_0, 1e-4))
        sin_phi = np.maximum(np.abs(np.sin(phi_0)), 1e-3)
        f_tip = (num_blades / 2.0) * (1.0 - r_norm) / np.maximum(r_norm * sin_phi, 1e-4)
        f_root = (num_blades / 2.0) * (r_norm - root_cutout / radius) / np.maximum(r_norm * sin_phi, 1e-4)
        F_tip = (2.0 / np.pi) * np.arccos(np.exp(-np.clip(f_tip, 0.0, 30.0)))
        F_root = (2.0 / np.pi) * np.arccos(np.exp(-np.clip(f_root, 0.0, 30.0)))
        f_loss = np.maximum(F_tip * F_root, 1e-3)

        alpha_0 = theta_base_r - phi_0
        u_res_sq_0 = u_t_0 ** 2 + u_p_0 ** 2
        re_0 = (rho * np.sqrt(u_res_sq_0) * chords) / max(mu_visc, 1e-6)
        cl_0, cd_0, _ = airfoil.evaluate(alpha_0, re_0, mach_base_r)
        C_y_0 = cl_0 * np.cos(phi_0) - cd_0 * np.sin(phi_0)
        K_0 = (sigma_r / (8.0 * f_loss * np.maximum(r_norm, 0.05))) * (u_res_sq_0 / (v_tip ** 2)) * C_y_0
        K_pos = np.maximum(K_0, 0.0)
        lam_i = np.maximum(lambda_tot_r - mu_axial, 1e-4)
        for _ in range(4):
            lt = mu_axial + lam_i
            v_gl = np.sqrt(mu_edge ** 2 + lt ** 2)
            g_val = lam_i * v_gl - K_pos
            dg = np.maximum(v_gl + lam_i * np.maximum(lt, 0.0) / np.maximum(v_gl, 1e-6), 1e-5)
            lam_i = np.maximum(0.0, lam_i - g_val / dg)
        lambda_i_new = lam_i
        lambda_tot_new = np.clip(mu_axial + lambda_i_new, 1e-4, 3.5)
        if np.max(np.abs(lambda_tot_new - lambda_tot_r)) < 1.0e-8:
            lambda_tot_r = lambda_tot_new
            break
        lambda_tot_r = (1.0 - relax) * lambda_tot_r + relax * lambda_tot_new

    lambda_i_glauert_r = lambda_tot_r - mu_axial
    lambda_i_mean = float(np.average(np.maximum(0.0, lambda_i_glauert_r), weights=r_stations))
    lambda_G = float(np.sqrt(mu_edge ** 2 + (mu_axial + lambda_i_mean) ** 2))
    if abs(mu_edge) > 1e-6 and lambda_G > 1e-6:
        ratio_mu_lg = abs(mu_edge) / lambda_G
        kx_inflow = ((4.0 / 3.0) * ratio_mu_lg) / (1.2 + ratio_mu_lg)
    else:
        kx_inflow = 0.0

    lambda_i_grid = np.maximum(
        -mu_axial + 1e-4, lambda_i_glauert_r[:, None] * (1.0 + kx_inflow * RNORM_grid * np.cos(PSI_grid))
    )

    th_1c_rad = np.radians(cyclic_1c_deg)
    th_1s_rad = np.radians(cyclic_1s_deg)
    if flapping_mode == "steady_1st_harmonic":
        mu_blend = float(np.clip(abs(mu_edge) / 0.08, 0.0, 1.0))
        beta_0_rad = 0.030 * (lambda_i_mean * 10.0) * mu_blend
        # Gimbaled proprotor hub w/ hub spring (nu_beta > 1): smooth partial flapping follow-up
        beta_1c_rad = -0.35 * th_1s_rad + 0.14 * mu_edge * np.radians(max(collective_deg, 0.0))
        beta_1s_rad = 0.35 * th_1c_rad - 0.08 * mu_edge * lambda_i_mean
    else:
        beta_0_rad, beta_1c_rad, beta_1s_rad = 0.0, 0.0, 0.0

    beta_grid = beta_0_rad + beta_1c_rad * np.cos(PSI_grid) + beta_1s_rad * np.sin(PSI_grid)
    dbeta_dt_grid = omega * (-beta_1c_rad * np.sin(PSI_grid) + beta_1s_rad * np.cos(PSI_grid))

    theta_grid = (
        np.radians(collective_deg)
        + np.radians(twist_deg) * (RNORM_grid - 0.75)
        + th_1c_rad * np.cos(PSI_grid)
        + th_1s_rad * np.sin(PSI_grid)
    )

    UT_grid = omega * R_grid + v_edge_ms * np.sin(PSI_grid)
    UP_grid = v_axial_ms + lambda_i_grid * v_tip + R_grid * dbeta_dt_grid + v_edge_ms * beta_grid * np.cos(PSI_grid)
    UR_grid = -v_edge_ms * np.cos(PSI_grid)

    rev_flow_mask = UT_grid < 0.0
    U_res_sq = UT_grid ** 2 + UP_grid ** 2
    U_res = np.sqrt(U_res_sq)
    Mach_grid = np.sqrt(UT_grid ** 2 + v_axial_ms ** 2) / a_sound
    Re_grid = (rho * U_res * CHORD_grid) / max(mu_visc, 1e-6)

    phi_normal = np.arctan2(UP_grid, np.maximum(np.abs(UT_grid), 1e-4))
    alpha_raw = np.where(~rev_flow_mask, theta_grid - phi_normal, -(theta_grid + phi_normal))
    alpha_wrapped = np.clip(alpha_raw, np.radians(-45.0), np.radians(45.0))

    cl_flat, cd_flat, valid_flat = airfoil.evaluate(
        alpha_wrapped.ravel(), Re_grid.ravel(), Mach_grid.ravel()
    )
    CL_grid = cl_flat.reshape(R_grid.shape)
    CD_grid = cd_flat.reshape(R_grid.shape)
    valid_mask_grid = valid_flat.reshape(R_grid.shape)

    CL_grid = np.where(rev_flow_mask, -0.65 * CL_grid, CL_grid)
    CD_grid = np.where(rev_flow_mask, CD_grid + 0.045 + 0.5 * (np.sin(alpha_wrapped) ** 2), CD_grid)

    if abs(mu_edge) > 1e-6:
        m_excess = np.maximum(0.0, Mach_grid - 0.78)
        CD_grid = CD_grid + 18.0 * (m_excess ** 3)

    if spanwise_flow_correction and abs(mu_edge) > 1e-6:
        CL_grid, CD_grid, D_radial_grid = apply_gormont_radial_flow_correction(
            alpha_wrapped, Re_grid, UT_grid, UR_grid, CL_grid, CD_grid,
            airfoil.thickness_ratio, airfoil, rho, CHORD_grid
        )
    else:
        D_radial_grid = np.zeros_like(CL_grid)

    sgn_ut = np.where(rev_flow_mask, -1.0, 1.0)
    q_dyn = 0.5 * rho * U_res_sq * CHORD_grid

    dFz_aero = q_dyn * (CL_grid * np.cos(phi_normal) - CD_grid * np.sin(phi_normal))
    dFpsi_dr_grid = q_dyn * (sgn_ut * CD_grid * np.cos(phi_normal) + CL_grid * np.sin(phi_normal))
    dFz_dr_grid = dFz_aero - D_radial_grid * np.sin(beta_grid) if spanwise_flow_correction else dFz_aero

    az_weight = float(num_blades) / float(num_azimuth)
    thrust_N = float(az_weight * np.sum(dFz_dr_grid * DR_grid))
    torque_Nm = float(az_weight * np.sum(R_grid * dFpsi_dr_grid * DR_grid))
    power_W = max(omega * torque_Nm, 1.0)

    dHx_grid = dFpsi_dr_grid * np.sin(PSI_grid) - dFz_dr_grid * beta_grid * np.cos(PSI_grid)
    dYy_grid = -rotation_dir * (dFpsi_dr_grid * np.cos(PSI_grid) + dFz_dr_grid * beta_grid * np.sin(PSI_grid))
    if spanwise_flow_correction:
        dHx_grid = dHx_grid + D_radial_grid * np.cos(PSI_grid)
        dYy_grid = dYy_grid + rotation_dir * D_radial_grid * np.sin(PSI_grid)

    h_force_N = float(az_weight * np.sum(dHx_grid * DR_grid))
    y_force_N = float(az_weight * np.sum(dYy_grid * DR_grid))
    mx_hub_Nm = float(az_weight * np.sum(-rotation_dir * R_grid * dFz_dr_grid * np.sin(PSI_grid) * DR_grid))
    my_hub_Nm = float(az_weight * np.sum(-R_grid * dFz_dr_grid * np.cos(PSI_grid) * DR_grid))
    mz_hub_Nm = float(-rotation_dir * torque_Nm)

    F_shaft = np.array([h_force_N, y_force_N, thrust_N])
    M_shaft = np.array([mx_hub_Nm, my_hub_Nm, mz_hub_Nm])
    F_body_N, M_body_Nm, _ = shaft_to_body_transform(F_shaft, M_shaft, theta_nac_deg, r_hub_body)

    area_disk = np.pi * (radius ** 2)
    ct = thrust_N / max(rho * area_disk * (v_tip ** 2), 1e-6)
    cp = power_W / max(rho * area_disk * (v_tip ** 3), 1e-6)

    if thrust_N > 0.0 and v_inf_ms < 1.0:
        p_ideal_W = (thrust_N ** 1.5) / np.sqrt(2.0 * rho * area_disk)
        fm_hover = float(np.clip(p_ideal_W / power_W, 0.0, 0.95))
    else:
        fm_hover = 0.0

    if thrust_N > 0.0 and v_axial_ms > 1.0:
        eta_prop = float(np.clip((thrust_N * v_axial_ms) / power_W, 0.0, 0.96))
    else:
        eta_prop = 0.0

    alpha_deg_grid = np.degrees(alpha_wrapped)
    working_mask = (RNORM_grid >= 0.25) & (UT_grid > 0.15 * v_tip)
    stall_mask = (np.abs(alpha_deg_grid) > airfoil.alpha_stall_deg) & working_mask
    if np.any(working_mask):
        p95_alpha_deg = float(np.percentile(np.abs(alpha_deg_grid[working_mask]), 95))
    else:
        p95_alpha_deg = float(np.percentile(np.abs(alpha_deg_grid), 95))
    stall_margin_deg = float(airfoil.alpha_stall_deg - p95_alpha_deg)

    return EdgewiseRotorResult(
        thrust_N=thrust_N,
        h_force_N=h_force_N,
        y_force_N=y_force_N,
        mx_hub_Nm=mx_hub_Nm,
        my_hub_Nm=my_hub_Nm,
        torque_Nm=torque_Nm,
        mz_hub_Nm=mz_hub_Nm,
        power_kW=power_W * 1e-3,
        F_body_N=F_body_N,
        M_body_Nm=M_body_Nm,
        CT=ct,
        CP=cp,
        FM_hover=fm_hover,
        eta_prop=eta_prop,
        mu_edge=mu_edge,
        mu_axial=mu_axial,
        lambda_G=lambda_G,
        lambda_i_mean=lambda_i_mean,
        kx_inflow=kx_inflow,
        max_tip_mach=float(np.max(Mach_grid)),
        min_ut_ms=float(np.min(UT_grid)),
        reverse_flow_area_frac=float(np.mean(rev_flow_mask)),
        stall_area_frac=float(np.mean(stall_mask)),
        stall_margin_deg=stall_margin_deg,
        valid_fraction=float(np.mean(valid_mask_grid)),
        beta_flap_deg=(
            float(np.degrees(beta_0_rad)),
            float(np.degrees(beta_1c_rad)),
            float(np.degrees(beta_1s_rad)),
        ),
    )


def trim_axial_rotor_collective(
    target_thrust_per_rotor_N: float,
    aircraft_cfg: Dict[str, Any],
    airfoil: AirfoilModel,
    rpm: float,
    v_axial_ms: float,
    rho: float,
    a_sound: float,
    mu_visc: float,
    theta_bounds_deg: Tuple[float, float] = (-2.0, 52.0),
) -> Tuple[float, EdgewiseRotorResult, bool]:
    """Solve single-rotor collective pitch to match target axial thrust (hover or cruise)."""
    rot = aircraft_cfg["geometry"]["rotor"]
    nac = 90.0 if v_axial_ms < 1e-3 else 0.0

    def eval_th(th0: float) -> EdgewiseRotorResult:
        return run_edgewise_bemt(
            radius=float(rot["radius_m"]),
            root_cutout=float(rot["root_cutout_m"]),
            num_blades=int(rot["num_blades"]),
            c_root=float(rot["root_chord_m"]),
            taper=float(rot["taper_ratio"]),
            collective_deg=float(th0),
            twist_deg=float(rot["twist_deg"]),
            rpm=float(rpm),
            v_inf_ms=float(v_axial_ms),
            theta_nac_deg=nac,
            airfoil=airfoil,
            rho=rho,
            a_sound=a_sound,
            mu_visc=mu_visc,
            num_radial=28,
            num_azimuth=1,
            flapping_mode="rigid",
            spanwise_flow_correction=False,
        )

    grid = np.linspace(theta_bounds_deg[0], theta_bounds_deg[1], 28)
    vals = [eval_th(g) for g in grid]
    diffs = np.array([v.thrust_N - target_thrust_per_rotor_N for v in vals])

    for i in range(len(diffs) - 1):
        if diffs[i] <= 0.0 <= diffs[i + 1]:
            th_sol = float(brentq(lambda x: eval_th(x).thrust_N - target_thrust_per_rotor_N, grid[i], grid[i + 1], xtol=1e-3))
            return th_sol, eval_th(th_sol), True

    idx_best = int(np.argmin(np.abs(diffs)))
    return float(grid[idx_best]), vals[idx_best], False
