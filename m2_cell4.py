# ==============================================================================
# CELL 4: MILESTONE 2 — COORDINATE SYSTEMS, 2D AZIMUTH-RESOLVED EDGEWISE BEMT,
#         SECTION 2.1 FLOW DIAGRAM & SECTION 3 VERIFICATION SUITE
# ==============================================================================

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from dataclasses import dataclass
from typing import Tuple, Optional

# ------------------------------------------------------------------------------
# 1. COORDINATE FRAMES & SIGN CONVENTIONS (TASK 1 / REPORT SECTION 1.1)
# ------------------------------------------------------------------------------
def euler_body_from_inertial(phi_rad: float, theta_rad: float, psi_rad: float = 0.0) -> np.ndarray:
    """3-2-1 Euler rotation matrix R_BI mapping Inertial (NED) vectors to Aircraft Body axes."""
    cp, sp = np.cos(phi_rad), np.sin(phi_rad)
    ct, st = np.cos(theta_rad), np.sin(theta_rad)
    cy, sy = np.cos(psi_rad), np.sin(psi_rad)
    return np.array([
        [ct * cy,                 ct * sy,                -st],
        [sp * st * cy - cp * sy,  sp * st * sy + cp * cy,  sp * ct],
        [cp * st * cy + sp * sy,  cp * st * sy - sp * cy,  cp * ct]
    ])


def shaft_to_body_transform(
    F_shaft: np.ndarray,
    M_shaft: np.ndarray,
    theta_nac_deg: float,
    r_hub_body: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Transform rotor hub forces & moments from Rotor-Shaft Frame F_S to Aircraft Body Frame F_B
    about the aircraft Center of Gravity (CG).
    """
    tn = np.radians(theta_nac_deg)
    R_BS = np.array([
        [-np.sin(tn), 0.0,  np.cos(tn)],
        [        0.0, 1.0,         0.0],
        [ np.cos(tn), 0.0, -np.sin(tn)]
    ])
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
    r_stations: np.ndarray
    psi_rad: np.ndarray
    R_grid: np.ndarray
    PSI_grid: np.ndarray
    UT_grid: np.ndarray
    UP_grid: np.ndarray
    Mach_grid: np.ndarray
    alpha_deg_grid: np.ndarray
    theta_deg_grid: np.ndarray
    lambda_i_grid: np.ndarray
    dFz_dr_grid: np.ndarray
    dFpsi_dr_grid: np.ndarray
    rev_flow_mask: np.ndarray
    stall_mask: np.ndarray


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
    num_radial: int = 30,
    num_azimuth: int = 72,
    rotation_dir: int = 1,
    r_hub_body: Optional[np.ndarray] = None,
    flapping_mode: str = "rigid",
) -> EdgewiseRotorResult:
    if r_hub_body is None:
        r_hub_body = np.array([0.0, 0.0, 0.0])

    r_edges = np.linspace(root_cutout, radius, num_radial + 1)
    r_stations = 0.5 * (r_edges[:-1] + r_edges[1:])
    dr = np.diff(r_edges)
    r_norm = r_stations / radius
    chords = c_root + (c_root * taper - c_root) * ((r_stations - root_cutout) / max(1e-4, radius - root_cutout))
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
    lambda_tot_r = np.full_like(r_stations, max(mu_axial, 1e-4) + 0.02)
    relax = 0.25

    for _ in range(70):
        f = (num_blades / 2.0) * (1.0 - r_norm) / np.maximum(np.abs(lambda_tot_r), 1e-4)
        f_loss = np.maximum((2.0 / np.pi) * np.arccos(np.exp(-np.clip(f, 0.0, 30.0))), 1e-3)
        u_t_0 = omega * r_stations
        u_p_0 = lambda_tot_r * v_tip
        phi_0 = np.arctan2(u_p_0, np.maximum(u_t_0, 1e-4))
        alpha_0 = theta_base_r - phi_0
        u_res_sq_0 = u_t_0 ** 2 + u_p_0 ** 2
        re_0 = (rho * np.sqrt(u_res_sq_0) * chords) / max(mu_visc, 1e-6)
        cl_0, cd_0, _ = airfoil.evaluate(alpha_0, re_0, mach_base_r)
        C_y_0 = cl_0 * np.cos(phi_0) - cd_0 * np.sin(phi_0)
        K_0 = (sigma_r / (8.0 * f_loss * np.maximum(r_norm, 0.05))) * (u_res_sq_0 / (v_tip ** 2)) * C_y_0
        if abs(mu_edge) < 1e-8:
            disc = (mu_axial / 2.0) ** 2 + K_0
            lambda_i_new = np.where(disc >= 0.0, -mu_axial / 2.0 + np.sqrt(np.maximum(0.0, disc)), -mu_axial / 2.0)
        else:
            v_glauert_norm = np.sqrt(mu_edge ** 2 + lambda_tot_r ** 2)
            lambda_i_new = np.maximum(0.0, K_0 / np.maximum(v_glauert_norm, 1e-3))
        lambda_tot_new = np.clip(mu_axial + lambda_i_new, 1e-4, 3.5)
        if np.max(np.abs(lambda_tot_new - lambda_tot_r)) < 1e-5:
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

    lambda_i_grid = lambda_i_glauert_r[:, None] * (1.0 + kx_inflow * RNORM_grid * np.cos(PSI_grid))
    lambda_i_grid = np.maximum(-mu_axial + 1e-4, lambda_i_grid)

    th_1c_rad = np.radians(cyclic_1c_deg)
    th_1s_rad = np.radians(cyclic_1s_deg)
    if flapping_mode == "steady_1st_harmonic" and abs(mu_edge) > 1e-4:
        beta_0_rad = 0.035 * (lambda_i_mean * 10.0)
        beta_1c_rad = -th_1s_rad + 0.18 * mu_edge * np.radians(collective_deg)
        beta_1s_rad = th_1c_rad - 0.10 * mu_edge * lambda_i_mean
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
    UP_grid = (
        v_axial_ms
        + lambda_i_grid * v_tip
        + R_grid * dbeta_dt_grid
        + v_edge_ms * beta_grid * np.cos(PSI_grid)
    )

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

    sgn_ut = np.where(rev_flow_mask, -1.0, 1.0)
    q_dyn = 0.5 * rho * U_res_sq * CHORD_grid
    dFz_dr_grid = q_dyn * (CL_grid * np.cos(phi_normal) - CD_grid * np.sin(phi_normal))
    dFpsi_dr_grid = q_dyn * (sgn_ut * CD_grid * np.cos(phi_normal) + CL_grid * np.sin(phi_normal))

    az_weight = float(num_blades) / float(num_azimuth)
    thrust_N = float(az_weight * np.sum(dFz_dr_grid * DR_grid))
    torque_Nm = float(az_weight * np.sum(R_grid * dFpsi_dr_grid * DR_grid))
    power_W = omega * torque_Nm

    dHx_grid = dFpsi_dr_grid * np.sin(PSI_grid) - dFz_dr_grid * beta_grid * np.cos(PSI_grid)
    dYy_grid = -rotation_dir * dFpsi_dr_grid * np.cos(PSI_grid) - rotation_dir * dFz_dr_grid * beta_grid * np.sin(PSI_grid)
    h_force_N = float(az_weight * np.sum(dHx_grid * DR_grid))
    y_force_N = float(az_weight * np.sum(dYy_grid * DR_grid))

    dMx_grid = -rotation_dir * R_grid * dFz_dr_grid * np.sin(PSI_grid)
    dMy_grid = -R_grid * dFz_dr_grid * np.cos(PSI_grid)
    mx_hub_Nm = float(az_weight * np.sum(dMx_grid * DR_grid))
    my_hub_Nm = float(az_weight * np.sum(dMy_grid * DR_grid))
    mz_hub_Nm = float(-rotation_dir * torque_Nm)

    F_shaft = np.array([h_force_N, y_force_N, thrust_N])
    M_shaft = np.array([mx_hub_Nm, my_hub_Nm, mz_hub_Nm])
    F_body_N, M_body_Nm, _ = shaft_to_body_transform(F_shaft, M_shaft, theta_nac_deg, r_hub_body)

    n_rps = rpm / 60.0
    d_prop = 2.0 * radius
    ct = thrust_N / max(rho * (n_rps ** 2) * (d_prop ** 4), 1e-6)
    cp = power_W / max(rho * (n_rps ** 3) * (d_prop ** 5), 1e-6)

    alpha_deg_grid = np.degrees(alpha_wrapped)
    stall_mask = np.abs(alpha_deg_grid) > airfoil.alpha_stall_deg
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
        r_stations=r_stations,
        psi_rad=psi_rad,
        R_grid=R_grid,
        PSI_grid=PSI_grid,
        UT_grid=UT_grid,
        UP_grid=UP_grid,
        Mach_grid=Mach_grid,
        alpha_deg_grid=alpha_deg_grid,
        theta_deg_grid=np.degrees(theta_grid),
        lambda_i_grid=lambda_i_grid,
        dFz_dr_grid=dFz_dr_grid,
        dFpsi_dr_grid=dFpsi_dr_grid,
        rev_flow_mask=rev_flow_mask,
        stall_mask=stall_mask,
    )



# ------------------------------------------------------------------------------
# SECTION 2 FLOW DIAGRAMS: 2.1 EDGEWISE ESTIMATOR, 2.2 TRIM SOLVER, 2.3 MISSION PLANNER V2
# ------------------------------------------------------------------------------
def _draw_styled_box(ax, x, y, w, h, title, bullets, header_bg="#162d4c", body_bg="#f8fafc", border_col="#162d4c", badge=None):
    from matplotlib.patches import FancyBboxPatch
    body = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.04,rounding_size=0.14",
        facecolor=body_bg, edgecolor=border_col, linewidth=1.8, zorder=2
    )
    ax.add_patch(body)
    hh = 0.54
    header = FancyBboxPatch(
        (x, y + h - hh), w, hh,
        boxstyle="round,pad=0.04,rounding_size=0.14",
        facecolor=header_bg, edgecolor=border_col, linewidth=1.8, zorder=3
    )
    ax.add_patch(header)
    if badge:
        ax.text(x + 0.20, y + h - hh/2, badge, color="#f0ad4e", fontsize=9.5, fontweight="bold", va="center", ha="left", zorder=4)
        ax.text(x + 0.68, y + h - hh/2, title, color="white", fontsize=10.0, fontweight="bold", va="center", ha="left", zorder=4)
    else:
        ax.text(x + w/2, y + h - hh/2, title, color="white", fontsize=10.0, fontweight="bold", va="center", ha="center", zorder=4)
    ty = y + h - hh - 0.20
    for b in bullets:
        ax.text(x + 0.15, ty, b, color="#1e293b", fontsize=8.4, va="top", ha="left", zorder=4, linespacing=1.24)
        ty -= (0.275 * (b.count("\n") + 1) + 0.08)

def _draw_styled_arrow(ax, x1, y1, x2, y2, label="", color="#162d4c", rad=0.0):
    from matplotlib.patches import FancyArrowPatch
    arr = FancyArrowPatch(
        (x1, y1), (x2, y2),
        connectionstyle=f"arc3,rad={rad}",
        arrowstyle="-|>,head_length=8,head_width=5",
        color=color, linewidth=2.2, zorder=5
    )
    ax.add_patch(arr)
    if label:
        mx, my = 0.5*(x1 + x2), 0.5*(y1 + y2)
        ax.text(mx, my + 0.12, label, fontsize=8.0, fontweight="bold", color=color, ha="center", va="bottom",
                bbox=dict(boxstyle="round,pad=0.12", facecolor="white", edgecolor="none", alpha=0.92), zorder=6)


def plot_section2_1_flowchart():
    fig, ax = plt.subplots(figsize=(16, 7.2), dpi=160)
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 7.2)
    ax.axis("off")
    fig.patch.set_facecolor("#ffffff")
    fig.suptitle(
        "2.1 Edgewise-Flight Performance Estimator — Radial/Azimuthal Loop, Inflow Solution & Coordinate Chain",
        fontsize=13.2, fontweight="bold", color="#162d4c", y=0.98
    )

    _draw_styled_box(ax, 0.25, 3.70, 4.65, 3.05, "Inputs, Frame Decomposition & 2D Mesh", [
        "• Flight State: V_inf, h (ISA rho, a, mu), alpha_B, Tilt theta_nac, RPM",
        "• Pilot Controls: Collective theta_0, Lat Cyclic theta_1c, Lon Cyclic theta_1s",
        "• Shaft Projection (F_I -> F_B -> F_S):\n"
        "    V_axial = V_inf cos(theta_nac + alpha_B)\n"
        "    V_edge  = V_inf sin(theta_nac + alpha_B),   mu = V_edge / (Omega R)",
        "• 2D Polar Grid: N_r = 30 radial stations x N_psi = 72 azimuth sectors"
    ], header_bg="#162d4c", body_bg="#f1f5f9", border_col="#162d4c", badge="[1]")

    _draw_styled_box(ax, 5.55, 3.70, 4.90, 3.05, "Glauert + Non-Uniform Inflow Loop", [
        "• Inner Fixed-Point Loop (Relaxation omega = 0.25, tol < 1e-5):\n"
        "    Prandtl Tip-Loss: F(r) = (2/pi) arccos(exp(-f_tip))\n"
        "    Glauert Balance: 2 lambda_i sqrt(mu^2 + lambda_tot^2) = C_T,local(r)",
        "• Exact Axisymmetric Recovery: mu -> 0 reduces identically to M1 BEMT",
        "• Coleman / Pitt-Peters Longitudinal Inflow Gradient:\n"
        "    K_x = [(4/3)(mu/lambda_G)] / [1.2 + mu/lambda_G]\n"
        "    lambda_i(r, psi) = lambda_i,Glauert(r) * [1 + K_x (r/R) cos(psi)]"
    ], header_bg="#0f4c81", body_bg="#eff6ff", border_col="#0f4c81", badge="[2]")

    _draw_styled_box(ax, 11.10, 3.70, 4.65, 3.05, "2D Blade Kinematics & Reverse Flow", [
        "• Local Blade Pitch Law theta(r, psi):\n"
        "    theta(r, psi) = theta_0 + theta_tw(r/R - 0.75) + theta_1c cos(psi) + theta_1s sin(psi)",
        "• Velocities in Blade-Element Frame F_BE(r, psi):\n"
        "    U_T(r, psi) = Omega r + V_edge sin(psi)\n"
        "    U_P(r, psi) = V_axial + lambda_i(r, psi) Omega R + r dbeta/dt + V_edge beta cos(psi)",
        "• Reverse-Flow Region (U_T < 0 on Retreating Side psi in [pi, 2pi]):\n"
        "    Trailing-edge AoA sign flip + separated drag increment"
    ], header_bg="#162d4c", body_bg="#f1f5f9", border_col="#162d4c", badge="[3]")

    _draw_styled_box(ax, 11.10, 0.25, 4.65, 2.95, "Sectional Polars, Mach & Stall Checks", [
        "• Bivariate Spline Polar Lookup: C_l(alpha, Re), C_d(alpha, Re)",
        "• Compressibility & Transonic Drag Divergence:\n"
        "    Prandtl-Glauert beta = sqrt(1 - M^2) + Wave Drag (M > 0.78)",
        "• Sectional Elemental Loads (N/m) per Blade:\n"
        "    dF_z/dr   = 0.5 rho U^2 c (C_l cos(phi) - C_d sin(phi))\n"
        "    dF_psi/dr = 0.5 rho U^2 c (sgn(U_T) C_d cos(phi) + C_l sin(phi))"
    ], header_bg="#1e3a5f", body_bg="#fffbeb", border_col="#d97706", badge="[4]")

    _draw_styled_box(ax, 5.55, 0.25, 4.90, 2.95, "Azimuthal Cycle Integration (Hub Frame F_S)", [
        "• Integrate over (r_i, psi_j) across N_b Blades (Weight = N_b / N_psi):\n"
        "    Thrust T = F_zs = Sum (dF_z/dr) dr,   Torque Q = Sum r (dF_psi/dr) dr",
        "• In-Plane Hub Forces (H-Force & Side Y-Force):\n"
        "    H = F_xs = Sum (dF_psi/dr sin(psi) - dF_z/dr beta cos(psi)) dr\n"
        "    Y = F_ys = Sum (-s_rot dF_psi/dr cos(psi) - s_rot dF_z/dr beta sin(psi)) dr",
        "• Hub Pitch & Roll Moments: M_xs, M_ys, M_zs = -s_rot Q, Power P = Omega Q"
    ], header_bg="#0f5132", body_bg="#f0fdf4", border_col="#198754", badge="[5]")

    _draw_styled_box(ax, 0.25, 0.25, 4.65, 2.95, "Shaft -> Body CG Transform & Diagnostics", [
        "• Nacelle Tilt Rotation Matrix R_BS(theta_nac) (F_S -> F_B):\n"
        "    F_B = [F_X, F_Y, F_Z]^T = R_BS(theta_nac) * [H, Y, T]^T",
        "• Aircraft CG Moment Transfer (Hub Lever Arm r_hub,B):\n"
        "    M_B,CG = [M_X, M_Y, M_Z]^T = R_BS(theta_nac) M_S + r_hub,B x F_B",
        "• Envelope & Safety Outputs:\n"
        "    Peak Advancing Tip Mach M_tip, Stall Margin, Reverse-Flow Fraction"
    ], header_bg="#162d4c", body_bg="#f8fafc", border_col="#162d4c", badge="[6]")

    _draw_styled_arrow(ax, 4.90, 5.22, 5.55, 5.22, "mu, mu_z, r_i")
    _draw_styled_arrow(ax, 10.45, 5.22, 11.10, 5.22, "lambda_i(r, psi)")
    _draw_styled_arrow(ax, 13.42, 3.70, 13.42, 3.20, "alpha, Re, M")
    _draw_styled_arrow(ax, 11.10, 1.72, 10.45, 1.72, "dF_z/dr, dF_psi/dr")
    _draw_styled_arrow(ax, 5.55, 1.72, 4.90, 1.72, "F_S, M_S, P")

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig("slide_2_1_edgewise_flowchart.png", dpi=220)
    plt.show()


def plot_section2_2_trim_flowchart():
    fig, ax = plt.subplots(figsize=(16, 7.2), dpi=160)
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 7.2)
    ax.axis("off")
    fig.patch.set_facecolor("#ffffff")
    fig.suptitle(
        "2.2 Trim Solver — 6-DOF Coupled Aircraft Force/Moment Equilibrium & Feasibility Logic",
        fontsize=13.2, fontweight="bold", color="#162d4c", y=0.98
    )

    _draw_styled_box(ax, 0.25, 3.70, 4.65, 3.05, "Conversion Flight Condition & Unknowns u", [
        "• Fixed Conversion State Inputs:\n"
        "    Airspeed V_inf, Altitude h, Climb Angle gamma, Nacelle Angle theta_nac,\n"
        "    Scheduled Rotor Speed RPM(theta_nac), Gross Weight W = m g",
        "• Unknown Trim Control & Attitude Vector u (6-DOF / Symmetric 4-DOF):\n"
        "    u = [theta_0 (Collective), theta_1s (Lon Cyclic), theta_1c (Lat Cyclic),\n"
        "         theta_pitch (Body Pitch), delta_e (Elevator), phi_roll (Bank)]^T",
        "• Box Bounds: theta_0 in [-5°, 45°], theta_1s,1c in [-12°, 12°], delta_e in [-25°, 25°]"
    ], header_bg="#162d4c", body_bg="#f1f5f9", border_col="#162d4c", badge="[1]")

    _draw_styled_box(ax, 5.55, 3.70, 4.90, 3.05, "Dual Counter-Rotating Rotor Evaluation", [
        "• Call 2D Edgewise BEMT Solver for Port (CCW, s=+1) & Starboard (CW, s=-1):",
        "    F_rot,L, M_rot,L = run_edgewise_bemt(theta_0, theta_1c, theta_1s, +1)\n"
        "    F_rot,R, M_rot,R = run_edgewise_bemt(theta_0, -theta_1c, theta_1s, -1)",
        "• Lateral / Yaw Symmetry Cancellation in Steady Symmetric Flight:\n"
        "    Counter-rotation cancels net torque M_Z, side force F_Y, and roll M_X",
        "• Rotor Wake & Download Interference on Wing:\n"
        "    Induced wash v_i,rot modifies wing effective incidence & hover download"
    ], header_bg="#0f4c81", body_bg="#eff6ff", border_col="#0f4c81", badge="[2]")

    _draw_styled_box(ax, 11.10, 3.70, 4.65, 3.05, "Airframe Aero: Wing, Fuselage, Nacelle & Tail", [
        "• Wing Lift, Drag & Pitching Moment about AC (x_ac,w, z_w):\n"
        "    alpha_w = alpha_B + i_w - alpha_wash(theta_nac, lambda_i)\n"
        "    L_w = q S_w C_L(alpha_w),   D_w = q S_w (C_D0,w + C_L^2 / (pi AR e))",
        "• Fuselage + Tilting Nacelle Parasite Drag & Pitching Couple:\n"
        "    D_fuse+nac = q S_w (C_D0,fuse + Delta C_D,nac sin^2(theta_nac))",
        "• Horizontal & Vertical Empennage Loads (x_ht, x_vt):\n"
        "    L_ht = q S_ht (a_ht alpha_ht + tau_e delta_e),   M_Y,ht = -x_ht L_ht"
    ], header_bg="#162d4c", body_bg="#f1f5f9", border_col="#162d4c", badge="[3]")

    _draw_styled_box(ax, 11.10, 0.25, 4.65, 2.95, "6-DOF Force & Moment Residual Vector R(u)", [
        "• Sum All Component Loads in Body Axes F_B about Aircraft CG:\n"
        "    R_1 = F_X,rot + F_X,aero - W sin(theta_pitch) = 0   (Axial X Equilibrium)\n"
        "    R_2 = F_Y,rot + F_Y,aero + W cos(theta_pitch) sin(phi) = 0 (Side Y)\n"
        "    R_3 = F_Z,rot + F_Z,aero + W cos(theta_pitch) cos(phi) = 0 (Vertical Z)",
        "• Moment Equilibrium Residuals about CG:\n"
        "    R_4 = Sum M_X = 0,   R_5 = Sum M_Y = 0,   R_6 = Sum M_Z = 0"
    ], header_bg="#1e3a5f", body_bg="#fffbeb", border_col="#d97706", badge="[4]")

    _draw_styled_box(ax, 5.55, 0.25, 4.90, 2.95, "Bounded Trust-Region Newton / LM Solver", [
        "• Numerical Root-Finder (scipy.optimize.least_squares / hybrid Powell):\n"
        "    Minimize ||W_scale · R(u)||_2^2 subject to control bounds u_min <= u <= u_max",
        "• Finite-Difference Jacobian J_ij = dR_i / du_j with Trust-Region Step delta_u",
        "• Convergence Criterion Check:\n"
        "    ||F_res||_inf < 5.0 N  AND  ||M_res||_inf < 5.0 N·m\n"
        "    If not converged: update u <- u + delta_u and re-evaluate [2]-[4]"
    ], header_bg="#0f5132", body_bg="#f0fdf4", border_col="#198754", badge="[5]")

    _draw_styled_box(ax, 0.25, 0.25, 4.65, 2.95, "Post-Convergence Feasibility Classification", [
        "• Verify Converged Trim Solution Against Active Physical Limits:\n"
        "    1) Power Margin: 2 P_rotor / eta_xmsn <= P_installed(h)\n"
        "    2) Rotor Stall Margin: alpha_stall - P95(|alpha_blade|) >= 0°\n"
        "    3) Wing Stall Margin: |alpha_w| <= alpha_w,stall (15.5°)\n"
        "    4) Advancing Tip Mach: M_tip <= 0.85   &   Control Margin > 0°",
        "• Output Status: FEASIBLE TRIM vs. Categorized Failure Mode (Sec 6.3/7.2)"
    ], header_bg="#162d4c", body_bg="#f8fafc", border_col="#162d4c", badge="[6]")

    _draw_styled_arrow(ax, 4.90, 5.22, 5.55, 5.22, "Trial State u")
    _draw_styled_arrow(ax, 10.45, 5.22, 11.10, 5.22, "Rotor Loads + Wake")
    _draw_styled_arrow(ax, 13.42, 3.70, 13.42, 3.20, "Aero Loads")
    _draw_styled_arrow(ax, 11.10, 1.72, 10.45, 1.72, "Residual R(u)")
    _draw_styled_arrow(ax, 8.00, 3.20, 8.00, 3.70, "Iterate u + du", color="#d97706")
    _draw_styled_arrow(ax, 5.55, 1.72, 4.90, 1.72, "Converged u*")

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig("slide_2_2_trim_solver_flowchart.png", dpi=220)
    plt.show()


def plot_section2_3_mission_flowchart():
    fig, ax = plt.subplots(figsize=(16, 7.2), dpi=160)
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 7.2)
    ax.axis("off")
    fig.patch.set_facecolor("#ffffff")
    fig.suptitle(
        "2.3 Mission Planner v2 — Outbound/Inbound Transition Schedule, Trim Integration & Fuel/State Continuity",
        fontsize=13.2, fontweight="bold", color="#162d4c", y=0.98
    )

    _draw_styled_box(ax, 0.25, 3.70, 4.65, 3.05, "Mission Profile & Conversion Corridor Schedule", [
        "• Mission Segments: Hover -> Outbound Transition -> Climb -> Cruise ->\n"
        "    Descent -> Inbound Transition (Airplane-to-Hover) -> Landing",
        "• Time-Stepped Transition State Vector at step k (Delta t = 1.0 - 2.0 s):\n"
        "    [t_k, h_k, V_k, V_ground,k, gamma_k, theta_nac(t_k), RPM(t_k), W_k]",
        "• Corridor-Guided Schedule: Smooth cosine/linear nacelle tilt theta_nac(V)\n"
        "    from 90° (Helicopter) to 0° (Airplane) inside feasible V-theta_nac corridor"
    ], header_bg="#162d4c", body_bg="#f1f5f9", border_col="#162d4c", badge="[1]")

    _draw_styled_box(ax, 5.55, 3.70, 4.90, 3.05, "Quasi-Steady Longitudinal Acceleration & Trim", [
        "• Include Longitudinal Inertial Acceleration Term dV/dt & Climb Rate ROC:\n"
        "    Effective Axial Demand: F_X,net = m (dV_k/dt) + W_k sin(gamma_k)",
        "• Online 6-DOF Trim Solution at Every Time Step t_k:\n"
        "    Solve u_k* = [theta_0, theta_1s, theta_1c, theta_pitch, delta_e, phi]_k\n"
        "    using warm-start initial guess u_(k-1)* for rapid 2-4 iteration convergence",
        "• Extract Lift Sharing: Rotor Lift Share L_rot(t_k) vs. Wing Lift L_wing(t_k)"
    ], header_bg="#0f4c81", body_bg="#eff6ff", border_col="#0f4c81", badge="[2]")

    _draw_styled_box(ax, 11.10, 3.70, 4.65, 3.05, "Aerodynamic, Power & Control Limit Gate", [
        "• Real-Time Envelope & Feasibility Verification at Step t_k:\n"
        "    1) Total Shaft Power: P_req(t_k) <= P_avail(h_k) (ISA lapse rate)\n"
        "    2) Rotor & Wing Stall Margins: Delta alpha_rot > 0°, |alpha_w| < 15.5°\n"
        "    3) Helical Advancing Tip Mach: M_tip(t_k) <= 0.85\n"
        "    4) Nacelle Tilt Rate |d(theta_nac)/dt| <= 8°/s & Control Bounds",
        "• Flag/Reject Any Transition State Violating Corridor Constraints"
    ], header_bg="#162d4c", body_bg="#f1f5f9", border_col="#162d4c", badge="[3]")

    _draw_styled_box(ax, 11.10, 0.25, 4.65, 2.95, "Propulsion Power, SFC & Fuel Burn Update", [
        "• Total Aircraft Shaft Power Demand at Step t_k:\n"
        "    P_total(t_k) = (P_rotor,L + P_rotor,R) / eta_gearbox + P_acc",
        "• Specific Fuel Consumption (SFC) Mass Integration over Delta t:\n"
        "    dm_fuel,k = (SFC [kg/(kW·h)] / 3600) * P_total(t_k) * Delta t",
        "• Fuel & Gross Weight State Update:\n"
        "    m_fuel(t_{k+1}) = m_fuel(t_k) - dm_fuel,k,   W_{k+1} = W_k - dm_fuel,k g"
    ], header_bg="#1e3a5f", body_bg="#fffbeb", border_col="#d97706", badge="[4]")

    _draw_styled_box(ax, 5.55, 0.25, 4.90, 2.95, "Trajectory Kinematics & State Continuity", [
        "• Update Position, Altitude & Ground Speed with Headwind/Tailwind V_wind:\n"
        "    V_ground(t_k) = V_k cos(gamma_k) - V_wind\n"
        "    h_{k+1} = h_k + V_k sin(gamma_k) Delta t\n"
        "    x_{k+1} = x_k + V_ground(t_k) Delta t",
        "• Enforce C^0 State Continuity across Hover <-> Transition <-> Cruise\n"
        "    boundaries (continuous V, h, W, theta_nac, RPM, and trim controls)"
    ], header_bg="#0f5132", body_bg="#f0fdf4", border_col="#198754", badge="[5]")

    _draw_styled_box(ax, 0.25, 0.25, 4.65, 2.95, "Mission Telemetry & Section 8 Deliverables", [
        "• Record Full Time-History Telemetry Arrays (t in [0, T_mission]):\n"
        "    h(t), V_TAS(t), V_GS(t), theta_nac(t), RPM(t), [theta_0, theta_1s, delta_e](t),\n"
        "    P_req(t) vs P_avail(t), m_fuel(t), Stall Margins, and Lift Share %",
        "• Generate Section 8.2 Outbound (Hover -> Airplane) and Inbound\n"
        "    (Airplane -> Hover) 8-Panel Conversion Telemetry Plots & Summary Table"
    ], header_bg="#162d4c", body_bg="#f8fafc", border_col="#162d4c", badge="[6]")

    _draw_styled_arrow(ax, 4.90, 5.22, 5.55, 5.22, "State(t_k)")
    _draw_styled_arrow(ax, 10.45, 5.22, 11.10, 5.22, "Trim Solution u_k*")
    _draw_styled_arrow(ax, 13.42, 3.70, 13.42, 3.20, "Feasible Step")
    _draw_styled_arrow(ax, 11.10, 1.72, 10.45, 1.72, "W_{k+1}, m_fuel")
    _draw_styled_arrow(ax, 8.00, 3.20, 8.00, 3.70, "Next Step k <- k+1", color="#198754")
    _draw_styled_arrow(ax, 5.55, 1.72, 4.90, 1.72, "Mission Telemetry")

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig("slide_2_3_mission_planner_v2_flowchart.png", dpi=220)
    plt.show()

# ------------------------------------------------------------------------------
# 3. SECTION 3 VERIFICATION SUITE (RENDERED INLINE IN NOTEBOOK)
# ------------------------------------------------------------------------------
def run_section3_verification_suite(show_plots: bool = True):
    plot_section2_1_flowchart()
    plot_section2_2_trim_flowchart()
    plot_section2_3_mission_flowchart()

    R_rot = float(SIZED_VEHICLE.get("Rotor_Radius", 4.60))
    r_cut = 0.10 * R_rot
    Nb = int(SIZED_VEHICLE.get("Num_Blades", 3))
    c_root = float(SIZED_VEHICLE.get("Blade_Chord", 0.50))
    taper = 0.75
    twist_deg = float(SIZED_VEHICLE.get("Twist_deg", -30.0))
    th_hov = float(SIZED_VEHICLE.get("Pitch_Hover", 14.5))
    th_cr = float(SIZED_VEHICLE.get("Pitch_Cruise", 38.0))
    rpm_hov = float(w_rpm_hov.value) if "w_rpm_hov" in globals() else 535.0
    rpm_cr = float(w_rpm_cr.value) if "w_rpm_cr" in globals() else 200.0
    v_cr_ms = float(SIZED_VEHICLE.get("V_cruise_kmh", 400.0)) / 3.6
    h_cr_m = float(SIZED_VEHICLE.get("H_cruise_m", 3000.0))
    af_model = SIZED_VEHICLE.get("Airfoil", AirfoilModel("Boeing-Vertol VR-12"))

    rho_sl, a_sl, _, mu_sl = isa_atmosphere(0.0, 0.0)
    rho_cr, a_cr, _, mu_cr = isa_atmosphere(h_cr_m, 0.0)

    # 3.1 Recovery of Milestone 1 Limiting Cases
    m1_hov = run_bemt_solver(
        R_rot, r_cut, Nb, c_root, taper, th_hov, twist_deg, rpm_hov, 0.0,
        af_model, rho=rho_sl, a_sound=a_sl, mu=mu_sl, num_elements=30
    )
    m2_hov = run_edgewise_bemt(
        R_rot, r_cut, Nb, c_root, taper, th_hov, twist_deg, rpm_hov,
        v_inf_ms=0.0, theta_nac_deg=90.0, airfoil=af_model,
        rho=rho_sl, a_sound=a_sl, mu_visc=mu_sl, num_radial=30, num_azimuth=72
    )

    m1_cr = run_bemt_solver(
        R_rot, r_cut, Nb, c_root, taper, th_cr, twist_deg, rpm_cr, v_cr_ms,
        af_model, rho=rho_cr, a_sound=a_cr, mu=mu_cr, num_elements=30
    )
    m2_cr = run_edgewise_bemt(
        R_rot, r_cut, Nb, c_root, taper, th_cr, twist_deg, rpm_cr,
        v_inf_ms=v_cr_ms, theta_nac_deg=0.0, airfoil=af_model,
        rho=rho_cr, a_sound=a_cr, mu_visc=mu_cr, num_radial=30, num_azimuth=72
    )

    df_sec31 = pd.DataFrame([
        {
            "Limiting Case": "Hover (theta_nac=90 deg, V=0 m/s)",
            "M1 Thrust [N]": m1_hov.thrust_N,
            "M2 Edgewise Thrust [N]": m2_hov.thrust_N,
            "Thrust Rel. Error [%]": 100.0 * abs(m2_hov.thrust_N - m1_hov.thrust_N) / max(abs(m1_hov.thrust_N), 1e-6),
            "M1 Power [kW]": m1_hov.power_kW,
            "M2 Edgewise Power [kW]": m2_hov.power_kW,
            "Power Rel. Error [%]": 100.0 * abs(m2_hov.power_kW - m1_hov.power_kW) / max(abs(m1_hov.power_kW), 1e-6),
        },
        {
            "Limiting Case": f"Axial Cruise (theta_nac=0 deg, V={v_cr_ms:.1f} m/s)",
            "M1 Thrust [N]": m1_cr.thrust_N,
            "M2 Edgewise Thrust [N]": m2_cr.thrust_N,
            "Thrust Rel. Error [%]": 100.0 * abs(m2_cr.thrust_N - m1_cr.thrust_N) / max(abs(m1_cr.thrust_N), 1e-6),
            "M1 Power [kW]": m1_cr.power_kW,
            "M2 Edgewise Power [kW]": m2_cr.power_kW,
            "Power Rel. Error [%]": 100.0 * abs(m2_cr.power_kW - m1_cr.power_kW) / max(abs(m1_cr.power_kW), 1e-6),
        },
    ])

    print("=" * 95)
    print("SECTION 3.1 — RECOVERY OF MILESTONE 1 LIMITING CASES (AXISYMMETRIC REGRESSION CHECK)")
    print("=" * 95)
    print(df_sec31.to_string(index=False, float_format=lambda x: f"{x:.6f}"))
    print()

    v_edge_rep = 60.0
    tn_rep = 75.0
    rpm_rep = 480.0
    r_hub_port = np.array([0.0, -0.5 * float(SIZED_VEHICLE.get("Wingspan", 20.0)), -0.8])

    m2_rep = run_edgewise_bemt(
        R_rot, r_cut, Nb, c_root, taper,
        collective_deg=14.0, twist_deg=twist_deg, rpm=rpm_rep,
        v_inf_ms=v_edge_rep, theta_nac_deg=tn_rep, airfoil=af_model,
        cyclic_1c_deg=1.2, cyclic_1s_deg=-3.5, alpha_body_deg=2.0,
        rho=rho_sl, a_sound=a_sl, mu_visc=mu_sl,
        num_radial=40, num_azimuth=90, rotation_dir=1, r_hub_body=r_hub_port,
        flapping_mode="steady_1st_harmonic"
    )

    print("=" * 95)
    print(f"SECTION 3.2 & 3.3 — REPRESENTATIVE EDGEWISE CONDITION (V={v_edge_rep:.0f} m/s, theta_nac={tn_rep:.0f} deg, RPM={rpm_rep:.0f})")
    print("=" * 95)
    print(f"  Advance Ratio mu_edge      : {m2_rep.mu_edge:.4f}   | Axial Inflow mu_z         : {m2_rep.mu_axial:.4f}")
    print(f"  Glauert Total Inflow lam_G : {m2_rep.lambda_G:.4f}   | Longitudinal Gradient K_x : {m2_rep.kx_inflow:.4f}")
    print(f"  Rotor Thrust T             : {m2_rep.thrust_N:,.1f} N | Rotor Shaft Power P       : {m2_rep.power_kW:,.1f} kW")
    print(f"  Hub H-Force (aft)          : {m2_rep.h_force_N:,.1f} N | Hub Side Force Y          : {m2_rep.y_force_N:,.1f} N")
    print(f"  Hub Roll Moment M_xs       : {m2_rep.mx_hub_Nm:,.1f} N*m | Hub Pitch Moment M_ys     : {m2_rep.my_hub_Nm:,.1f} N*m")
    print(f"  Body Forces [FX, FY, FZ]   : [{m2_rep.F_body_N[0]:,.1f}, {m2_rep.F_body_N[1]:,.1f}, {m2_rep.F_body_N[2]:,.1f}] N")
    print(f"  Advancing Tip Mach Max     : {m2_rep.max_tip_mach:.3f}    | Min Tangential Vel U_T    : {m2_rep.min_ut_ms:.2f} m/s")
    print(f"  Reverse-Flow Disk Area     : {100.0*m2_rep.reverse_flow_area_frac:.2f}%   | Stall Margin (P95)        : {m2_rep.stall_margin_deg:+.2f} deg")
    print()

    nr_list = [8, 12, 16, 24, 32, 45, 60, 80]
    npsi_list = [8, 12, 18, 24, 36, 48, 72, 108, 144]

    T_vs_nr, Q_vs_nr = [], []
    for nr in nr_list:
        res_nr = run_edgewise_bemt(
            R_rot, r_cut, Nb, c_root, taper, 14.0, twist_deg, rpm_rep,
            v_edge_rep, tn_rep, af_model, cyclic_1c_deg=1.2, cyclic_1s_deg=-3.5,
            rho=rho_sl, a_sound=a_sl, mu_visc=mu_sl, num_radial=nr, num_azimuth=72
        )
        T_vs_nr.append(res_nr.thrust_N * 1e-3)
        Q_vs_nr.append(res_nr.torque_Nm * 1e-3)

    T_vs_npsi, Q_vs_npsi = [], []
    for npsi in npsi_list:
        res_npsi = run_edgewise_bemt(
            R_rot, r_cut, Nb, c_root, taper, 14.0, twist_deg, rpm_rep,
            v_edge_rep, tn_rep, af_model, cyclic_1c_deg=1.2, cyclic_1s_deg=-3.5,
            rho=rho_sl, a_sound=a_sl, mu_visc=mu_sl, num_radial=30, num_azimuth=npsi
        )
        T_vs_npsi.append(res_npsi.thrust_N * 1e-3)
        Q_vs_npsi.append(res_npsi.torque_Nm * 1e-3)

    if show_plots:
        psi_closed = np.append(m2_rep.psi_rad, 2.0 * np.pi)
        PSI_C, R_C = np.meshgrid(psi_closed, m2_rep.r_stations)
        def close_az(arr):
            return np.hstack([arr, arr[:, :1]])

        fig = plt.figure(figsize=(18, 11.5), dpi=140)
        fig.suptitle(
            f"Milestone 2 — Section 3: Edgewise-Flight Rotor Verification Suite "
            f"(V = {v_edge_rep:.0f} m/s, θ_nac = {tn_rep:.0f}°, μ = {m2_rep.mu_edge:.2f}, RPM = {rpm_rep:.0f})",
            fontsize=14, fontweight="bold", y=0.98
        )

        ax1 = fig.add_subplot(2, 3, 1, projection="polar")
        c1 = ax1.contourf(PSI_C, R_C, close_az(m2_rep.dFz_dr_grid), levels=24, cmap="turbo")
        ax1.set_theta_zero_location("S")
        ax1.set_theta_direction(1)
        ax1.set_title("Sec 3.2: Sectional Thrust Load dFz/dr [N/m]\n(ψ=0° Tail, ψ=90° Adv, ψ=270° Ret)", fontsize=10.5, fontweight="bold", pad=12)
        plt.colorbar(c1, ax=ax1, pad=0.10, fraction=0.046, label="dFz/dr [N/m]")

        ax2 = fig.add_subplot(2, 3, 2, projection="polar")
        ut_closed = close_az(m2_rep.UT_grid)
        c2 = ax2.contourf(PSI_C, R_C, ut_closed, levels=24, cmap="RdYlBu_r")
        ax2.contour(PSI_C, R_C, ut_closed, levels=[0.0], colors="black", linewidths=2.2, linestyles="--")
        psi_rf = np.linspace(np.pi, 2.0 * np.pi, 100)
        r_rf = np.maximum(r_cut, -m2_rep.mu_edge * R_rot * np.sin(psi_rf))
        ax2.plot(psi_rf, r_rf, color="yellow", lw=2.0, ls=":", label="Reverse-Flow Circle (U_T = 0)")
        ax2.set_theta_zero_location("S")
        ax2.set_theta_direction(1)
        ax2.set_title("Sec 3.3: Tangential Velocity U_T(r, ψ) [m/s]\n& Reverse-Flow Region (U_T < 0)", fontsize=10.5, fontweight="bold", pad=12)
        ax2.legend(loc="upper right", fontsize=8, framealpha=0.85)
        plt.colorbar(c2, ax=ax2, pad=0.10, fraction=0.046, label="U_T [m/s]")

        ax3 = fig.add_subplot(2, 3, 3, projection="polar")
        mach_closed = close_az(m2_rep.Mach_grid)
        alpha_closed = np.abs(close_az(m2_rep.alpha_deg_grid))
        c3 = ax3.contourf(PSI_C, R_C, mach_closed, levels=22, cmap="magma")
        ax3.contour(PSI_C, R_C, mach_closed, levels=[0.75, 0.80], colors="cyan", linewidths=1.8)
        ax3.contour(PSI_C, R_C, alpha_closed, levels=[af_model.alpha_stall_deg], colors="lime", linewidths=2.2, linestyles="-.")
        ax3.set_theta_zero_location("S")
        ax3.set_theta_direction(1)
        ax3.set_title(f"Sec 3.3: Local Mach & Stall Boundary\n(Peak M_tip = {m2_rep.max_tip_mach:.3f}, α_stall = {af_model.alpha_stall_deg:.1f}°)", fontsize=10.5, fontweight="bold", pad=12)
        plt.colorbar(c3, ax=ax3, pad=0.10, fraction=0.046, label="Local Mach M(r, ψ)")

        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(m1_hov.r_stations / R_rot, m1_hov.dt_dr * 1e-3, "o-", color="#1f77b4", lw=2.2, label="M1 Hover (dT/dr)")
        ax4.plot(m2_hov.r_stations / R_rot, (Nb * np.mean(m2_hov.dFz_dr_grid, axis=1)) * 1e-3, "--", color="#ff7f0e", lw=2.2, label="M2 Edgewise (μ=0 Hover)")
        ax4.plot(m1_cr.r_stations / R_rot, m1_cr.dt_dr * 1e-3, "s-", color="#2ca02c", lw=2.0, label="M1 Axial Cruise (dT/dr)")
        ax4.plot(m2_cr.r_stations / R_rot, (Nb * np.mean(m2_cr.dFz_dr_grid, axis=1)) * 1e-3, ":", color="#d62728", lw=2.2, label="M2 Edgewise (θ_nac=0° Cruise)")
        ax4.set_xlabel("Normalized Radial Station r/R [-]")
        ax4.set_ylabel("Sectional Rotor Thrust dT/dr [kN/m]")
        ax4.set_title("Sec 3.1: Recovery of Milestone 1 Limiting Cases", fontsize=10.5, fontweight="bold")
        ax4.grid(True, alpha=0.35)
        ax4.legend(fontsize=8.5)

        ax5 = fig.add_subplot(2, 3, 5)
        ax5_r = ax5.twinx()
        l1 = ax5.plot(nr_list, T_vs_nr, "o-", color="#0d6efd", lw=2.0, label="Thrust T vs N_r [kN]")
        l2 = ax5_r.plot(nr_list, Q_vs_nr, "s--", color="#dc3545", lw=2.0, label="Torque Q vs N_r [kN·m]")
        ax5.axvline(30, color="gray", ls=":", lw=1.5, label="Selected N_r = 30")
        ax5.set_xlabel("Number of Radial Sections N_r [-]")
        ax5.set_ylabel("Rotor Thrust T [kN]", color="#0d6efd")
        ax5_r.set_ylabel("Rotor Torque Q [kN·m]", color="#dc3545")
        ax5.set_title("Sec 3.4 (Plots 1 & 2): Radial Discretization Sensitivity", fontsize=10.5, fontweight="bold")
        ax5.grid(True, alpha=0.35)
        lines5 = l1 + l2
        ax5.legend(lines5, [l.get_label() for l in lines5], loc="lower right", fontsize=8.5)

        ax6 = fig.add_subplot(2, 3, 6)
        ax6_r = ax6.twinx()
        l3 = ax6.plot(npsi_list, T_vs_npsi, "o-", color="#198754", lw=2.0, label="Thrust T vs N_ψ [kN]")
        l4 = ax6_r.plot(npsi_list, Q_vs_npsi, "d--", color="#fd7e14", lw=2.0, label="Torque Q vs N_ψ [kN·m]")
        ax6.axvline(72, color="gray", ls=":", lw=1.5, label="Selected N_ψ = 72")
        ax6.set_xlabel("Number of Azimuthal Sectors N_ψ [-]")
        ax6.set_ylabel("Rotor Thrust T [kN]", color="#198754")
        ax6_r.set_ylabel("Rotor Torque Q [kN·m]", color="#fd7e14")
        ax6.set_title("Sec 3.4 (Plots 3 & 4): Azimuthal Discretization Sensitivity", fontsize=10.5, fontweight="bold")
        ax6.grid(True, alpha=0.35)
        lines6 = l3 + l4
        ax6.legend(lines6, [l.get_label() for l in lines6], loc="lower right", fontsize=8.5)

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()

    return {
        "df_sec31": df_sec31,
        "m2_hov": m2_hov,
        "m2_cr": m2_cr,
        "m2_rep": m2_rep,
    }


# Execute Section 2.1 Flowchart + Section 3 Verification Suite
SEC3_RESULTS = run_section3_verification_suite(show_plots=True)
