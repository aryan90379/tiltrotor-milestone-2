"""Generate all publication-grade Milestone 3 figures from config & simulation outputs."""

from pathlib import Path
import sys
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from milestone3_package.src import load_config


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.size": 9.5,
        "axes.titlesize": 10.5,
        "axes.titleweight": "bold",
        "axes.labelsize": 9.5,
        "axes.labelweight": "bold",
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8.0,
        "figure.facecolor": "white",
        "axes.facecolor": "#fbfcfe",
        "axes.edgecolor": "#334155",
        "axes.linewidth": 0.9,
    }
)

SEG_COLORS = {
    "Takeoff / hover": "#dbeafe",
    "Conversion": "#fef3c7",
    "Cruise": "#dcfce7",
    "Reconversion": "#fde68a",
    "Landing": "#ede9fe",
}


def _shade_segments(ax: plt.Axes, df: pd.DataFrame, add_labels: bool = True) -> None:
    segs = ["Takeoff / hover", "Conversion", "Cruise", "Reconversion", "Landing"]
    short_lbl = {
        "Takeoff / hover": "1",
        "Conversion": "2",
        "Cruise": "3: Cruise",
        "Reconversion": "4",
        "Landing": "5",
    }
    ylim = ax.get_ylim()
    for s in segs:
        sub = df[df["segment"] == s]
        if sub.empty:
            continue
        t0 = float(df.loc[sub.index[0] - 1, "time_min"]) if sub.index[0] > 0 else float(sub["time_min"].iloc[0])
        t1 = float(sub["time_min"].iloc[-1])
        ax.axvspan(t0, t1, color=SEG_COLORS.get(s, "#f1f5f9"), alpha=0.42, zorder=0)
        if s != "Takeoff / hover":
            ax.axvline(t0, color="#64748b", ls=":", lw=0.9, alpha=0.8, zorder=1)
        if add_labels and (t1 - t0) > 6.0:
            ax.text(
                0.5 * (t0 + t1),
                ylim[1] - 0.06 * (ylim[1] - ylim[0]),
                short_lbl[s],
                ha="center",
                va="top",
                fontsize=8.0,
                fontweight="bold",
                color="#1e293b",
                bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="#94a3b8", alpha=0.85),
            )


def plot_1_1_schematic(cfg: dict, out_paths: list[Path]) -> None:
    """Three-view dimensioned engineering schematic of the frozen TR-9 tiltrotor."""
    g = cfg["geometry"]
    rot = g["rotor"]
    wing = g["wing"]
    emp = g["empennage"]
    coords = g["coordinates_body_m"]

    L_f, W_f, H_f = float(g["fuselage_length_m"]), float(g["fuselage_width_m"]), float(g["fuselage_height_m"])
    R = float(rot["radius_m"])
    b_w, S_w, lam_w = float(wing["span_m"]), float(wing["area_m2"]), float(wing["taper_ratio"])
    c_root_w = 2.0 * S_w / (b_w * (1.0 + lam_w))
    c_tip_w = lam_w * c_root_w
    b_ht = float(np.sqrt(float(emp["s_htail_m2"]) * float(emp["ar_htail"])))
    c_ht = float(emp["s_htail_m2"]) / b_ht
    h_vt = float(np.sqrt(float(emp["s_vtail_m2"]) * float(emp["ar_vtail"])))
    c_vt = float(emp["s_vtail_m2"]) / h_vt

    x_cg_h = float(coords["x_cg_helo_m"])
    x_cg_a = float(coords["x_cg_airplane_m"])
    x_hub = float(coords["x_hub_pivot_m"])
    y_hub = float(coords["y_hub_m"])
    x_w_le = float(coords["x_ac_wing_m"]) - 0.25 * c_root_w
    x_ht_le = float(coords["x_ac_htail_m"]) - 0.25 * c_ht
    x_vt_le = float(coords["x_ac_vtail_m"]) - 0.25 * c_vt

    fig = plt.figure(figsize=(12.0, 7.68), dpi=260)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1.0], height_ratios=[1.15, 1.0], hspace=0.28, wspace=0.20)

    # --- Panel A: Top Planform View ---
    ax_top = fig.add_subplot(gs[0, 0])
    fx = np.array([0.0, 1.4, 2.8, 10.6, 14.2, L_f, 14.2, 10.6, 2.8, 1.4, 0.0])
    fy = np.array([0.0, 0.72, 0.5 * W_f, 0.5 * W_f, 0.42, 0.0, -0.42, -0.5 * W_f, -0.5 * W_f, -0.72, 0.0])
    ax_top.fill(fx, fy, color="#cbd5e1", ec="#0f172a", lw=1.4, zorder=3, label="Fuselage (15.20 × 2.15 m)")

    # Payload cabin region
    p0, p1 = coords["x_payload_bay_range_m"]
    ax_top.add_patch(
        patches.Rectangle((p0, -0.82), p1 - p0, 1.64, fc="#fde68a", ec="#b45309", lw=1.1, ls="--", zorder=4, label="Payload Bay (1,500 kg)")
    )

    # Wing planform
    wx = [x_w_le, x_w_le + 0.18, x_w_le + 0.18 + c_tip_w, x_w_le + c_root_w, x_w_le + 0.18 + c_tip_w, x_w_le + 0.18, x_w_le]
    wy = [0.0, 0.5 * b_w, 0.5 * b_w, 0.0, -0.5 * b_w, -0.5 * b_w, 0.0]
    ax_top.fill(wx, wy, color="#bfdbfe", ec="#1d4ed8", lw=1.4, alpha=0.88, zorder=2, label="Wing (S=43 m², b=20 m, i_w=3.5°)")

    # Integral wing fuel tanks + cross-shaft
    for sgn in [-1.0, 1.0]:
        ax_top.add_patch(
            patches.Rectangle((x_w_le + 0.45, sgn * 1.3 if sgn > 0 else -5.8), 1.15, 4.5, fc="#bbf7d0", ec="#15803d", lw=1.0, zorder=3, label="Integral Fuel Tanks (1,100 kg)" if sgn > 0 else None)
        )
        # Flaperon
        ax_top.add_patch(
            patches.Rectangle((x_w_le + 1.55, sgn * 1.4 if sgn > 0 else -9.2), 0.52, 7.8, fc="#93c5fd", ec="#1e40af", lw=0.8, ls=":", zorder=3)
        )
        # Wingtip nacelles
        ax_top.add_patch(
            patches.FancyBboxPatch((x_hub - 1.35, sgn * y_hub - 0.48), 2.75, 0.96, boxstyle="round,pad=0.08", fc="#475569", ec="#0f172a", lw=1.2, zorder=5, label="Turboshaft Nacelles + Tilt Axis" if sgn > 0 else None)
        )
        # Helicopter rotor discs
        circ = patches.Circle((x_hub, sgn * y_hub), R, fc="#fca5a5", ec="#dc2626", lw=1.3, ls="--", alpha=0.18, zorder=1)
        circ_edge = patches.Circle((x_hub, sgn * y_hub), R, fc="none", ec="#dc2626", lw=1.3, ls="--", zorder=6, label="Proprotor Disc (R=4.60 m)" if sgn > 0 else None)
        ax_top.add_patch(circ)
        ax_top.add_patch(circ_edge)
        # Airplane-mode proprotor plane line
        ax_top.plot([x_hub - 1.45, x_hub - 1.45], [sgn * y_hub - R, sgn * y_hub + R], color="#0f766e", lw=2.4, zorder=6)

    # Interconnect shaft line
    ax_top.plot([x_hub, x_hub], [-y_hub, y_hub], color="#dc2626", lw=1.2, ls="-.", zorder=4, label="Cross-Shaft Drive")

    # Empennage
    ax_top.add_patch(patches.Rectangle((x_ht_le, -0.5 * b_ht), c_ht, b_ht, fc="#94a3b8", ec="#0f172a", lw=1.2, zorder=2, label="H-Tail (9.8 m²) / V-Tail (4.6 m²)"))

    # CG marker
    ax_top.plot(x_cg_h, 0.0, marker="o", ms=7, mfc="#f59e0b", mec="black", mew=1.3, zorder=8, label=f"CG (x={x_cg_h:.2f}m Helo / {x_cg_a:.2f}m Air)")

    # Dimension annotations
    ax_top.annotate("", xy=(0.0, -15.3), xytext=(L_f, -15.3), arrowprops=dict(arrowstyle="<->", color="#0f172a", lw=1.1))
    ax_top.text(0.5 * L_f, -16.3, f"L_fuse = {L_f:.2f} m", ha="center", va="top", fontsize=8.0, fontweight="bold")
    ax_top.annotate("", xy=(16.3, -0.5 * b_w), xytext=(16.3, 0.5 * b_w), arrowprops=dict(arrowstyle="<->", color="#1d4ed8", lw=1.1))
    ax_top.text(16.7, 0.0, f"b = {b_w:.1f} m", rotation=90, ha="left", va="center", fontsize=8.0, fontweight="bold", color="#1d4ed8")
    ax_top.annotate("", xy=(x_hub, 1.075), xytext=(x_hub, y_hub - R), arrowprops=dict(arrowstyle="<->", color="#15803d", lw=1.1))
    ax_top.text(x_hub + 0.3, 3.2, "Clearance\n4.33 m", fontsize=7.3, fontweight="bold", color="#15803d")

    ax_top.set_xlim(-2.2, 18.2)
    ax_top.set_ylim(-17.8, 15.8)
    ax_top.set_aspect("equal")
    ax_top.set_title("(a) Top Planform View (Nose Origin x=0, +y Starboard)", fontsize=9.8)
    ax_top.set_xlabel("Station x [m]")
    ax_top.set_ylabel("Spanwise y [m]")
    ax_top.grid(True, alpha=0.28, ls="--")

    # --- Panel B: Side Elevation View (showing both 90° Helo & 0° Airplane Nacelle Modes) ---
    ax_side = fig.add_subplot(gs[1, 0])
    # Ground datum at z_up = -2.05 m (CG at z_up = 0.0 m; fuselage belly at -1.02 m; gear wheels at -2.05 m)
    ax_side.axhline(-2.05, color="#475569", lw=1.5, ls="-")
    ax_side.text(0.3, -2.38, "Ground Plane (Gear Extended)", fontsize=7.5, color="#475569", fontweight="bold")

    sx = np.array([0.0, 1.1, 2.6, 10.5, 14.5, L_f, 14.6, 10.2, 1.8, 0.0])
    sz = np.array([-0.35, 0.45, 1.02, 1.02, 0.72, 0.35, -0.15, -1.02, -1.02, -0.35])
    ax_side.fill(sx, sz, color="#cbd5e1", ec="#0f172a", lw=1.4, zorder=2)

    # Cockpit window & payload cabin
    ax_side.fill([1.1, 2.5, 2.5, 1.4], [0.42, 0.96, 0.35, 0.25], color="#38bdf8", ec="#0284c7", lw=1.0, zorder=3)
    ax_side.add_patch(patches.Rectangle((p0, -0.82), p1 - p0, 1.55, fc="#fde68a", ec="#b45309", lw=1.0, ls="--", zorder=3))
    ax_side.text(0.5 * (p0 + p1), -0.05, "Payload Cabin\n(1,500 kg)", ha="center", va="center", fontsize=7.4, fontweight="bold", color="#92400e", zorder=4)

    # High wing root section + V-tail + H-tail
    ax_side.add_patch(patches.Polygon([[x_w_le, 0.72], [x_w_le + c_root_w, 0.56], [x_w_le + c_root_w, 0.82], [x_w_le, 0.98]], fc="#93c5fd", ec="#1d4ed8", lw=1.2, zorder=4))
    ax_side.add_patch(patches.Polygon([[x_vt_le, 0.68], [x_vt_le + 0.9, 0.68 + h_vt], [x_vt_le + 0.9 + 0.75 * c_vt, 0.68 + h_vt], [x_vt_le + c_vt, 0.55]], fc="#94a3b8", ec="#0f172a", lw=1.2, zorder=1))
    ax_side.add_patch(patches.Rectangle((x_ht_le, 0.50), c_ht, 0.20, fc="#475569", ec="#0f172a", lw=1.0, zorder=4))

    # Tricycle landing gear
    for gx in [1.95, 7.05]:
        ax_side.plot([gx, gx], [-1.02, -1.80], color="#1e293b", lw=2.2, zorder=3)
        ax_side.add_patch(patches.Circle((gx, -1.83), 0.22, fc="#1e293b", ec="black", zorder=4))

    # Airplane mode nacelle (0 deg, solid teal) + rotor plane
    z_hub_up = -float(coords["z_hub_m"])  # +0.85 m above CG
    ax_side.add_patch(patches.Rectangle((x_hub - 1.45, z_hub_up - 0.34), 2.50, 0.68, fc="#0f766e", ec="#042f2e", lw=1.2, alpha=0.88, zorder=5))
    ax_side.plot([x_hub - 1.55, x_hub - 1.55], [z_hub_up - R, z_hub_up + R], color="#0f766e", lw=2.5, zorder=6, label="Airplane Mode (θ_nac = 0°)")

    # Helicopter mode nacelle (90 deg, dashed crimson) + rotor plane
    ax_side.add_patch(patches.Rectangle((x_hub - 0.34, z_hub_up - 0.30), 0.68, 2.15, fc="none", ec="#dc2626", lw=1.4, ls="--", zorder=6))
    ax_side.plot([x_hub - R, x_hub + R], [z_hub_up + 1.95, z_hub_up + 1.95], color="#dc2626", lw=2.2, ls="--", zorder=6, label="Helicopter Mode (θ_nac = 90°)")

    # CG & Pivot markers
    ax_side.plot(x_cg_h, 0.0, marker="o", ms=7, mfc="#f59e0b", mec="black", mew=1.3, zorder=8)
    ax_side.plot(x_hub, z_hub_up, marker="^", ms=6.5, mfc="#dc2626", mec="black", zorder=8)
    ax_side.annotate(f"CG ({x_cg_h:.2f} m)\nPivot ({x_hub:.2f} m)", xy=(x_cg_h, 0.0), xytext=(8.6, -1.75), fontsize=7.4, fontweight="bold", arrowprops=dict(arrowstyle="->", lw=0.9))
    ax_side.annotate(f"l_ht = {float(coords['x_ac_htail_m'])-x_cg_h:.2f} m", xy=(float(coords["x_ac_htail_m"]), 0.60), xytext=(9.6, 2.35), fontsize=7.6, fontweight="bold", color="#334155", arrowprops=dict(arrowstyle="->", lw=0.9))

    # Body reference axes indicator
    ax_side.annotate("+X_B (fwd)", xy=(-1.9, 3.6), xytext=(-0.3, 3.6), fontsize=7.5, fontweight="bold", color="#dc2626", arrowprops=dict(arrowstyle="->", color="#dc2626", lw=1.3))
    ax_side.annotate("+Z_B", xy=(-0.3, 2.3), xytext=(-0.3, 3.6), fontsize=7.5, fontweight="bold", color="#dc2626", arrowprops=dict(arrowstyle="->", color="#dc2626", lw=1.3))

    ax_side.set_xlim(-2.4, 17.5)
    ax_side.set_ylim(-2.8, 5.8)
    ax_side.set_aspect("equal")
    ax_side.set_title("(b) Side Profile View (Showing Both Helicopter 90° & Airplane 0° Nacelles)", fontsize=9.8)
    ax_side.set_xlabel("Station x from Nose [m]")
    ax_side.set_ylabel("Vertical Height wrt CG [m]")
    ax_side.legend(loc="upper right", fontsize=7.4)
    ax_side.grid(True, alpha=0.28, ls="--")

    # --- Panel C: Front Elevation View + Legend Summary ---
    ax_front = fig.add_subplot(gs[0, 1])
    ax_front.axhline(-2.05, color="#475569", lw=1.4)
    ax_front.add_patch(patches.Ellipse((0.0, 0.0), W_f, H_f, fc="#cbd5e1", ec="#0f172a", lw=1.4, zorder=3))
    ax_front.add_patch(patches.Rectangle((-0.5 * b_w, 0.60), b_w, 0.26, fc="#93c5fd", ec="#1d4ed8", lw=1.3, zorder=2))
    ax_front.add_patch(patches.Rectangle((-0.5 * b_ht, 0.52), b_ht, 0.14, fc="#94a3b8", ec="#0f172a", lw=1.0, zorder=1))
    ax_front.plot([0.0, 0.0], [0.68, 0.68 + h_vt], color="#475569", lw=3.0, zorder=2)

    for sgn in [-1.0, 1.0]:
        ax_front.add_patch(patches.Rectangle((sgn * y_hub - 0.42, z_hub_up - 0.42), 0.84, 0.84, fc="#475569", ec="black", zorder=5))
        ax_front.add_patch(patches.Circle((sgn * y_hub, z_hub_up), R, fc="#99f6e4", ec="#0f766e", lw=1.3, ls="-", alpha=0.20, zorder=1))
        ax_front.plot([sgn * y_hub - R, sgn * y_hub + R], [z_hub_up + 1.95, z_hub_up + 1.95], color="#dc2626", lw=2.0, ls="--", zorder=6)
        ax_front.plot([sgn * 1.45, sgn * 1.45], [-0.90, -1.83], color="#1e293b", lw=2.0, zorder=3)
        ax_front.add_patch(patches.Circle((sgn * 1.45, -1.83), 0.22, fc="#1e293b", zorder=4))

    ax_front.plot(0.0, 0.0, marker="o", ms=7, mfc="#f59e0b", mec="black", zorder=7)
    ax_front.annotate("", xy=(-y_hub, -3.5), xytext=(y_hub, -3.5), arrowprops=dict(arrowstyle="<->", color="#1d4ed8", lw=1.1))
    ax_front.text(0.0, -4.2, "Hub Span 2·y_hub = 20.00 m (D_rotor = 9.20 m)", ha="center", fontsize=7.8, fontweight="bold", color="#1d4ed8")

    ax_front.set_xlim(-16.0, 16.0)
    ax_front.set_ylim(-4.9, 5.9)
    ax_front.set_aspect("equal")
    ax_front.set_title("(c) Front Elevation View (Proprotor Arcs & Clearance)", fontsize=9.8)
    ax_front.set_xlabel("Spanwise Station y [m]")
    ax_front.set_ylabel("Height wrt CG [m]")
    ax_front.grid(True, alpha=0.28, ls="--")

    # --- Panel D: Legend & Frozen Key Configuration Summary Card ---
    ax_info = fig.add_subplot(gs[1, 1])
    ax_info.axis("off")
    handles, labels = ax_top.get_legend_handles_labels()
    ax_info.legend(handles, labels, loc="upper center", ncol=2, fontsize=7.6, frameon=True, facecolor="#f8fafc", edgecolor="#cbd5e1", title="Labeled Configuration Elements", title_fontsize=8.5)

    spec_txt = (
        "TR-9 FROZEN DESIGN COORDINATES & MASS SUMMARY\n"
        "• Reference Datum: Nose tip x = 0 m (+x aft layout; +X_B fwd in trim)\n"
        f"• MTOW = 6,800 kg | Empty = 4,200 kg | Payload = 1,500 kg | Fuel = 1,100 kg\n"
        f"• Proprotors: 2 × 3-bladed VR-12, R = 4.60 m, σ = 0.091, θ_tw = -35°, 535/360 rpm\n"
        f"• High Wing: S = 43.0 m², b = 20.0 m, AR = 9.30, i_w = 3.5°, 20° conv. flaperons\n"
        f"• Empennage: H-Tail S_ht = 9.80 m² (l_ht = 7.60 m) | V-Tail S_vt = 4.60 m²\n"
        f"• Propulsion: 2 × 2,150 kW Turboshafts (4,300 kW SL, 4,050 kW Xmsn limit)\n"
        f"• CG Shift: Helo (6.20, 0, 0) m -> Airplane (6.16, 0, 0) m; Pivot at (6.10, ±10, -0.85) m"
    )
    ax_info.text(
        0.5, 0.02, spec_txt, ha="center", va="bottom", fontsize=7.8, family="monospace",
        bbox=dict(boxstyle="round,pad=0.45", fc="#eff6ff", ec="#3b82f6", lw=1.1)
    )

    fig.suptitle("TR-9 Final Frozen Tiltrotor Configuration — Three-View Dimensioned Schematic", fontsize=12.0, fontweight="bold", y=0.98)
    for p in out_paths:
        fig.savefig(p, dpi=260, bbox_inches="tight")
    plt.close(fig)


def plot_2_2_workflow_diagram(out_paths: list[Path]) -> None:
    """Crisp integrated modular architecture & data-exchange diagram (Slide 10)."""
    fig, ax = plt.subplots(figsize=(14.4, 5.9), dpi=260)
    ax.set_xlim(0, 14.4)
    ax.set_ylim(0, 5.9)
    ax.axis("off")

    c_in = ("#f1f5f9", "#334155")
    c_m1 = ("#dbeafe", "#1d4ed8")
    c_m2 = ("#ccfbf1", "#0f766e")
    c_m3 = ("#fef3c7", "#b45309")
    c_out = ("#ede9fe", "#6d28d9")

    def box(x, y, w, h, title, body, colors, badge=None):
        fc, ec = colors
        ax.add_patch(patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08,rounding_size=0.12", fc=fc, ec=ec, lw=1.8, zorder=3))
        ax.text(x + 0.5 * w, y + h - 0.24, title, ha="center", va="top", fontsize=9.3, fontweight="bold", color="#0f172a", zorder=4)
        ax.text(x + 0.5 * w, y + 0.44 * h, body, ha="center", va="center", fontsize=7.8, color="#1e293b", zorder=4)
        if badge:
            bc = {"M1": "#1d4ed8", "M2": "#0f766e", "M3 Updated": "#d97706", "M3 New": "#b45309"}[badge]
            ax.text(x + w - 0.08, y + h - 0.06, badge, ha="right", va="top", fontsize=6.8, fontweight="bold", color="white",
                    bbox=dict(boxstyle="round,pad=0.14", fc=bc, ec="none"), zorder=5)

    def arrow(x0, y0, x1, y1, label="", lx_off=0.0, ly_off=0.12):
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle="-|>,head_width=0.25,head_length=0.4", color="#334155", lw=1.5), zorder=5)
        if label:
            ax.text(0.5 * (x0 + x1) + lx_off, 0.5 * (y0 + y1) + ly_off, label, ha="center", va="center", fontsize=7.2, fontweight="bold", color="#0f172a",
                    bbox=dict(boxstyle="round,pad=0.10", fc="white", ec="#cbd5e1", alpha=0.92), zorder=6)

    # Top Input Bar
    box(0.3, 4.95, 13.8, 0.78, "USER CONFIGURATION INPUTS (Zero Hard-Coded Parameters)",
        "config/aircraft_final.yaml (frozen geometry, mass, polars, limits)   •   config/mission_final.yaml (6 segments, payload, ISA ΔT, headwind, reserve rules)", c_in)

    # Middle Physics Modules
    box(0.3, 2.95, 2.05, 1.45, "1. Atmosphere\n(atmosphere.py)", "ρ(h), a(h), T(h), μ(h)\nV_GS = V_TAS cos γ − V_w", c_m1, "M1")
    box(2.65, 2.95, 2.15, 1.45, "2. Axial BEMT\n(airfoil_bemt.py)", "VR-12 multi-Re polars\nHover FM, Cruise η_p", c_m1, "M1")
    box(5.10, 2.95, 2.35, 1.45, "3. Edgewise BEMT\n(airfoil_bemt.py)", "Pitt-Peters skewed wake\nGormont radial stall delay\nF_rot, M_rot, M_tip, α_95%", c_m2, "M2")
    box(7.75, 2.95, 2.25, 1.45, "4. Aircraft Aero\n(aircraft_aero.py)", "Wing + 20° conv. flaperon\nDownload F_DL, downwash\nTail loads & Δx_CG(θ_nac)", c_m3, "M3 Updated")
    box(10.30, 2.95, 1.85, 1.45, "5. 6-DOF Trim\n(trim_solver.py)", "F_net = 0, M_net = 0\nFCC cyclic washout\nu=[θ_0, θ_1s, θ, δ_e]", c_m2, "M2")
    box(12.40, 2.95, 1.70, 1.45, "6. Propulsion\n(propulsion.py)", "P_avail(h) ISA lapse\n4050 kW gearbox cap\nPart-load SFC(χ)", c_m3, "M3 Updated")

    # Mission Planner Bar
    box(0.3, 1.25, 13.8, 1.15, "7. INTEGRATED END-TO-END MISSION PLANNER (mission_planner.py)",
        "Time-marching across VTOL Takeoff -> Outbound Conversion -> Climb/Cruise/Descent -> Reconversion -> Vertical Landing + 20-min Loiter Reserve\n"
        "Schedules θ_nac(t) & RPM(t)  •  Solves 6-DOF trim at each step  •  Integrates m_fuel(t) & W(t)  •  Evaluates all 6 normalized limit margins", c_m3, "M3 New")

    # Output Bar
    box(0.3, 0.15, 13.8, 0.72, "REPRODUCIBLE OUTPUTS (results/*.csv & *.png)",
        "Section 3 Hover/Cruise/Corridor/Limits Tables & Curves   •   Section 4 Mission Time-Histories (h, V, x, m, m_f, θ_nac, RPM, P_req/avail, 6 Feasibility Margins)", c_out)

    # Arrows from Input to Modules
    for xm in [1.32, 3.72, 6.27, 8.87, 11.22, 13.25]:
        arrow(xm, 4.95, xm, 4.40)

    # Horizontal exchanges
    arrow(2.35, 3.68, 2.65, 3.68, "ρ, a, μ", ly_off=0.25)
    arrow(4.80, 3.68, 5.10, 3.68, "VR-12", ly_off=0.25)
    arrow(7.45, 3.68, 7.75, 3.68, "F_rot, v_i", ly_off=0.25)
    arrow(10.00, 3.68, 10.30, 3.68, "F_aero, M", ly_off=0.25)
    arrow(12.15, 3.68, 12.40, 3.68, "P_rot", ly_off=0.25)

    # Exchanges with Mission Planner
    arrow(1.32, 2.95, 1.32, 2.40, "V_GS, ρ")
    arrow(6.27, 2.95, 6.27, 2.40, "M_tip, Δα_rot")
    arrow(8.87, 2.95, 8.87, 2.40, "L/D, Δα_w")
    arrow(11.22, 2.95, 11.22, 2.40, "u_trim, F_res")
    arrow(13.25, 2.95, 13.25, 2.40, "P_avail, ṁ_f")
    arrow(10.75, 2.40, 10.75, 2.95, "h, V, θ_nac, W(t)", lx_off=-0.65, ly_off=0.0)

    arrow(7.20, 1.25, 7.20, 0.87, "Telemetry & Margin Tables")

    for p in out_paths:
        fig.savefig(p, dpi=260, bbox_inches="tight")
    plt.close(fig)


def plot_3_1_hover_and_cruise(res_dir: Path, out_dir_extra: Path) -> None:
    """Section 3.1 Hover curve and Airplane-Mode Cruise curve (Slide 13)."""
    df_hov = pd.read_csv(res_dir / "sec3_1_hover_sweep.csv")
    df_cr = pd.read_csv(res_dir / "sec3_1_cruise_sweep.csv")

    # --- Left Plot: Hover Shaft Power & FM vs Gross Mass / Thrust ---
    fig1, ax1 = plt.subplots(figsize=(6.8, 3.15), dpi=260)
    ax1_r = ax1.twinx()

    sl = df_hov[df_hov["altitude_m"] == 0.0]
    h3 = df_hov[df_hov["altitude_m"] == 3000.0]

    ax1.plot(sl["mass_kg"] / 1000.0, sl["p_req_kW"], color="#1d4ed8", lw=2.2, label="P_req SL ISA (535 rpm)")
    ax1.plot(h3["mass_kg"] / 1000.0, h3["p_req_kW"], color="#0284c7", lw=1.9, ls="--", label="P_req 3000 m ISA")
    ax1.axhline(float(sl["p_avail_kW"].iloc[0]), color="#dc2626", lw=1.5, ls=":", label=f"P_avail SL ({sl['p_avail_kW'].iloc[0]:.0f} kW Xmsn)")
    ax1.axvline(6.80, color="#15803d", lw=1.5, ls="-.", label="Design MTOW (6.8 t)")

    sl_f = sl[sl["feasible"]]
    if not sl_f.empty:
        m_max = float(sl_f["mass_kg"].max()) / 1000.0
        p_max = float(sl_f["p_req_kW"].iloc[-1])
        ax1.plot(m_max, p_max, marker="*", ms=10, color="#dc2626", mec="black", zorder=6, label=f"SL Xmsn Power Cap ({m_max:.1f} t)")

    ax1_r.plot(sl["mass_kg"] / 1000.0, sl["FM"], color="#d97706", lw=1.9, ls="-", label="Figure of Merit FM (SL)")
    ax1_r.set_ylabel("Figure of Merit FM [-]", color="#b45309")
    ax1_r.tick_params(axis="y", labelcolor="#b45309")
    ax1_r.set_ylim(0.45, 0.88)

    ax1.set_title("OGE Hover Power & Figure of Merit vs Gross Mass (5.2% Download)", fontsize=9.5)
    ax1.set_xlabel("Aircraft Gross Mass [×10³ kg]")
    ax1.set_ylabel("Total Shaft Power [kW]", color="#1d4ed8")
    ax1.set_ylim(800, 4500)
    ax1.grid(True, alpha=0.3, ls="--")
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax1_r.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=6.8, ncol=2)

    for p in [res_dir / "fig_3_1a_hover_performance.png", out_dir_extra / "fig_3_1a_hover_performance.png"]:
        fig1.savefig(p, dpi=260, bbox_inches="tight")
    plt.close(fig1)

    # --- Right Plot: Airplane-Mode Power Curve & Aircraft L/D vs Airspeed ---
    fig2, ax2 = plt.subplots(figsize=(6.8, 3.15), dpi=260)
    ax2_r = ax2.twinx()

    fc = df_cr[df_cr["feasible"]]
    ax2.plot(df_cr["v_tas_ms"], df_cr["p_req_kW"], color="#cbd5e1", lw=1.6, ls="--")
    ax2.plot(fc["v_tas_ms"], fc["p_req_kW"], color="#0f766e", lw=2.3, label="P_req Feasible (360 rpm)")
    ax2.axhline(float(df_cr["p_avail_kW"].iloc[0]), color="#dc2626", lw=1.6, ls=":", label=f"P_avail 3000m ({df_cr['p_avail_kW'].iloc[0]:.0f} kW)")

    idx_br = fc["breguet_range_km"].idxmax()
    ax2.plot(fc.loc[idx_br, "v_tas_ms"], fc.loc[idx_br, "p_req_kW"], "o", ms=7.5, color="#f59e0b", mec="black", zorder=6,
             label=f"Best Range ({fc.loc[idx_br, 'v_tas_ms']:.0f} m/s, {fc.loc[idx_br, 'breguet_range_km']:.0f} km)")
    ax2.axvline(111.11, color="#1d4ed8", lw=1.5, ls="-.", label="Design Cruise (111.1 m/s)")

    ax2_r.plot(fc["v_tas_ms"], fc["aircraft_LD"], color="#7c3aed", lw=1.9, label="Trimmed Aircraft L/D")
    ax2_r.set_ylabel("Aircraft L/D [-]", color="#6d28d9")
    ax2_r.tick_params(axis="y", labelcolor="#6d28d9")
    ax2_r.set_ylim(4.0, 15.0)

    ax2.set_title("Airplane-Mode Power Curve & L/D vs TAS (3000 m ISA, 6800 kg, θ_nac=0°)", fontsize=9.5)
    ax2.set_xlabel("True Airspeed V_TAS [m/s]")
    ax2.set_ylabel("Total Shaft Power [kW]", color="#0f766e")
    ax2.set_ylim(300, 4100)
    ax2.grid(True, alpha=0.3, ls="--")
    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax2_r.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=6.8, ncol=2)

    for p in [res_dir / "fig_3_1b_cruise_performance.png", out_dir_extra / "fig_3_1b_cruise_performance.png"]:
        fig2.savefig(p, dpi=260, bbox_inches="tight")
    plt.close(fig2)


def plot_3_2_conversion_corridor(res_dir: Path, out_dir_extra: Path) -> None:
    """Section 3.2 Speed-Nacelle Conversion Corridor Map with Mission Overlay (Slide 14)."""
    df_cor = pd.read_csv(res_dir / "sec3_2_conversion_corridor_final.csv")
    df_tel = pd.read_csv(res_dir / "sec4_2_mission_telemetry.csv")

    v_u = np.sort(df_cor["V_ms"].unique())
    n_u = np.sort(df_cor["nacelle_deg"].unique())
    code_map = {
        "FEASIBLE": 0,
        "Wing Stall": 1,
        "Tip-Mach Limit": 2,
        "Rotor Stall": 3,
        "Control Saturation": 4,
        "Excessive Trim Residual": 4,
        "Power Limit": 5,
    }
    Z = np.zeros((len(n_u), len(v_u)))
    P_grid = np.zeros_like(Z)
    for _, r in df_cor.iterrows():
        i = int(np.where(n_u == r["nacelle_deg"])[0][0])
        j = int(np.where(v_u == r["V_ms"])[0][0])
        Z[i, j] = code_map.get(str(r["primary_constraint"]), 4)
        P_grid[i, j] = float(r["power_kW"])

    from matplotlib.colors import ListedColormap, BoundaryNorm
    cmap = ListedColormap(["#bbf7d0", "#fed7aa", "#fecaca", "#fde68a", "#e2e8f0", "#ddd6fe"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5], cmap.N)

    fig, ax = plt.subplots(figsize=(7.6, 5.65), dpi=260)
    ax.pcolormesh(v_u, n_u, Z, cmap=cmap, norm=norm, shading="nearest", alpha=0.92)

    # Power contours inside feasible corridor
    P_masked = np.where(Z == 0, P_grid, np.nan)
    try:
        cs = ax.contour(v_u, n_u, P_masked, levels=[700, 1000, 1500, 2200], colors="#0f766e", linewidths=0.9, alpha=0.65)
        ax.clabel(cs, fmt="%d kW", fontsize=7.2)
    except Exception:
        pass

    # Overlay Milestone 2 baseline feasible points if available for direct evolution proof
    m2_path = ROOT / "section7_feasible_envelope.csv"
    if m2_path.exists():
        df_m2 = pd.read_csv(m2_path)
        ax.scatter(df_m2["V_ms"], df_m2["nacelle_deg"], s=18, facecolors="none", edgecolors="#475569", lw=0.9,
                   label="Milestone 2 Feasible States (65/475)")

    # Overlay Outbound & Inbound conversion paths from Section 4 mission
    out_c = df_tel[df_tel["segment"].isin(["Takeoff / hover", "Conversion", "Cruise"]) & (df_tel["tas_ms"] <= 115.0)]
    in_c = df_tel[df_tel["segment"].isin(["Reconversion", "Landing"])]
    conv_only = df_tel[df_tel["segment"].isin(["Conversion", "Reconversion"])]
    idx_restrict = conv_only["min_norm_margin"].idxmin()
    rp = df_tel.loc[idx_restrict]

    s2_df = df_tel[df_tel["segment"] == "Conversion"]
    ax.plot(s2_df["tas_ms"], s2_df["nacelle_deg"], color="#1d4ed8", lw=2.8, marker="o", ms=3.5, label="Outbound Path (Seg 2: 90° -> 0°)")
    ax.plot(in_c["tas_ms"], in_c["nacelle_deg"], color="#dc2626", lw=2.2, ls="--", label="Inbound Reconversion (Seg 4: 0° -> 90°)")
    ax.plot(rp["tas_ms"], rp["nacelle_deg"], marker="*", ms=14, color="#f59e0b", mec="black", mew=1.2, zorder=9,
            label=f"Most Restrictive ({rp['tas_ms']:.0f} m/s, {rp['nacelle_deg']:.0f}°, {rp['governing_margin']})")

    # Legend patches for regimes
    legend_patches = [
        patches.Patch(fc="#bbf7d0", ec="#15803d", label=f"Feasible Corridor ({int((Z==0).sum())}/{Z.size} states)"),
        patches.Patch(fc="#fed7aa", ec="#c2410c", label="Wing Stall Boundary (Low Speed)"),
        patches.Patch(fc="#fecaca", ec="#dc2626", label="Advancing Tip-Mach > 0.85"),
        patches.Patch(fc="#fde68a", ec="#b45309", label="Rotor Blade Stall"),
        patches.Patch(fc="#e2e8f0", ec="#475569", label="Control / Trim Authority Limit"),
    ]
    h_lines, l_lines = ax.get_legend_handles_labels()
    ax.legend(handles=legend_patches + h_lines, loc="upper right", fontsize=7.1, framealpha=0.94)

    ax.set_title("Final Frozen TR-9 Speed–Nacelle Conversion Corridor (h = 500 m ISA, m = 6,800 kg)", fontsize=10.2)
    ax.set_xlabel("True Airspeed V_TAS [m/s]")
    ax.set_ylabel("Nacelle Tilt Angle θ_nac [deg] (90° = Hover, 0° = Airplane)")
    ax.set_xlim(-2, 122)
    ax.set_ylim(-3, 93)
    ax.grid(True, alpha=0.28, ls="--")

    for p in [res_dir / "fig_3_2_conversion_corridor.png", out_dir_extra / "fig_3_2_conversion_corridor.png"]:
        fig.savefig(p, dpi=260, bbox_inches="tight")
    plt.close(fig)


def plot_4_mission_histories(res_dir: Path, out_dir_extra: Path) -> None:
    """Generate individual Slide 19, Slide 20, Slide 22 panels AND combined notebook figures."""
    df = pd.read_csv(res_dir / "sec4_2_mission_telemetry.csv")
    meta = json.loads((res_dir / "mission_summary_metrics.json").read_text(encoding="utf-8"))
    t = df["time_min"]

    def _save_both(fig: plt.Figure, fname: str) -> None:
        fig.savefig(res_dir / fname, dpi=260, bbox_inches="tight")
        fig.savefig(out_dir_extra / fname, dpi=260, bbox_inches="tight")
        plt.close(fig)

    # --- Slide 19 Panel 1: Altitude vs Time ---
    fig, ax = plt.subplots(figsize=(4.6, 4.45), dpi=260)
    ax.plot(t, df["altitude_m"], color="#1d4ed8", lw=2.3, label="Altitude h(t)")
    ax.set_ylim(-150, 3450)
    _shade_segments(ax, df)
    ax.set_title("Altitude Profile vs Mission Time")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Altitude MSL [m]")
    ax.grid(True, alpha=0.3, ls="--")
    ax.legend(loc="lower center", fontsize=8.0)
    _save_both(fig, "fig_4_2a_p1_altitude.png")

    # --- Slide 19 Panel 2: Airspeed / Ground Speed & Cumulative Distance ---
    fig, ax = plt.subplots(figsize=(4.6, 4.45), dpi=260)
    ax_r = ax.twinx()
    ax.plot(t, df["tas_ms"], color="#0f766e", lw=2.2, label="True Airspeed TAS")
    ax.plot(t, df["gs_ms"], color="#14b8a6", lw=1.7, ls="--", label="Ground Speed GS (-4 m/s wind)")
    ax.set_ylim(-5, 132)
    _shade_segments(ax, df, add_labels=False)
    ax_r.plot(t, df["cum_dist_km"], color="#7c3aed", lw=2.2, label="Cum. Distance [km]")
    ax_r.set_ylabel("Cumulative Ground Distance [km]", color="#6d28d9")
    ax_r.tick_params(axis="y", labelcolor="#6d28d9")
    ax_r.set_ylim(-20, 840)
    ax.set_title("Speed & Cumulative Distance vs Time")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Speed [m/s]", color="#0f766e")
    ax.grid(True, alpha=0.3, ls="--")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax_r.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="center left", fontsize=7.4)
    _save_both(fig, "fig_4_2a_p2_speed_dist.png")

    # --- Slide 19 Panel 3: Gross Mass & Fuel Remaining ---
    fig, ax = plt.subplots(figsize=(4.6, 4.45), dpi=260)
    ax_r = ax.twinx()
    ax.plot(t, df["gross_mass_kg"], color="#1e293b", lw=2.2, label="Gross Mass [kg]")
    ax.set_ylim(5750, 6950)
    _shade_segments(ax, df, add_labels=False)
    ax_r.plot(t, df["fuel_rem_kg"], color="#dc2626", lw=2.2, label="Fuel Remaining [kg]")
    ax_r.axhline(meta["protected_reserve_kg"], color="#b45309", lw=1.6, ls="--", label=f"Reserve ({meta['protected_reserve_kg']:.0f} kg)")
    ax_r.set_ylabel("Fuel Remaining [kg]", color="#dc2626")
    ax_r.tick_params(axis="y", labelcolor="#dc2626")
    ax_r.set_ylim(0, 1220)
    ax.set_title("Aircraft Mass & Fuel State vs Time")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Gross Mass [kg]", color="#1e293b")
    ax.grid(True, alpha=0.3, ls="--")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax_r.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=7.3)
    _save_both(fig, "fig_4_2a_p3_mass_fuel.png")

    # --- Slide 20 Panel 1: Nacelle Angle & Rotor RPM ---
    fig, ax = plt.subplots(figsize=(4.6, 4.45), dpi=260)
    ax_r = ax.twinx()
    ax.plot(t, df["nacelle_deg"], color="#d97706", lw=2.3, label="Nacelle Angle θ_nac [deg]")
    ax.set_ylim(-5, 102)
    _shade_segments(ax, df)
    ax_r.plot(t, df["rpm"], color="#1d4ed8", lw=2.0, ls="--", label="Proprotor RPM")
    ax_r.set_ylabel("Proprotor Speed [RPM]", color="#1d4ed8")
    ax_r.tick_params(axis="y", labelcolor="#1d4ed8")
    ax_r.set_ylim(320, 575)
    ax.set_title("Nacelle Schedule & Proprotor RPM")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Nacelle Tilt θ_nac [deg]", color="#b45309")
    ax.grid(True, alpha=0.3, ls="--")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax_r.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="center right", fontsize=7.5)
    _save_both(fig, "fig_4_2b_p1_nacelle_rpm.png")

    # --- Slide 20 Panel 2: Power Required vs Power Available ---
    fig, ax = plt.subplots(figsize=(4.6, 4.45), dpi=260)
    ax.plot(t, df["p_eng_thermo_kW"], color="#94a3b8", lw=1.4, ls=":", label="Engine Thermo P_ISA(h)")
    ax.plot(t, df["p_avail_kW"], color="#dc2626", lw=2.0, ls="--", label="Effective Shaft P_avail")
    ax.plot(t, df["p_req_kW"], color="#0f766e", lw=2.3, label="Shaft Power Required P_req")
    ax.fill_between(t, df["p_req_kW"], df["p_avail_kW"], color="#bbf7d0", alpha=0.30, label="Available Power Margin")
    ax.set_ylim(0, 4600)
    _shade_segments(ax, df, add_labels=False)
    idx_pk = df["p_req_kW"].idxmax()
    ax.plot(df.loc[idx_pk, "time_min"], df.loc[idx_pk, "p_req_kW"], "o", ms=6.5, color="#dc2626", mec="black")
    ax.annotate(f"Peak {df.loc[idx_pk, 'p_req_kW']:.0f} kW", xy=(df.loc[idx_pk, "time_min"], df.loc[idx_pk, "p_req_kW"]),
                xytext=(df.loc[idx_pk, "time_min"] + 8, df.loc[idx_pk, "p_req_kW"] + 380), fontsize=7.5, fontweight="bold",
                arrowprops=dict(arrowstyle="->", lw=0.9))
    ax.set_title("Total Shaft Power Required vs Available")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Shaft Power [kW]")
    ax.grid(True, alpha=0.3, ls="--")
    ax.legend(loc="upper right", fontsize=7.2)
    _save_both(fig, "fig_4_2b_p2_power.png")

    # --- Slide 20 Panel 3: Key Aerodynamic & Feasibility Margins ---
    fig, ax = plt.subplots(figsize=(4.6, 4.45), dpi=260)
    ax.plot(t, df["rotor_stall_margin_deg"], color="#1d4ed8", lw=2.0, label="Rotor Stall Margin [deg]")
    ax.plot(t, df["wing_stall_margin_deg"], color="#0f766e", lw=1.9, ls="--", label="Wing Stall Margin [deg]")
    ax.plot(t, 100.0 * df["tip_mach_margin"], color="#7c3aed", lw=1.8, ls="-.", label="100 × Tip-Mach Margin")
    ax.axhline(0.0, color="#dc2626", lw=1.6, ls=":", label="Limit Boundary (0)")
    ax.set_ylim(-2.0, 26.0)
    _shade_segments(ax, df, add_labels=False)
    ax.set_title("Feasibility & Stall Margins vs Time")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Safety Margin [deg or 100×ΔM]")
    ax.grid(True, alpha=0.3, ls="--")
    ax.legend(loc="lower right", fontsize=7.3)
    _save_both(fig, "fig_4_2b_p3_margins.png")

    # --- Slide 22 Optional Plot: All Normalized Margins vs Time ---
    fig, ax = plt.subplots(figsize=(9.2, 2.95), dpi=260)
    ax.plot(t, 100.0 * df["norm_fuel_margin"], lw=1.9, color="#dc2626", label="Fuel Reserve Margin")
    ax.plot(t, 100.0 * df["norm_power_margin"], lw=1.9, color="#0f766e", label="Shaft Power Margin")
    ax.plot(t, 100.0 * df["norm_rotor_stall"], lw=1.9, color="#1d4ed8", label="Rotor Stall Margin")
    ax.plot(t, 100.0 * df["norm_tip_mach"], lw=1.8, ls="--", color="#7c3aed", label="Tip-Mach Margin")
    ax.plot(t, 100.0 * df["norm_wing_stall"], lw=1.8, ls="-.", color="#d97706", label="Wing Stall Margin")
    ax.plot(t, 100.0 * df["norm_control_margin"], lw=1.6, ls=":", color="#334155", label="Control Travel Margin")
    ax.axhline(0.0, color="black", lw=1.5, ls="--", label="Hard Limit (0%)")

    idx_min = df["min_norm_margin"].idxmin()
    ax.plot(df.loc[idx_min, "time_min"], 100.0 * df.loc[idx_min, "min_norm_margin"], marker="*", ms=12, color="#f59e0b", mec="black", zorder=8)
    ax.annotate(
        f"Global Min: {df.loc[idx_min, 'governing_margin']} ({100.0*df.loc[idx_min, 'min_norm_margin']:.1f}% @ {df.loc[idx_min, 'time_min']:.1f} min)",
        xy=(df.loc[idx_min, "time_min"], 100.0 * df.loc[idx_min, "min_norm_margin"]),
        xytext=(max(8.0, float(df.loc[idx_min, "time_min"]) - 48.0), 100.0 * float(df.loc[idx_min, "min_norm_margin"]) + 22.0),
        fontsize=8.0,
        fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.2", fc="#fef3c7", ec="#d97706"),
        arrowprops=dict(arrowstyle="->", lw=1.1),
    )
    ax.set_ylim(-5, 112)
    _shade_segments(ax, df, add_labels=False)
    ax.set_title("Normalized Aircraft Limit Margins Across Complete End-to-End Mission (0% = Limit Violation)", fontsize=9.8)
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Normalized Margin [%]")
    ax.grid(True, alpha=0.3, ls="--")
    ax.legend(loc="upper right", ncol=4, fontsize=7.2)
    _save_both(fig, "fig_4_3_normalized_margins.png")

    # --- Combined 6-Panel Notebook Summary Figure ---
    fig_nb, axes = plt.subplots(2, 3, figsize=(15.5, 8.8), dpi=200)
    axes[0, 0].plot(t, df["altitude_m"], color="#1d4ed8", lw=2.2)
    axes[0, 0].set_title("(a) Altitude Profile [m]")
    axes[0, 0].set_ylabel("Altitude [m]")

    axes[0, 1].plot(t, df["tas_ms"], color="#0f766e", lw=2.2, label="TAS [m/s]")
    axes[0, 1].plot(t, df["gs_ms"], color="#14b8a6", ls="--", lw=1.7, label="GS [m/s]")
    ax_d = axes[0, 1].twinx()
    ax_d.plot(t, df["cum_dist_km"], color="#7c3aed", lw=2.0, label="Dist [km]")
    ax_d.set_ylabel("Distance [km]", color="#6d28d9")
    axes[0, 1].set_title("(b) Airspeed, Ground Speed & Distance")
    axes[0, 1].set_ylabel("Speed [m/s]")
    axes[0, 1].legend(loc="center left", fontsize=7.5)

    axes[0, 2].plot(t, df["gross_mass_kg"], color="#1e293b", lw=2.1, label="Gross Mass [kg]")
    ax_f = axes[0, 2].twinx()
    ax_f.plot(t, df["fuel_rem_kg"], color="#dc2626", lw=2.1, label="Fuel [kg]")
    ax_f.axhline(meta["protected_reserve_kg"], color="#b45309", ls="--", lw=1.4, label="Reserve")
    ax_f.set_ylabel("Fuel Remaining [kg]", color="#dc2626")
    axes[0, 2].set_title("(c) Gross Mass & Fuel Remaining")
    axes[0, 2].set_ylabel("Gross Mass [kg]")

    axes[1, 0].plot(t, df["nacelle_deg"], color="#d97706", lw=2.2, label="θ_nac [deg]")
    ax_rpm = axes[1, 0].twinx()
    ax_rpm.plot(t, df["rpm"], color="#1d4ed8", ls="--", lw=1.9, label="RPM")
    ax_rpm.set_ylabel("RPM", color="#1d4ed8")
    axes[1, 0].set_title("(d) Nacelle Tilt & Proprotor RPM")
    axes[1, 0].set_ylabel("Nacelle [deg]")

    axes[1, 1].plot(t, df["p_avail_kW"], color="#dc2626", ls="--", lw=1.9, label="P_avail [kW]")
    axes[1, 1].plot(t, df["p_req_kW"], color="#0f766e", lw=2.2, label="P_req [kW]")
    axes[1, 1].set_title("(e) Shaft Power Required vs Available [kW]")
    axes[1, 1].set_ylabel("Power [kW]")
    axes[1, 1].legend(loc="upper right", fontsize=7.5)

    axes[1, 2].plot(t, df["rotor_stall_margin_deg"], color="#1d4ed8", lw=2.0, label="Rotor Stall [°]")
    axes[1, 2].plot(t, df["wing_stall_margin_deg"], color="#0f766e", ls="--", lw=1.9, label="Wing Stall [°]")
    axes[1, 2].plot(t, 100.0 * df["tip_mach_margin"], color="#7c3aed", ls="-.", lw=1.8, label="100×ΔM_tip")
    axes[1, 2].axhline(0.0, color="red", ls=":")
    axes[1, 2].set_title("(f) Key Feasibility Margins")
    axes[1, 2].set_ylabel("Margin")
    axes[1, 2].legend(loc="lower right", fontsize=7.5)

    for ax_i in axes.ravel():
        _shade_segments(ax_i, df, add_labels=False)
        ax_i.set_xlabel("Mission Time [min]")
        ax_i.grid(True, alpha=0.3, ls="--")

    fig_nb.suptitle(
        f"TR-9 End-to-End Representative Mission ({meta['total_distance_km']:.0f} km, {meta['total_time_min']:.1f} min, Payload = 1,500 kg, 4 m/s Headwind)",
        fontsize=12.5,
        fontweight="bold",
    )
    fig_nb.tight_layout(rect=[0, 0, 1, 0.96])
    _save_both(fig_nb, "fig_4_2_combined_mission_dashboard.png")


def make_all_plots() -> None:
    pkg_dir = Path(__file__).resolve().parents[1]
    res_dir = pkg_dir / "results"
    extra_dir = ROOT / "milestone3_figures"
    res_dir.mkdir(parents=True, exist_ok=True)
    extra_dir.mkdir(parents=True, exist_ok=True)

    cfg = load_config(pkg_dir / "config" / "aircraft_final.yaml")
    plot_1_1_schematic(cfg, [res_dir / "fig_1_1_final_aircraft_schematic.png", extra_dir / "fig_1_1_final_aircraft_schematic.png"])
    plot_2_2_workflow_diagram([res_dir / "fig_2_2_integrated_workflow.png", extra_dir / "fig_2_2_integrated_workflow.png"])
    plot_3_1_hover_and_cruise(res_dir, extra_dir)
    plot_3_2_conversion_corridor(res_dir, extra_dir)
    plot_4_mission_histories(res_dir, extra_dir)
    print("All Milestone 3 figures generated successfully in:", res_dir, "and", extra_dir)


if __name__ == "__main__":
    make_all_plots()
