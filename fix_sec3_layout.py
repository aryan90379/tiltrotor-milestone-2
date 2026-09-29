import nbformat
from nbclient import NotebookClient
import urllib.request
import subprocess

nb_path = "/Users/apple/Documents/courses/rotary/milestone 2/milestone_2.ipynb"
nb = nbformat.read(nb_path, as_version=4)

src4 = nb.cells[4].source

marker = "    fig = plt.figure("
idx = src4.rfind("fig = plt.figure(")
# Walk back to start of line
line_start = src4.rfind("\n", 0, idx) + 1
indent = src4[line_start:idx]
assert idx != -1, "Could not find fig = plt.figure in Cell 4"

new_plotting_code = """    fig = plt.figure(figsize=(19.5, 13.0), dpi=140)
    gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 0.92], hspace=0.46, wspace=0.42,
                          left=0.045, right=0.955, top=0.87, bottom=0.075)
    psi_grid, r_grid = np.meshgrid(np.Append(res_edge["psi_rad"], res_edge["psi_rad"][0] + 2*np.pi) if hasattr(np, 'Append') else np.append(res_edge["psi_rad"], res_edge["psi_rad"][0] + 2*np.pi), res_edge["r_m"])
    def close_az(arr):
        return np.hstack([arr, arr[:, :1]])

    from matplotlib.lines import Line2D

    def style_rotor_polar(ax, title_str):
        ax.set_theta_zero_location("S")
        ax.set_theta_direction(1)
        ax.set_xticks(np.radians([0, 90, 180, 270]))
        ax.set_xticklabels([
            "ψ = 0°\\n(AFT / TAIL)",
            "ψ = 90°\\n(ADVANCING\\n+V_∞ + Ωr)",
            "ψ = 180° (NOSE / FREESTREAM V_∞ ↓)",
            "ψ = 270°\\n(RETREATING\\nΩr - V_∞)"
        ], fontsize=8.5, fontweight="bold", color="#0f172a")
        ax.tick_params(axis="x", pad=8)
        ax.set_yticks([1.5, 3.0, 4.58])
        ax.set_yticklabels(["r=1.5m", "r=3.0m", "Tip R=4.58m"], fontsize=7.8, fontweight="bold", color="#1e293b")
        ax.set_rlabel_position(135)
        ax.set_title(title_str, pad=28, fontweight="bold", fontsize=10.5, color="#0f172a")

    # -------------------------------------------------------------------------
    # PANEL 1: dT/dr [N/m] Polar Map
    # -------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0], projection="polar")
    pcm1 = ax1.pcolormesh(psi_grid, r_grid, close_az(res_edge["dT_dr"]), cmap="turbo", shading="auto")
    ax1.contour(psi_grid, r_grid, close_az(res_edge["U_T"]), levels=[0.0], colors="white", linewidths=2.2, linestyles="--")
    style_rotor_polar(
        ax1,
        "Sec 3.3(a): Sectional Lift Load dT/dr [N/m]\\n"
        f"(V_∞ = 45 m/s, μ = {res_edge['mu']:.2f}, θ_nac = 80° Edgewise)"
    )
    cb1 = plt.colorbar(pcm1, ax=ax1, pad=0.15, fraction=0.042, shrink=0.82)
    cb1.set_label("Blade Sectional Lift dT/dr [N/m]", fontweight="bold", fontsize=9)
    ax1.annotate(
        "Peak Advancing Lift\\n(High U_T = Ωr + V_∞ sinψ)",
        xy=(np.radians(85), 3.8), xytext=(np.radians(48), 2.55),
        arrowprops=dict(arrowstyle="->", color="white", lw=1.8),
        fontsize=7.8, fontweight="bold", color="#0f172a", ha="center",
        bbox=dict(boxstyle="round,pad=0.25", fc="#fef08a", ec="#ca8a04", alpha=0.95)
    )
    ax1.annotate(
        "Reverse-Flow Circle (U_T ≤ 0)\\nDiameter = μ·R = 1.15 m",
        xy=(np.radians(270), 0.92), xytext=(np.radians(228), 2.95),
        arrowprops=dict(arrowstyle="->", color="white", lw=1.8),
        fontsize=7.8, fontweight="bold", color="white", ha="center",
        bbox=dict(boxstyle="round,pad=0.25", fc="#be123c", ec="white", alpha=0.94)
    )
    leg1 = [Line2D([0], [0], color="#be123c", lw=2.2, ls="--", label="Reverse-Flow Boundary (U_T = 0 m/s)")]
    ax1.legend(handles=leg1, loc="upper center", bbox_to_anchor=(0.5, -0.14), fontsize=8.0, frameon=True, facecolor="#f8fafc", edgecolor="#94a3b8")

    # -------------------------------------------------------------------------
    # PANEL 2: Angle of Attack α(r, ψ) [deg] Polar Map
    # -------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1], projection="polar")
    pcm2 = ax2.pcolormesh(psi_grid, r_grid, np.clip(close_az(res_edge["alpha_deg"]), -15, 25), cmap="RdYlBu_r", shading="auto")
    ax2.contour(psi_grid, r_grid, close_az(res_edge["U_T"]), levels=[0.0], colors="#e11d48", linewidths=2.2, linestyles="--")
    style_rotor_polar(
        ax2,
        "Sec 3.2 & 3.3: Blade Angle of Attack α(r, ψ) [deg]\\n"
        "& Reverse-Flow Circle (U_T = Ωr + V_∞ cosθ_n sinψ ≤ 0)"
    )
    cb2 = plt.colorbar(pcm2, ax=ax2, pad=0.15, fraction=0.042, shrink=0.82)
    cb2.set_label("Local Section AoA α(r, ψ) [deg]", fontweight="bold", fontsize=9)
    ax2.annotate(
        "Retreating High-α Zone\\n(Low dynamic pressure →\\nhigh inflow angle φ)",
        xy=(np.radians(270), 2.3), xytext=(np.radians(235), 3.15),
        arrowprops=dict(arrowstyle="->", color="#0f172a", lw=1.6),
        fontsize=7.6, fontweight="bold", color="#0f172a", ha="center",
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#e11d48", alpha=0.94)
    )
    leg2 = [Line2D([0], [0], color="#e11d48", lw=2.2, ls="--", label="Reverse-Flow Circle (U_T ≤ 0: flow from TE to LE)")]
    ax2.legend(handles=leg2, loc="upper center", bbox_to_anchor=(0.5, -0.14), fontsize=8.0, frameon=True, facecolor="#f8fafc", edgecolor="#94a3b8")

    # -------------------------------------------------------------------------
    # PANEL 3: Local Mach Number M(r, ψ) & EXPLANATION OF THE '∞' STALL CONTOUR
    # -------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2], projection="polar")
    pcm3 = ax3.pcolormesh(psi_grid, r_grid, close_az(res_edge["mach"]), cmap="magma", shading="auto")
    ax3.contour(psi_grid, r_grid, close_az(res_edge["stall_mask"].astype(float)), levels=[0.5], colors="#22c55e", linewidths=2.6)
    ax3.contour(psi_grid, r_grid, close_az(res_edge["U_T"]), levels=[0.0], colors="#38bdf8", linewidths=2.0, linestyles="--")
    style_rotor_polar(
        ax3,
        "Sec 3.3(b): Local Mach M(r, ψ) & Why Stall Forms an '∞' Shape\\n"
        f"(Peak Tip M = {res_edge['peak_mach']:.2f} at ψ=90°, Stall Area = {res_edge['stall_area_frac']*100:.1f}%)"
    )
    cb3 = plt.colorbar(pcm3, ax=ax3, pad=0.15, fraction=0.042, shrink=0.82)
    cb3.set_label("Local Section Mach Number M(r, ψ) [-]", fontweight="bold", fontsize=9)
    ax3.annotate(
        "WHY '∞' (FIGURE-8) STALL CONTOUR?\\n"
        "• Left Lobe (ψ≈270°): Retreating reverse-flow stall\\n"
        "• Right Lobe (ψ≈90°): Inboard root high-twist stall\\n"
        "  (θ_tw = -30° gives root θ = 33.5° → |α| ≥ 15.8°)",
        xy=(np.radians(90), 1.25), xytext=(np.radians(210), 2.95),
        arrowprops=dict(arrowstyle="->", color="#22c55e", lw=2.2),
        fontsize=7.3, fontweight="bold", color="#0f172a", ha="center",
        bbox=dict(boxstyle="round,pad=0.28", fc="#dcfce7", ec="#15803d", lw=1.5, alpha=0.97)
    )
    ax3.annotate(
        f"Advancing Tip Compressibility\\nPeak M = {res_edge['peak_mach']:.2f} at ψ = 90°",
        xy=(np.radians(90), 4.45), xytext=(np.radians(40), 2.95),
        arrowprops=dict(arrowstyle="->", color="#fde047", lw=1.8),
        fontsize=7.5, fontweight="bold", color="#0f172a", ha="center",
        bbox=dict(boxstyle="round,pad=0.25", fc="#fef9c3", ec="#ca8a04", alpha=0.95)
    )
    leg3 = [
        Line2D([0], [0], color="#22c55e", lw=2.6, ls="-", label="Green '∞' Loop = Stall Boundary (|α| ≥ α_stall = 15.8°)"),
        Line2D([0], [0], color="#38bdf8", lw=2.0, ls="--", label="Cyan Dashed Circle = Reverse-Flow Boundary (U_T ≤ 0)")
    ]
    ax3.legend(handles=leg3, loc="upper center", bbox_to_anchor=(0.5, -0.13), fontsize=7.6, frameon=True, facecolor="#f8fafc", edgecolor="#94a3b8")

    # -------------------------------------------------------------------------
    # PANEL 4 (Bottom-Left): Section 3.1 Hover Regression
    # -------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(r_1d, dT_1d, "k-", lw=2.8, label="Milestone 1 1D Annulus BEMT (Hover V_∞ = 0 m/s)")
    ax4.plot(r_2d, dT_2d, "r--", lw=2.2, label="Milestone 2 2D Azimuthal BEMT (ψ-Averaged)")
    ax4.fill_between(r_1d, dT_1d, color="#3b82f6", alpha=0.10, label="Integrated Blade Span Thrust Area")
    ax4.set_xlabel("Radial Blade Station r [m] (Root Cutout r_0 = 0.46 m → Tip R = 4.58 m)", fontweight="bold", fontsize=9.2)
    ax4.set_ylabel("Sectional Thrust Loading dT/dr [N/m]", fontweight="bold", fontsize=9.2)
    ax4.set_title(
        f"Sec 3.1: Hover Consistency Verification (V_∞ = 0 m/s, θ_nac = 90°)\\n"
        f"Exact Match Between 1D & 2D Solvers (Max ΔT = {rel_err_T:.2e}%)",
        fontweight="bold", fontsize=10.2
    )
    ax4.grid(True, alpha=0.35, linestyle="--")
    ax4.legend(fontsize=8.3, loc="upper left", frameon=True, facecolor="white", edgecolor="#cbd5e1")

    # -------------------------------------------------------------------------
    # PANEL 5 (Bottom-Middle): Section 3.4 Glauert Skewed-Wake Transition
    # -------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(tnac_arr, vi_arr, "b-o", lw=2.4, label="Mean Induced Velocity v_i0 [m/s] (Left Axis)")
    ax5_r = ax5.twinx()
    ax5_r.plot(tnac_arr, p_arr, "r-s", lw=2.2, label="Rotor Shaft Power P_rotor [kW] (Right Axis)")
    ax5.set_xlabel("Nacelle Tilt Angle θ_nac [deg] (90° = Helicopter Edgewise → 0° = Airplane Axial)", fontweight="bold", fontsize=9.0)
    ax5.set_ylabel("Mean Induced Inflow Velocity v_i0 [m/s]", color="b", fontweight="bold", fontsize=9.2)
    ax5_r.set_ylabel("Rotor Shaft Power P_rotor [kW]", color="r", fontweight="bold", fontsize=9.2)
    ax5.set_title(
        "Sec 3.4: Glauert Skewed-Wake Inflow Transition (V_∞ = 35 m/s)\\n"
        "Continuous Evolution from Edgewise Momentum to Axial Propeller Inflow",
        fontweight="bold", fontsize=10.2
    )
    ax5.invert_xaxis()
    ax5.grid(True, alpha=0.35, linestyle="--")
    lines5, labels5 = ax5.get_legend_handles_labels()
    lines5r, labels5r = ax5_r.get_legend_handles_labels()
    ax5.legend(lines5 + lines5r, labels5 + labels5r, fontsize=8.1, loc="upper right", frameon=True, facecolor="white")

    # -------------------------------------------------------------------------
    # PANEL 6 (Bottom-Right): Azimuthal Harmonics & Hub Moment Origin
    # -------------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2])
    psi_deg = res_edge["psi_deg"]
    blade1_T = np.trapezoid(res_edge["dT_dr"], res_edge["r_m"], axis=0) / 1000.0
    ax6.plot(psi_deg, blade1_T, color="#0f766e", lw=2.5, label="Single-Blade Lift T_blade(ψ) [kN] (1P Lateral Imbalance)")
    ax6.axhline(np.mean(blade1_T), color="#dc2626", ls="--", lw=1.8, label=f"Azimuth-Mean Single-Blade Lift = {np.mean(blade1_T):.2f} kN")
    ax6.axvspan(45, 135, color="#fef08a", alpha=0.35, label="Advancing Sector (ψ≈90°: Peak Dynamic Pressure)")
    ax6.axvspan(225, 315, color="#fecdd3", alpha=0.35, label="Retreating Sector (ψ≈270°: Reverse Flow + Stall)")
    ax6.set_xlabel("Blade Azimuth Angle ψ [deg] (0°=Tail, 90°=Advancing, 180°=Nose, 270°=Retreating)", fontweight="bold", fontsize=9.0)
    ax6.set_ylabel("Single-Blade Integrated Lift T_blade(ψ) [kN]", fontweight="bold", fontsize=9.2)
    ax6.set_title(
        "Sec 3.2: Why Uncycled Edgewise Flight Generates Hub Roll Moment\\n"
        f"(Advancing vs Retreating Lift Asymmetry → M_x = {res_edge['M_roll_Nm']/1000:.1f} kN·m)",
        fontweight="bold", fontsize=10.2
    )
    ax6.set_xticks([0, 90, 180, 270, 360])
    ax6.set_xticklabels(["0° (Tail)", "90° (Adv)", "180° (Nose)", "270° (Ret)", "360° (Tail)"], fontsize=8.2)
    ax6.legend(fontsize=7.8, loc="upper right", frameon=True, facecolor="white")
    ax6.grid(True, alpha=0.35, linestyle="--")

    fig.suptitle(
        "Milestone 2 — Section 3: Edgewise Aeromechanics, Reverse-Flow Circle & '∞' Two-Lobe Stall Boundary Verification\\n"
        "Definitions: V_∞ = Freestream Airspeed [m/s] | μ = V_∞ cos(θ_nac)/(ΩR) = Edgewise Advance Ratio | ψ = Blade Azimuth (0°=Tail, 90°=Advancing, 180°=Nose, 270°=Retreating)",
        fontsize=12.5, fontweight="bold", y=0.975, color="#0f172a"
    )
    fig.text(
        0.5, 0.010,
        "Figure Guide for Slides: Top Polar Plots view the counter-clockwise proprotor disk from above with freestream V_∞ blowing from top (ψ=180° Nose) to bottom (ψ=0° Tail).\\n"
        "In Sec 3.3(b), the green '∞' (figure-8) contour marks the Blade Stall Boundary (|α| ≥ 15.8°): its left lobe is Retreating Reverse-Flow Stall (ψ≈270°) and its right lobe is Inboard Root High-Twist Stall (ψ≈90°, where θ_tw = -30° gives root pitch 33.5°).",
        ha="center", va="bottom", fontsize=9.0, fontweight="bold", color="#1e293b",
        bbox=dict(boxstyle="round,pad=0.35", fc="#f1f5f9", ec="#64748b", lw=1.2)
    )
    plt.savefig("section3_verification.png", bbox_inches="tight", dpi=140)
    plt.show()
    return {"rel_err_T_pct": rel_err_T, "rel_err_Q_pct": rel_err_Q, "edgewise_case": res_edge}
"""

# Re-indent every line inside run_section3_verification_suite by adding 4 extra spaces if indent has 8 spaces
extra = "    " if len(indent) == 8 else ""
indented_lines = []
for ln in new_plotting_code.splitlines():
    if ln.startswith("    return "):
        indented_lines.append(ln)
    elif ln.strip() == "":
        indented_lines.append("")
    else:
        indented_lines.append(extra + ln)
indented_lines.append("\nSEC3_RESULTS = run_section3_verification_suite()\n")

nb.cells[4].source = src4[:line_start] + "\n".join(indented_lines)
nbformat.write(nb, nb_path)
print("Updated Cell 4 in milestone_2.ipynb. Executing notebook...")

client = NotebookClient(nb, timeout=600, kernel_name="python3", resources={"metadata": {"path": "/Users/apple/Documents/courses/rotary/milestone 2"}})
client.execute()
nbformat.write(nb, nb_path)
print("Notebook executed and saved!")
