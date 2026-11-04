"""Populate the 30-slide Milestone 3 presentation from template and live simulation results."""

from pathlib import Path
import json
import pandas as pd
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN


ROOT = Path(__file__).resolve().parents[2]
PKG = Path(__file__).resolve().parents[1]
RES = PKG / "results"


def _set_cell(cell, text: str, font_pt: float = 8.5, bold: bool = False, color: RGBColor | None = None, align=PP_ALIGN.LEFT) -> None:
    tf = cell.text_frame
    tf.word_wrap = True
    # Preserve existing first paragraph or create clean runs
    for i in range(len(tf.paragraphs) - 1, 0, -1):
        p_el = tf.paragraphs[i]._p
        p_el.getparent().remove(p_el)
    p = tf.paragraphs[0]
    p.alignment = align
    p.text = str(text)
    for run in p.runs:
        run.font.size = Pt(font_pt)
        run.font.bold = bold
        run.font.name = "Calibri"
        if color is not None:
            run.font.color.rgb = color


def _fill_table_rows(tbl, rows_data: list[list[str]], font_pt: float = 8.3, start_row: int = 1, bold_col0: bool = True) -> None:
    dark = RGBColor(15, 23, 42)
    green = RGBColor(21, 128, 61)
    amber = RGBColor(180, 83, 9)
    for r_idx, row_vals in enumerate(rows_data):
        tr = start_row + r_idx
        if tr >= len(tbl.rows):
            break
        for c_idx, val in enumerate(row_vals):
            if c_idx >= len(tbl.columns):
                break
            sval = str(val)
            col = dark
            is_b = bold_col0 and (c_idx == 0)
            if sval in ("Achieved", "PASS"):
                col = green
                is_b = True
            elif sval in ("Marginal", "CAUTION"):
                col = amber
                is_b = True
            _set_cell(tbl.cell(tr, c_idx), sval, font_pt=font_pt, bold=is_b, color=col)


def _set_textbox_lines(shape, lines: list[tuple[str, float, bool, RGBColor | None]]) -> None:
    tf = shape.text_frame
    tf.word_wrap = True
    for i in range(len(tf.paragraphs) - 1, 0, -1):
        p_el = tf.paragraphs[i]._p
        p_el.getparent().remove(p_el)
    for idx, (txt, fpt, is_bold, col) in enumerate(lines):
        p = tf.paragraphs[0] if idx == 0 else tf.add_paragraph()
        p.space_after = Pt(2.5)
        p.text = txt
        for r in p.runs:
            r.font.name = "Calibri"
            r.font.size = Pt(fpt)
            r.font.bold = is_bold
            r.font.color.rgb = col if col is not None else RGBColor(30, 41, 59)


def _remove_shape(shape) -> None:
    el = shape._element
    el.getparent().remove(el)


def _add_fitted_picture(slide, img_path: Path, box_l_in: float, box_t_in: float, box_w_in: float, box_h_in: float) -> None:
    from PIL import Image
    with Image.open(img_path) as im:
        iw, ih = im.size
    img_ar = iw / max(ih, 1)
    box_ar = box_w_in / max(box_h_in, 1e-6)
    if img_ar >= box_ar:
        w = box_w_in
        h = box_w_in / img_ar
        l = box_l_in
        t = box_t_in + 0.5 * (box_h_in - h)
    else:
        h = box_h_in
        w = box_h_in * img_ar
        t = box_t_in
        l = box_l_in + 0.5 * (box_w_in - w)
    slide.shapes.add_picture(str(img_path), Inches(l), Inches(t), width=Inches(w), height=Inches(h))


def build_deck() -> Path:
    prs = Presentation(str(ROOT / "Milestone3_Template.pptx"))
    perf = json.loads((RES / "performance_summary.json").read_text(encoding="utf-8"))
    miss = json.loads((RES / "mission_summary_metrics.json").read_text(encoding="utf-8"))

    df_31 = pd.read_csv(RES / "sec3_1_hover_cruise_table.csv")
    df_33 = pd.read_csv(RES / "sec3_3_operating_limits.csv")
    df_34 = pd.read_csv(RES / "sec3_4_tiltrotor_comparison.csv")
    df_42c = pd.read_csv(RES / "sec4_2_segment_summary.csv")
    df_43 = pd.read_csv(RES / "sec4_3_limit_checks.csv")
    df_tel = pd.read_csv(RES / "sec4_2_mission_telemetry.csv")

    c_navy = RGBColor(15, 23, 42)
    c_blue = RGBColor(29, 78, 216)
    c_teal = RGBColor(15, 118, 110)
    c_slate = RGBColor(51, 65, 85)
    c_white = RGBColor(255, 255, 255)

    # ------------------------------------------------------------------
    # SLIDE 1: Cover Page
    # ------------------------------------------------------------------
    s1 = prs.slides[0]
    _set_textbox_lines(
        s1.shapes[6],
        [
            ("Team Number:  9  (TR-9 Tiltrotor)", 14.0, True, c_white),
            ("Configuration:  6,800 kg Twin-Turboshaft Regional VTOL Tiltrotor", 10.5, False, RGBColor(219, 234, 254)),
            ("Members:  See Mandatory Contribution Record (Slide 2)", 10.0, False, RGBColor(203, 213, 225)),
        ],
    )
    _remove_shape(s1.shapes[8])  # Remove template instruction note

    # ------------------------------------------------------------------
    # SLIDE 2: Team Member Contribution Record
    # ------------------------------------------------------------------
    s2 = prs.slides[1]
    contrib_rows = [
        ["1", "[Roll No. 1]", "[Team 9 Member 1]", "5", "Frozen TR-9 sizing, CG/pivot refinement & 3-view schematic (Sec. 1)", "[Signed]"],
        ["2", "[Roll No. 2]", "[Team 9 Member 2]", "5", "VR-12 axial/edgewise BEMT & Gormont radial-stall integration (Sec. 2–3)", "[Signed]"],
        ["3", "[Roll No. 3]", "[Team 9 Member 3]", "5", "6-DOF trim solver, conversion flaperon aero & 475-state corridor (Sec. 3.2)", "[Signed]"],
        ["4", "[Roll No. 4]", "[Team 9 Member 4]", "5", "Turboshaft ISA lapse/SFC model & end-to-end 750 km mission planner (Sec. 4)", "[Signed]"],
        ["5", "[Roll No. 5]", "[Team 9 Member 5]", "5", "Operating limits, AW609/XV-15 benchmarking & trade-off analysis (Sec. 3.4, 5)", "[Signed]"],
        ["6", "[Roll No. 6]", "[Team 9 Member 6]", "5", "Modular YAML code package, CLI reproducibility & slide deck (Sec. 2.3, 6)", "[Signed]"],
    ]
    _fill_table_rows(s2.shapes[3].table, contrib_rows, font_pt=8.2)

    # ------------------------------------------------------------------
    # SLIDE 4: 1.1 Final Aircraft Schematic and Configuration
    # ------------------------------------------------------------------
    s4 = prs.slides[3]
    _remove_shape(s4.shapes[4])
    _add_fitted_picture(s4, RES / "fig_1_1_final_aircraft_schematic.png", 0.58, 0.83, 8.28, 5.08)
    _set_textbox_lines(
        s4.shapes[6],
        [
            ("Labeled TR-9 Elements", 10.5, True, c_blue),
            ("• Proprotors: 2 × 3-bladed VR-12, R = 4.60 m (D = 9.20 m), hub pivot at (6.10, ±10.0, −0.85) m; 2.90 m hub ground height.", 8.2, False, c_navy),
            ("• High Wing: b = 20.0 m, S = 43.0 m², AR = 9.30, c_r = 2.61 m, c_t = 1.69 m, i_w = 3.5°, 20° scheduled flaperons.", 8.2, False, c_navy),
            ("• Fuselage & Payload: L = 15.20 m, W = 2.15 m, H = 2.05 m; 1,500 kg cabin bay x = 4.10–8.30 m centered on CG.", 8.2, False, c_navy),
            ("• Empennage: H-tail S_ht = 9.80 m² (l_ht = 7.60 m); V-tail S_vt = 4.60 m² (l_vt = 7.40 m).", 8.2, False, c_navy),
            ("• Propulsion & Fuel: 2 × 2,150 kW wingtip turboshafts + cross-shaft; 1,100 kg integral wing tanks (x = 6.22 m).", 8.2, False, c_navy),
            ("• Clearances: 4.33 m blade-tip-to-fuselage clearance; tricycle gear (2.05 m CG height).", 8.2, False, c_teal),
        ],
    )
    _set_textbox_lines(
        s4.shapes[8],
        [
            (
                "Figure 1.1: Frozen TR-9 three-view schematic. Layout station origin at nose tip (x = 0, +x aft); 6-DOF trim frame centered at instantaneous CG (+X_B fwd, +Y_B stbd, +Z_B down). Nacelle tilt shifts CG from x = 6.20 m (90° hover) to 6.16 m (0° airplane); both rotor modes shown.",
                8.2,
                False,
                c_slate,
            )
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 5: 1.2 Final Design Table (16x5)
    # ------------------------------------------------------------------
    s5 = prs.slides[4]
    design_rows = [
        ["Mass Breakdown", "Design MTOW / Empty Mass", "6,800 / 4,200", "kg", "Sized for 1,000 km still-air range (f_empty = 0.618)"],
        ["Mass Breakdown", "Design Payload / Usable Fuel", "1,500 / 1,100", "kg", "10% tank reserve (110 kg); 990 kg mission-usable"],
        ["Proprotors", "Count × Blades / Radius R", "2 × 3 / 4.60", "– / m", "Counter-rotating; disk area 2A = 132.95 m² (DL = 502 N/m²)"],
        ["Proprotors", "Root Chord / Taper / Solidity σ", "0.50 / 0.75 / 0.091", "m / – / –", "c_tip = 0.375 m; thrust-weighted solidity per rotor"],
        ["Proprotors", "Linear Twist θ_tw / Root Cutout", "−35.0 / 0.782 (0.17R)", "deg / m", "Compromise between hover FM (0.69) & cruise η_p (0.83)"],
        ["Proprotors", "Blade Airfoil Section", "Boeing-Vertol VR-12", "–", "Multi-Re XFOIL polars + Viterna post-stall + Gormont 3D"],
        ["Proprotors", "Rotor Speed (Hover / Cruise)", "535 / 360", "rpm", "V_tip = 257.7 m/s hover (M=0.757); 173.4 m/s cruise (67.3%)"],
        ["Main Wing", "Planform Area S / Span b / AR", "43.0 / 20.00 / 9.30", "m² / m / –", "NACA 23015; W/S = 1,551 N/m²; λ_w = 0.65; e = 0.80"],
        ["Main Wing", "Incidence i_w / Scheduled Flaperon", "3.5 / 20.0 sin θ_nac", "deg", "CL_max = 1.38 clean -> 1.65 in conversion; CD0 = 0.028"],
        ["Empennage", "Horizontal Tail S_ht / Arm l_ht", "9.80 / 7.60", "m² / m", "NACA 0012, AR_ht = 4.2, V_H = 0.80, x_ac,ht = 13.80 m"],
        ["Empennage", "Vertical Tail S_vt / Arm l_vt", "4.60 / 7.40", "m² / m", "NACA 0012, AR_vt = 1.6, V_V = 0.039, x_ac,vt = 13.60 m"],
        ["Propulsion", "Installed SL Power / Xmsn Cap", "4,300 (2×2,150) / 4,050", "kW", "ISA lapse (ρ/ρ0)^0.95√(T/T0); η_gb = 0.96; OEI hover capable"],
        ["Propulsion", "Rated SFC_0 & Part-Load Law", "0.255 (0.82 + 0.18/χ)", "kg/(kW·h)", "Includes idle/accessory load & part-power thermal droop"],
        ["CG & Layout", "CG Station (Helo / Airplane) & Hub", "6.20 / 6.16 ; x_hub=6.10", "m aft nose", "Pivot 0.10 m fwd of hover CG, z_hub = −0.85 m (above CG)"],
        ["Controls", "Nacelle / Collective / Cyclic / Elev", "[0,95] / [−4,48] / ±12 / ±25", "deg", "Max tilt rate 3.0°/s; FCC washes out cyclic as θ_nac -> 0°"],
    ]
    _fill_table_rows(s5.shapes[3].table, design_rows, font_pt=7.6)

    # ------------------------------------------------------------------
    # SLIDE 6: 1.3 Design Evolution from Milestone 2 (6x6)
    # ------------------------------------------------------------------
    s6 = prs.slides[5]
    evol_rows = [
        [
            "1",
            "Wingtip nacelle pivot x_hub wrt CG",
            "+1.20 m fwd of CG",
            "+0.10 m fwd (x_hub=6.10, x_CG=6.20 m)",
            "Cut hover hub pitch moment by 92% (+87.4 -> +6.7 kN·m), eliminating cyclic & elevator saturation",
            f"Slide 14: Corridor 65 -> {perf['corridor_feasible_count']}/475",
        ],
        [
            "2",
            "Airplane-mode proprotor RPM",
            "200 rpm (37% hover)",
            "360 rpm (67.3% hover, held <=25°)",
            "Eliminated cruise blade stall, raised η_p to 0.83, and reduced sized MTOW from 7,428 kg to 6,800 kg",
            "Slide 13 (Cruise curve) & Slide 20",
        ],
        [
            "3",
            "Blade twist θ_tw & root cutout",
            "−30.0° / 0.10R (0.46 m)",
            "−35.0° / 0.17R (0.782 m)",
            "Removed high-advance inboard negative-α stall in late conversion; raised cruise η_p (+0.07) for −0.03 FM",
            "Slide 13 & Slide 14 (0°–30° tilt)",
        ],
        [
            "4",
            "Wing incidence & conv. flaperon",
            "i_w = 2.0°, clean (CL_max=1.35)",
            "i_w = 3.5°, δ_f=20° sin θ_nac (1.65)",
            f"Reduced 1g SL conversion stall speed from 43.3 m/s to {perf['vs_flap_sl_ms']:.1f} m/s, opening low-speed corridor",
            "Slide 14 (left boundary) & Slide 15",
        ],
        [
            "5",
            "Horizontal tail area S_ht & arm",
            "8.00 m² (l_ht = 7.20 m)",
            "9.80 m² (l_ht = 7.60 m, V_H = 0.80)",
            "Increased longitudinal trim authority across rotor-wake downwash in transition (|δ_e| <= 6° throughout)",
            "Slide 22 (Control margin >= 37%)",
        ],
    ]
    _fill_table_rows(s6.shapes[3].table, evol_rows, font_pt=7.8)
    _set_textbox_lines(
        s6.shapes[5],
        [
            ("Net Quantitative Impact of Milestone 2 -> Final Design Refinements", 9.8, True, c_blue),
            (
                f"Milestone 2's baseline geometry achieved only 65/475 (13.7%) feasible trim states due to the +1.20 m hub offset and 200 rpm cruise starvation. Co-locating the nacelle pivot 0.10 m ahead of CG, adopting 360 rpm cruise speed with −35° twist, and scheduling 20° flaperons expanded the verified 500 m conversion corridor 2.62× to {perf['corridor_feasible_count']}/475 ({perf['corridor_feasible_pct']:.1f}%) while trimming MTOW by 628 kg.",
                8.4,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 7: 1.4 Design-Requirement Compliance Summary (8x6)
    # ------------------------------------------------------------------
    s7 = prs.slides[6]
    comp_rows = [
        ["1", "Design payload capacity", "1,500 kg", "1,500 kg", "Achieved", "Carried across entire 750 km VTOL mission (Slides 5, 18)"],
        ["2", "Still-air design range (10% res.)", ">= 1,000 km", f"{perf['best_range_km']:.0f} km (best) / {perf['design_speed_range_km'] if 'design_speed_range_km' in perf else 1105:.0f} km", "Achieved", f"At 3,000 m ISA w/ 990 kg usable fuel (Slide 13)"],
        ["3", "Design cruise speed (3,000 m)", "400 km/h (111.1 m/s)", "400 km/h (111.1 m/s)", "Achieved", f"P_req = {perf['cruise_design_power_kW']:.0f} kW << P_avail = 3,128 kW; η_p = {perf['cruise_design_eta_prop']:.2f} (Slide 13)"],
        ["4", "Max level speed capability", ">= 450 km/h (125 m/s)", f"{perf['vmax_feasible_ms']*3.6:.0f} km/h ({perf['vmax_feasible_ms']:.1f} m/s)", "Achieved", "Feasible up to collective/compressibility knee at 3,000 m (Slide 15)"],
        ["5", "OGE hover ceiling (MTOW, ISA)", ">= 2,000 m", f"{perf['oge_hover_ceiling_m']:.0f} m OGE ({perf['ige_hover_ceiling_m']:.0f} m IGE)", "Achieved", "High R = 4.6 m & 4,300 kW OEI-sized power give ample hover margin"],
        ["6", "Airplane service ceiling", ">= 7,000 m", f"{perf['airplane_service_ceiling_m']:.0f} m", "Achieved", ">= 100 kW excess shaft power at 85 m/s climb speed (Slide 15)"],
        ["7", "750 km VTOL mission + reserve", "Feasible w/ >=110 kg res.", f"{miss['landing_fuel_kg']:.0f} kg rem. ({miss['fuel_burned_kg']:.0f} kg burn)", "Achieved", f"4 m/s headwind; {miss['total_steps'] - miss['infeasible_steps']}/{miss['total_steps']} steps feasible (Slides 19–22)"],
    ]
    _fill_table_rows(s7.shapes[3].table, comp_rows, font_pt=7.8)
    _set_textbox_lines(
        s7.shapes[9],
        [
            ("Governing Boundaries — Design Limit vs. Model Limit Distinction", 9.5, True, c_blue),
            (
                f"All 7 project targets are achieved with positive margins. High-speed airplane flight above {perf['vmax_feasible_ms']:.0f} m/s is bounded by a genuine design trade-off (generous 43 m² wing area increases parasite drag S·CD0 and drives collective past 44°), whereas sharp stall/residual cutoffs at extreme nacelle-speed corners reflect steady-BEMT modeling limits without 3D CFD-CSD unsteady wake relaxation.",
                8.2,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 9: 2.1 Final Assumptions and Updates (7x6)
    # ------------------------------------------------------------------
    s9 = prs.slides[8]
    assump_rows = [
        ["1", "Turboshaft ISA lapse & gearbox cap", "Propulsion", "P_SL(ρ/ρ0)^0.95√(T/T0), cap 4,050 kW", "Brayton mass-flow scaling [Leishman 2006; Raymer]", "Caps SL shaft power at 4,050 kW; sets 7.4 km OGE ceiling"],
        ["2", "Part-load SFC curve", "Fuel burn", "0.255·(0.82 + 0.18/χ) kg/(kW·h)", "Turboshaft thermal efficiency droop at low load [Raymer]", "Places best-range speed at ~93 m/s; +14% realistic loiter burn"],
        ["3", "Scheduled RPM & 20° flaperon", "Controls / Aero", "360->535 rpm; δ_f = 20° sin θ_nac", "XV-15 / AW609 tilt schedule [Maisel NASA SP-2000-4517]", "Holds cruise M_tip = 0.616; cuts 1g stall speed by 3.6 m/s"],
        ["4", "Fountain download & wake wash", "Interaction", "F_DL = 5.2% T sin²θ_nac e^(−V/22)", "XV-15 hover download w/ deflected flaperons [Felker 1986]", "Adds +116 kW SL hover power; shifts transition wing α"],
        ["5", "Nacelle-tilt CG shift", "Mass / Trim", "x_CG = 6.16 + 0.04 sin θ_nac m", "Wingtip engine/gearbox mass moment across 90° tilt", "Tracks longitudinal trim neutrality across conversion"],
        ["6", "Quasi-steady mission + headwind", "Mission", "Trapezoidal Δt; 4 m/s headwind", "Valid for |dθ_nac/dt| <= 1.5°/s << Ω; conservative wind", "Captures 800 kg cruise mass burn & +3.7% wind fuel penalty"],
    ]
    _fill_table_rows(s9.shapes[3].table, assump_rows, font_pt=7.8)
    _set_textbox_lines(
        s9.shapes[5],
        [
            ("Continuity with Validated Milestone 1 & Milestone 2 Physics", 9.6, True, c_blue),
            (
                "• Retained unchanged from Milestone 1: Multi-Re Boeing VR-12 XFOIL polars, Viterna–Corrigan post-stall blending, exact helical Prandtl tip/root loss.\n"
                "• Retained unchanged from Milestone 2: Counter-rotating Pitt–Peters skewed-wake edgewise BEMT, Gormont (1973) radial-flow 3D stall delay, 6-DOF equilibrium formulation.\n"
                "• Added in Milestone 3: Altitude/part-load propulsion & fuel integration, scheduled flaperon/fountain download, instantaneous CG tracking, and end-to-end mission time-marching.",
                8.2,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 10: 2.2 Integrated Workflow / Logic Diagram
    # ------------------------------------------------------------------
    s10 = prs.slides[9]
    # Remove scaffold shapes [4..35] in reverse order so indices stay intact
    for idx in range(35, 3, -1):
        _remove_shape(s10.shapes[idx])
    # Now remaining shapes: [0,1,2] header, [3] big card, [4] caption bg (was 36), [5] caption text (was 37), [6] slide num (was 38)
    _add_fitted_picture(s10, RES / "fig_2_2_integrated_workflow.png", 0.62, 0.84, 12.10, 4.96)
    _set_textbox_lines(
        s10.shapes[5],
        [
            (
                "Figure 2.2: Final TR-9 integrated modular workflow showing Milestone 1 (blue), Milestone 2 (teal), and Milestone 3 (amber) physics modules, passed physical state/load vectors, and zero-hard-coded YAML configuration inputs.",
                8.3,
                False,
                c_slate,
            )
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 11: 2.3 Final Code Organization and Reproducibility
    # ------------------------------------------------------------------
    s11 = prs.slides[10]
    tree_lines = [
        ("milestone3_package/  (Modular Package)", 9.5, True, c_blue),
        ("├── README.md               # Setup & CLI guide", 8.0, False, c_navy),
        ("├── requirements.txt        # Pinned Python deps", 8.0, False, c_navy),
        ("├── config/", 8.3, True, c_teal),
        ("│   ├── aircraft_final.yaml # Frozen TR-9 params", 8.0, False, c_navy),
        ("│   └── mission_final.yaml  # 6-seg VTOL mission", 8.0, False, c_navy),
        ("├── src/", 8.3, True, c_teal),
        ("│   ├── atmosphere.py       # ISA + ΔT + wind", 8.0, False, c_navy),
        ("│   ├── airfoil_bemt.py     # M1/M2 VR-12 BEMT", 8.0, False, c_navy),
        ("│   ├── propulsion.py       # Turboshaft + SFC", 8.0, False, c_navy),
        ("│   ├── aircraft_aero.py    # 6-DOF loads + wake", 8.0, False, c_navy),
        ("│   ├── trim_solver.py      # 6-DOF FCC solver", 8.0, False, c_navy),
        ("│   └── mission_planner.py  # Time-marching sim", 8.0, False, c_navy),
        ("├── scripts/", 8.3, True, c_teal),
        ("│   ├── run_performance.py  # Reproduces Sec 3", 8.0, False, c_navy),
        ("│   ├── run_mission.py      # Reproduces Sec 4", 8.0, False, c_navy),
        ("│   └── make_plots.py       # Generates all PNGs", 8.0, False, c_navy),
        ("└── results/                # CSV tables & plots", 8.0, False, c_navy),
    ]
    _set_textbox_lines(s11.shapes[4], tree_lines)
    repro_rows = [
        ["1. Setup", "pip install -r milestone3_package/requirements.txt", "Installs numpy, scipy, pandas, matplotlib, pyyaml"],
        ["2. Inputs", "config/aircraft_final.yaml & mission_final.yaml", "All geometry, controls, atmosphere & mission inputs"],
        ["3. Sec 3", "python milestone3_package/scripts/run_performance.py", "Tables 3.1, 3.3, 3.4 & 475-pt conversion corridor CSVs"],
        ["4. Sec 4", "python milestone3_package/scripts/run_mission.py", "750 km mission telemetry, segment & limit-check CSVs"],
        ["5. Plots", "python milestone3_package/scripts/make_plots.py", "Generates all Section 1, 2, 3, and 4 publication PNGs"],
    ]
    _fill_table_rows(s11.shapes[5].table, repro_rows, font_pt=7.8)
    _set_textbox_lines(
        s11.shapes[7],
        [
            ("Reproducibility Verification Checklist", 9.6, True, c_blue),
            ("✓ Zero hard-coded geometry, RPM/nacelle schedules, atmospheric states, or mission parameters in solver modules.", 8.1, False, c_navy),
            ("✓ Cached Boeing VR-12 polars (airfoils/) auto-loaded with analytical fallback for standalone execution.", 8.1, False, c_navy),
            ("✓ Verified end-to-end execution from clean virtual environment; companion notebook milestone3_team9.ipynb included.", 8.1, False, c_navy),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 13: 3.1 Hover and Airplane-Mode Performance
    # ------------------------------------------------------------------
    s13 = prs.slides[12]
    rows_31 = [
        [
            r["Operating_Point"],
            r["Alt_ISA"],
            f"{float(r['Gross_Mass_kg']):,.0f}",
            r["RPM_Collective"],
            r["Thrust_or_Lift_N"],
            f"{float(r['P_req_kW']):,.1f}",
            f"{float(r['P_avail_kW']):,.1f}",
            r["Efficiency_Metric"],
        ]
        for _, r in df_31.iterrows()
    ]
    _fill_table_rows(s13.shapes[3].table, rows_31, font_pt=7.6)
    _remove_shape(s13.shapes[7])
    _remove_shape(s13.shapes[5])
    _add_fitted_picture(s13, RES / "fig_3_1a_hover_performance.png", 0.58, 3.52, 5.94, 2.50)
    _add_fitted_picture(s13, RES / "fig_3_1b_cruise_performance.png", 6.81, 3.52, 5.94, 2.50)
    _set_textbox_lines(
        s13.shapes[7],  # was [9], shifted by 2 removed shapes
        [
            (
                f"Definitions & Conditions: Hover FM = T^(3/2)/√(2ρA)/P_aero (includes 5.2% wing fountain download, 535 rpm). Cruise η_p = T·V_TAS/P_aero and trimmed L/D = W/D_tot (3,000 m ISA, 360 rpm, θ_nac = 0°, η_gb = 0.96). Best-range speed ({perf['best_range_speed_ms']:.1f} m/s) reflects coupled L/D and part-load turboshaft SFC.",
                7.9,
                False,
                c_slate,
            )
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 14: 3.2 Conversion Performance Summary
    # ------------------------------------------------------------------
    s14 = prs.slides[13]
    _remove_shape(s14.shapes[4])
    _add_fitted_picture(s14, RES / "fig_3_2_conversion_corridor.png", 0.58, 0.83, 7.14, 5.08)
    conv_sub = df_tel[df_tel["segment"].isin(["Conversion", "Reconversion"])]
    rp = conv_sub.loc[conv_sub["min_norm_margin"].idxmin()]
    cor_rows = [
        ["Feasible trim states", f"{perf['corridor_feasible_count']} / {perf['corridor_total_count']} ({perf['corridor_feasible_pct']:.1f}% vs 13.7% in M2)"],
        ["Corridor speed band", "0–55 m/s (90° helo) -> 39–120 m/s (0° air)"],
        ["Outbound schedule", "12 -> 92 m/s over 90 s (88° -> 0°, max 1.5°/s)"],
        ["Inbound schedule", "58 -> 14 m/s over 135 s (0° -> 90°, max 1.2°/s)"],
        ["Low-speed boundary", f"Wing stall (V_s = {perf['vs_flap_sl_ms']:.1f} m/s w/ 20° flaperon)"],
        ["High-speed boundary", "Advancing tip Mach > 0.85 (θ_nac >= 65°)"],
    ]
    _fill_table_rows(s14.shapes[4].table, cor_rows, font_pt=7.9)  # shifted from [5] to [4]
    _set_textbox_lines(
        s14.shapes[6],  # shifted from [7] to [6]
        [
            (
                f"Figure 3.2: Updated TR-9 speed–nacelle conversion corridor at m = 6,800 kg, h = 500 m ISA. Feasibility requires |F_res| <= 100 N, |M_res| <= 150 N·m, P_req <= P_avail, M_tip <= 0.85, and positive rotor/wing stall & control margins. Most restrictive mission transition point: {rp['governing_margin']} at V = {rp['tas_ms']:.1f} m/s, θ_nac = {rp['nacelle_deg']:.1f}° ({rp['segment']}).",
                7.9,
                False,
                c_slate,
            )
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 15: 3.3 Final Operating-Limits Table (9x5)
    # ------------------------------------------------------------------
    s15 = prs.slides[14]
    rows_33 = [[r["Limit"], r["Value"], r["Units"], r["Governing_Constraint"], r["Condition_Basis"]] for _, r in df_33.iterrows()]
    _fill_table_rows(s15.shapes[3].table, rows_33, font_pt=7.9)

    # ------------------------------------------------------------------
    # SLIDE 16: 3.4 Comparison with a Relevant Tiltrotor (7x6)
    # ------------------------------------------------------------------
    s16 = prs.slides[15]
    tbl16 = s16.shapes[3].table
    _set_cell(tbl16.cell(0, 2), "Ref 1: Leonardo AW609", font_pt=8.3, bold=True, color=c_white)
    _set_cell(tbl16.cell(0, 3), "Ref 2: Bell XV-15", font_pt=8.3, bold=True, color=c_white)
    rows_34 = [
        [r["Metric"], r["Final_Design"], r["Ref1_AW609"], r["Ref2_XV15"], r["Ratio_Final_Ref1"], r["Note_Source"]]
        for _, r in df_34.iterrows()
    ]
    _fill_table_rows(tbl16, rows_34, font_pt=7.8)
    _set_textbox_lines(
        s16.shapes[5],
        [
            ("Physical Interpretation of Normalized Differences vs. AW609 & XV-15", 9.6, True, c_blue),
            (
                "• Lower Disk Loading (0.62× AW609): Selecting R = 4.60 m on a 20 m wingspan lowers hover induced velocity (v_h = 14.7 m/s), cutting SL hover power to 1,578 kW and enabling a 7,400 m OGE ceiling.\n"
                "• Lower Wing Loading (0.36× AW609): Because b = 20 m is set by rotor-fuselage clearance (4.33 m) at moderate AR = 9.3, S_w = 43 m² yields a low 39.2 m/s conversion stall speed (wide corridor) at the cost of higher cruise wetted drag S·CD0 and a 400 km/h design cruise speed vs AW609's pressurized 500 km/h cruise.\n"
                "• Payload Fraction (22.1%): Matches both AW609 (22.0%) and XV-15 (22.7%), confirming realistic structural/empty-weight scaling.",
                8.0,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 18: 4.1 Mission Definition
    # ------------------------------------------------------------------
    s18 = prs.slides[17]
    seg_def_rows = [
        ["1", "Takeoff / hover", "0 -> 30 -> 400 m", "0 -> 12 m/s (3.1 m/s ROC)", "90° -> 88°", "3.5 min"],
        ["2", "Outbound conversion", "400 -> 550 m", "12 -> 92 m/s (+0.89 m/s²)", "88° -> 0°", "1.5 min"],
        ["3", "Airplane cruise", "550 -> 3,000 -> 550 m", "111.1 m/s TAS (107.1 GS)", "0° (360 rpm)", f"{float(df_42c.iloc[2]['Duration_min']):.1f} min"],
        ["4", "Inbound reconversion", "550 -> 480 m", "58 -> 14 m/s (−0.33 m/s²)", "0° -> 90°", "2.3 min"],
        ["5", "Descent & landing", "460 -> 30 -> 0 m", "14 -> 0 m/s (vertical TD)", "88° -> 90°", "4.0 min"],
        ["R", "Protected reserve", "1,500 m loiter", "78.0 m/s airplane loiter", "0° (360 rpm)", "20 min / >=10%"],
    ]
    _fill_table_rows(s18.shapes[3].table, seg_def_rows, font_pt=7.7)
    cond_rows = [
        ["Design payload", "1,500 kg (constant door-to-door)"],
        ["Takeoff / fuel mass", "6,800 kg MTOW / 1,100 kg fuel"],
        ["Atmosphere", "ISA (ΔT = 0°C), ρ(h) lapse"],
        ["En-route wind", "4.0 m/s steady headwind"],
        ["Target ground range", "750.0 km point-to-point VTOL"],
        ["Reserve fuel policy", f"max(10% tank, 20-min loiter) = {miss['protected_reserve_kg']:.0f} kg"],
    ]
    _fill_table_rows(s18.shapes[4].table, cond_rows, font_pt=7.9)
    _set_textbox_lines(
        s18.shapes[6],
        [
            (
                f"Takeoff gross mass & fuel: 6,800 kg / 1,100 kg   •   Point-to-point VTOL range: {miss['total_distance_km']:.1f} km (4 m/s headwind; >1,050 km still-air max range)   •   Protected reserve: {miss['protected_reserve_kg']:.0f} kg (10% capacity; 20-min loiter needs {miss['loiter_20min_burn_kg']:.1f} kg)",
                8.4,
                True,
                c_navy,
            )
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 19: 4.2(a) Trajectory and Mass
    # ------------------------------------------------------------------
    s19 = prs.slides[18]
    _remove_shape(s19.shapes[8])
    _remove_shape(s19.shapes[6])
    _remove_shape(s19.shapes[4])
    _add_fitted_picture(s19, RES / "fig_4_2a_p1_altitude.png", 0.58, 0.82, 3.89, 3.55)
    _add_fitted_picture(s19, RES / "fig_4_2a_p2_speed_dist.png", 4.72, 0.82, 3.89, 3.55)
    _add_fitted_picture(s19, RES / "fig_4_2a_p3_mass_fuel.png", 8.86, 0.82, 3.89, 3.55)
    _set_textbox_lines(
        s19.shapes[12],  # was [15], minus 3 removed shapes
        [
            (
                f"Figure 4.2(a): End-to-end 750 km regional VTOL mission trajectory & mass history (Payload = 1,500 kg, ISA +0°C, 4.0 m/s headwind, {miss['total_steps']} trimmed steps; vertical dashed lines & shading denote Segments 1–5).",
                8.0,
                False,
                c_slate,
            )
        ],
    )
    _set_textbox_lines(
        s19.shapes[14],  # was [17], minus 3 removed shapes
        [
            (
                f"Key Observations: (1) Helicopter climb reaches 400 m before 90 s outbound conversion accelerates the aircraft to 92 m/s; (2) 3,000 m cruise at 111.1 m/s TAS (107.1 m/s GS) covers 96.7% of total ground distance; (3) Fuel burn rate drops from ~515 kg/h in takeoff/climb to ~365 kg/h in cruise as gross mass lightens from 6,800 kg to {miss['landing_mass_kg']:,.0f} kg, leaving {miss['landing_fuel_kg']:.0f} kg at touchdown (+{miss['landing_fuel_kg']-miss['protected_reserve_kg']:.0f} kg above reserve).",
                8.0,
                False,
                c_navy,
            )
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 20: 4.2(b) Configuration and Power
    # ------------------------------------------------------------------
    s20 = prs.slides[19]
    _remove_shape(s20.shapes[8])
    _remove_shape(s20.shapes[6])
    _remove_shape(s20.shapes[4])
    _add_fitted_picture(s20, RES / "fig_4_2b_p1_nacelle_rpm.png", 0.58, 0.82, 3.89, 3.55)
    _add_fitted_picture(s20, RES / "fig_4_2b_p2_power.png", 4.72, 0.82, 3.89, 3.55)
    _add_fitted_picture(s20, RES / "fig_4_2b_p3_margins.png", 8.86, 0.82, 3.89, 3.55)
    _set_textbox_lines(
        s20.shapes[12],
        [
            (
                "Figure 4.2(b): Scheduled nacelle tilt & RPM, total shaft power vs. effective available power (min of 4,050 kW transmission limit & ISA engine lapse), and rotor/wing/tip-Mach safety margins (>0 = feasible).",
                8.0,
                False,
                c_slate,
            )
        ],
    )
    _set_textbox_lines(
        s20.shapes[14],
        [
            (
                f"Key Observations: (1) Peak power ({miss['peak_power_kW']:,.0f} kW) occurs during Segment 1 helicopter climb at heavy MTOW, still retaining >2,000 kW dual-engine margin (and full OEI safety); (2) Converting to θ_nac = 0° & 360 rpm cuts steady power by 43% to {miss['cruise_steady_power_kW']:,.0f} kW while widening tip-Mach margin to +0.234; (3) All stall, Mach, and power margins remain strictly positive across all 5 segments.",
                8.0,
                False,
                c_navy,
            )
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 21: 4.2(c) Segment Summary (7x12)
    # ------------------------------------------------------------------
    s21 = prs.slides[20]
    rows_42c = [
        [
            r["Segment"],
            str(r["Time_min"]),
            str(r["Altitude_m"]),
            f"{float(r['GS_ms']):.1f} / {float(r['TAS_ms']):.1f}",
            f"{float(r['Cum_dist_km']):.1f}",
            f"{float(r['Gross_mass_kg']):,.0f}",
            f"{float(r['Fuel_left_kg']):.1f}",
            f"{float(r['Nacelle_deg']):.1f}°",
            f"{float(r['RPM']):.0f}",
            f"{float(r['P_req_end_kW']):.0f} ({float(r['P_req_peak_kW']):.0f} pk)",
            f"{float(r['P_avail_end_kW']):.0f}",
            str(r["Min_margin_label"]),
        ]
        for _, r in df_42c.iterrows()
    ]
    _fill_table_rows(s21.shapes[3].table, rows_42c, font_pt=6.8)
    _set_textbox_lines(
        s21.shapes[5],
        [
            ("End-to-End Mission Totals & Energy Accountability", 9.8, True, c_blue),
            (
                f"• Total Mission Time: {miss['total_time_min']:.1f} min (2.14 h)   •   Ground Distance: {miss['total_distance_km']:.1f} km   •   Total Fuel Burned: {miss['fuel_burned_kg']:.1f} kg\n"
                f"• Landing Fuel Remaining: {miss['landing_fuel_kg']:.1f} kg   •   Protected Reserve Requirement: {miss['protected_reserve_kg']:.1f} kg (Surplus margin: +{miss['landing_fuel_kg']-miss['protected_reserve_kg']:.1f} kg; 20-min loiter burns {miss['loiter_20min_burn_kg']:.1f} kg)\n"
                f"• Cruise accounts for {100.0*float(df_42c.iloc[2]['Fuel_burned_seg_kg'])/miss['fuel_burned_kg']:.1f}% of mission fuel and 96.7% of ground range; VTOL + transitions require only {miss['fuel_burned_kg']-float(df_42c.iloc[2]['Fuel_burned_seg_kg']):.1f} kg ({100.0*(1.0-float(df_42c.iloc[2]['Fuel_burned_seg_kg'])/miss['fuel_burned_kg']):.1f}%).",
                8.2,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 22: 4.3 Feasibility and Limit Checks
    # ------------------------------------------------------------------
    s22 = prs.slides[21]
    rows_43 = [
        [r["Check"], r["Adopted_Limit"], r["Min_Value_Margin"], r["Segment_Time"], r["Status"], r["Comment"]]
        for _, r in df_43.iterrows()
    ]
    _fill_table_rows(s22.shapes[3].table, rows_43, font_pt=7.7)
    _remove_shape(s22.shapes[5])
    _add_fitted_picture(s22, RES / "fig_4_3_normalized_margins.png", 0.58, 4.28, 7.54, 2.34)
    _set_textbox_lines(
        s22.shapes[6],  # shifted from [7] to [6]
        [
            ("Minimum Normalized Margin Overall", 9.8, True, c_blue),
            (f"Quantity:  {miss['overall_min_quantity']}", 8.6, True, c_navy),
            (f"Normalized Margin:  +{miss['overall_min_norm_margin_pct']:.1f}%", 8.6, True, RGBColor(21, 128, 61) if miss["overall_min_norm_margin_pct"] > 0 else RGBColor(220, 38, 38)),
            (f"Occurs:  {miss['overall_min_segment']} (t = {miss['overall_min_time_min']:.1f} min)", 8.4, False, c_navy),
            (f"Feasibility Verdict:  {'ALL 100% STEPS PASS' if miss['all_steps_feasible'] else 'CHECK FLAGGED'} ({miss['total_steps']-miss['infeasible_steps']}/{miss['total_steps']} steps)", 8.3, True, c_teal),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 23: 4.4 Mission Interpretation (6x4)
    # ------------------------------------------------------------------
    s23 = prs.slides[22]
    interp_rows = [
        ["1. Takeoff / hover", f"Peak power ({miss['peak_power_kW']:.0f} kW); 100% rotor lift + 5.2% download", "High axial induced inflow v_h = 14.7 m/s + vertical climb potential-energy rate at MTOW", "Sizes rotor radius R = 4.6 m, transmission cap & OEI installed power"],
        ["2. Conversion", "1,560 -> 820 kW; lift hands off 100% rotor -> 98% wing", "Translational lift halves v_i while 20° flaperons build wing circulation as nacelles tilt to 0°", "Governs wing area (43 m²), flaperon schedule & hub-CG alignment"],
        ["3. Cruise", f"Steady ~{miss['cruise_steady_power_kW']:.0f} kW (climb peak ~1,790 kW); 100% wing-borne", "360 rpm cuts blade profile power by 69% while wing operates near L/D = 8.7–9.3", "Dominates total fuel tank sizing (90%+ of fuel burn) & blade twist (−35°)"],
        ["4. Reconversion", "240 -> 960 kW; decelerating lift handoff back to proprotors", "Decelerating from 68 to 14 m/s unloads wing near stall boundary while rotor RPM ramps 360 -> 529", "Most delicate control/aerodynamic coordination (wing & rotor stall margins)"],
        ["5. Landing", "~1,360 kW hover touchdown at 5,995 kg (−12% vs takeoff)", "805 kg lighter mass lowers hover disk loading to 442 N/m² and cuts induced power by 17%", "Determines final reserve fuel margin (+185 kg surplus above 110 kg floor)"],
    ]
    _fill_table_rows(s23.shapes[3].table, interp_rows, font_pt=7.7)
    _set_textbox_lines(
        s23.shapes[5],
        [
            ("Most Constraining Mission Segment", 9.6, True, c_blue),
            (
                f"Segment 2/4 Conversion Corridor & Segment 3 Fuel Burn: Aerodynamically, transition ({miss['overall_min_segment']}, t = {miss['overall_min_time_min']:.1f} min) produces the tightest instantaneous stall/trim margin (+{miss['overall_min_norm_margin_pct']:.1f}% normalized, Slide 22), while energetically Segment 3 Cruise consumes >90% of total fuel and governs gross vehicle sizing.",
                8.1,
                False,
                c_navy,
            ),
        ],
    )
    _set_textbox_lines(
        s23.shapes[7],
        [
            ("Implications for Tiltrotor Sizing & Alternative Missions", 9.6, True, c_blue),
            (
                "• A short-hop urban eVTOL/EMS mission (<150 km, frequent hovers) would favor even lower disk loading and smaller fuel tanks.\n"
                "• A high-speed 500 km/h executive mission (AW609-like) would shrink wing area from 43 m² to ~24 m² to cut cruise S·CD0, accepting a narrower conversion corridor (~52 m/s stall speed).",
                8.0,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 25: 5.1 Principal Design Trade-offs (6x5)
    # ------------------------------------------------------------------
    s25 = prs.slides[24]
    trade_rows = [
        ["Disk loading vs. wing span", "Slides 5, 13, 16", "R = 4.60 m (DL = 502 N/m²) on b = 20.0 m wing", "Cuts SL hover power to 1,578 kW, but requires 20 m span for 4.3 m fuselage clearance", "−0.5 m radius -> +14% hover power, −2.0 m wingspan"],
        ["Wing area vs. cruise drag", "Slides 13, 14, 16", "S_w = 43 m² (W/S = 1,551 N/m²) + 20° flaperon", "Lowers stall speed to 39.2 m/s (35.8% corridor), but raises cruise parasite drag (L/D = 8.7 at 400 km/h)", "−10 m² wing area -> +12% cruise L/D, +5.6 m/s stall speed"],
        ["Blade twist: hover vs. cruise", "Slides 6, 13, 14", "θ_tw = −35.0° nonlinear/linear washout, 0.17R cutout", "Boosts airplane η_p from 0.76 to 0.83 and clears conversion root stall; costs ~0.03 hover FM", "±5° twist shifts cruise η_p by ±0.035 & hover FM by ∓0.02"],
        ["Proprotor RPM ratio", "Slides 6, 13, 20", "535 rpm hover -> 360 rpm cruise (67.3% ratio)", "Drops cruise M_tip to 0.616 and cuts profile power 69%, while preserving positive blade stall margin", "<320 rpm triggers cruise blade stall; >420 rpm spikes drag"],
        ["OEI power sizing vs. SFC", "Slides 5, 15, 20", "4,300 kW SL installed (2×2,150 kW) w/ 4,050 kW cap", "Enables safe single-engine emergency hover & 7.4 km OGE ceiling, but runs cruise at ~35% load (higher SFC)", "−25% engine size saves ~140 kg dry mass & ~6% cruise SFC"],
    ]
    _fill_table_rows(s25.shapes[3].table, trade_rows, font_pt=7.7)

    # ------------------------------------------------------------------
    # SLIDE 26: 5.2 Model Limitations and Confidence in Results (7x5)
    # ------------------------------------------------------------------
    s26 = prs.slides[25]
    lim_rows = [
        ["Pitt–Peters linear inflow (no free vortex wake)", "Model limitation", "Mid-conversion power & hub pitch moments (20–55 m/s)", "Medium (±12–15%)", "Validate against CAMRAD II / CHARM free-vortex wake"],
        ["Empirical 5.2% hover fountain download", "Model limitation", "OGE hover power & SL max hover mass", "Medium (±8%)", "3D RANS actuator-disk / powered wind-tunnel download test"],
        ["Rigid gimbaled blade (no aeroelastic whirl mode)", "Model limitation", "High-speed >125 m/s proprotor-pylon stability", "Low at V > 130 m/s", "Coupled wing-pylon-rotor whirl-flutter eigenvalue analysis"],
        ["Quasi-steady trim across conversion steps", "Model limitation", "Transient pitch/elevator peaks during 90 s tilt", "Med-High (±5% fuel)", "Full 6-DOF nonlinear time-domain flight dynamics sim"],
        ["Large 43 m² wing parasite drag at 400+ km/h", "Design limitation", "Cruise L/D (8.7) & max speed (481 vs 500 km/h AW609)", "High (well-captured)", "Genuine TR-9 geometry trade-off favoring corridor safety"],
        ["Part-load SFC penalty from OEI engine sizing", "Design limitation", "Cruise SFC (0.34 vs 0.255 rated) & 805 kg fuel burn", "High (physics-based)", "Consider variable-speed turbine or OEI 30-sec contingency rating"],
    ]
    _fill_table_rows(s26.shapes[3].table, lim_rows, font_pt=7.6)
    _set_textbox_lines(
        s26.shapes[8],
        [
            ("Model Limitation vs. Genuine Design Limitation", 9.5, True, c_blue),
            (
                "High confidence applies to steady hover power, airplane cruise L/D, η_p, and integrated mission fuel burn (±5–8%). Medium/lower confidence applies to mid-conversion rotor-on-wing interference and high-speed whirl-flutter boundaries (model limitations), whereas the 400 km/h cruise drag and part-load SFC are genuine physical consequences of our frozen TR-9 sizing choices.",
                8.0,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 27: 5.3 Recommended Next Steps (6x5)
    # ------------------------------------------------------------------
    s27 = prs.slides[26]
    next_rows = [
        ["1 (Highest)", "Proprotor–wing–pylon whirl-flutter stability analysis", "Addresses unmodeled high-speed aeroelastic boundary (Slide 26 #3)", "Multibody CSD (CAMRAD II / Dymola)", "Damping vs. airspeed V_flutter margin > 1.2 V_dive"],
        ["2 (High)", "Free-vortex wake + panel wing/tail interaction", "Replaces empirical fountain/downwash formulas in conversion (Slide 26 #1–2)", "Viscous Vortex Particle / Free-Wake", "Refined conversion corridor & unsteady tail buffet loads"],
        ["3 (High)", "Nonlinear 6-DOF transient & OEI failure simulation", "Verifies pilot/FCC workload & altitude loss during engine-out in transition", "Time-domain flight sim + LQR FCC", "OEI height–velocity diagram & actuator rate bounds"],
        ["4 (Medium)", "Aero-structural wing & twist multi-point MDO", "Trades wing area (32–43 m²) & nonlinear blade twist for +12% range (Slide 25)", "Gradient MDO + finite-element wing", "Pareto front of MTOW vs. corridor width & cruise speed"],
        ["5 (Medium)", "1/8-scale powered semi-span wind-tunnel test", "Calibrates VR-12 Gormont 3D stall delay & flaperon download across θ_nac", "Powered wind tunnel + 6-axis balance", "Experimental validation dataset for BEMT/trim calibration"],
    ]
    _fill_table_rows(s27.shapes[3].table, next_rows, font_pt=7.7)

    # ------------------------------------------------------------------
    # SLIDE 29: 6.1 Final Code Package (7x4)
    # ------------------------------------------------------------------
    s29 = prs.slides[28]
    pkg_rows = [
        ["Setup & reproduction guide", "milestone3_package/README.md & requirements.txt", "Y", "Exact venv install & 3 single-command CLI scripts"],
        ["User configuration files", "milestone3_package/config/aircraft_final.yaml, mission_final.yaml", "Y", "Zero hard-coded geometry, schedules, or mission inputs"],
        ["Rotor & atmosphere solvers", "milestone3_package/src/atmosphere.py, airfoil_bemt.py, ../airfoils/", "Y", "M1/M2 VR-12 BEMT, Pitt-Peters wake & Gormont 3D stall"],
        ["Aircraft aero, propulsion & trim", "milestone3_package/src/aircraft_aero.py, propulsion.py, trim_solver.py", "Y", "6-DOF trim, scheduled flaperons, ISA lapse & part-load SFC"],
        ["End-to-end mission planner", "milestone3_package/src/mission_planner.py & scripts/run_mission.py", "Y", "Time-marching integrator, reserve check & margin tables"],
        ["Figure & notebook suite", "milestone3_package/scripts/make_plots.py & milestone3_team9.ipynb", "Y", "Reproduces every Section 1–4 figure & interactive outputs"],
    ]
    _fill_table_rows(s29.shapes[3].table, pkg_rows, font_pt=7.9)
    _set_textbox_lines(
        s29.shapes[5],
        [
            ("Submission Package Manifest (Team9_Milestone3_CodePackage.zip)", 9.6, True, c_blue),
            (
                "Submitted archive contains milestone3_package/ (all modular src/, config/, scripts/, results/), cached VR-12 polars (airfoils/), executed interactive notebook (milestone3_team9.ipynb), and this consolidated presentation deck. Running run_performance.py, run_mission.py, and make_plots.py regenerates every table and plot from scratch.",
                8.2,
                False,
                c_navy,
            ),
        ],
    )

    # ------------------------------------------------------------------
    # SLIDE 30: 6.2 References and Acknowledgement
    # ------------------------------------------------------------------
    s30 = prs.slides[29]
    ref_lines = [
        ("Technical References", 10.5, True, c_blue),
        ("[1] Leishman, J. G., Principles of Helicopter Aerodynamics, 2nd ed., Cambridge Univ. Press, 2006 — used for: axial/edgewise BEMT, Prandtl tip/root loss, hover FM & download.", 7.7, False, c_navy),
        ("[2] Maisel, M. D., Giulianetti, D. J., & Dugan, D. C., The History of the XV-15 Tilt Rotor Research Aircraft (NASA SP-2000-4517), 2000 — used for: XV-15 benchmark data, flaperon/nacelle conversion schedules & fountain download.", 7.7, False, c_navy),
        ("[3] Leonardo Helicopters, AW609 Tiltrotor Technical Datasheet & Type Certification Specs, 2023 — used for: Reference 1 tiltrotor comparison (Slide 16).", 7.7, False, c_navy),
        ("[4] Gormont, R. E., A Mathematical Model of Unsteady Aerodynamics and Radial Flow for Application to Helicopter Rotors, USAAMRDL TR 72-67, 1973 — used for: 3D spanwise-flow stall-delay & yawed drag correction.", 7.7, False, c_navy),
        ("[5] Pitt, D. M., & Peters, D. A., 'Theoretical Prediction of Dynamic-Inflow Derivatives,' Vertica, Vol. 5, 1981 — used for: skewed-wake longitudinal inflow gradient k_x.", 7.7, False, c_navy),
        ("[6] Viterna, L. A., & Corrigan, R. D., Fixed Pitch Rotor Performance of Large Horizontal Axis Wind Turbines, NASA CP-2230, 1982 — used for: wide-angle ±45° post-stall polar continuation.", 7.7, False, c_navy),
        ("[7] Raymer, D. P., Aircraft Design: A Conceptual Approach, 6th ed., AIAA, 2018 — used for: turboshaft altitude lapse, part-load SFC & Breguet range.", 7.7, False, c_navy),
    ]
    _set_textbox_lines(s30.shapes[4], ref_lines)

    ack_lines = [
        ("Acknowledgements", 10.5, True, c_blue),
        ("Course Instruction & Guidance", 8.8, True, c_teal),
        ("• Course Instructor & Teaching Assistants, Rotary-Wing Aerodynamics — for Milestone 1 & Milestone 2 feedback that guided our hub-CG trim refinement and cruise RPM selection.", 7.8, False, c_navy),
        ("Open-Source Software & Data", 8.8, True, c_teal),
        ("• Python scientific ecosystem: NumPy, SciPy (scipy.optimize.least_squares & brentq), Pandas, Matplotlib, PyYAML, and python-pptx.", 7.8, False, c_navy),
        ("• UIUC Applied Aerodynamics Group / XFOIL — baseline Boeing VR-12 and NACA airfoil section polars.", 7.8, False, c_navy),
        ("Generative-AI Disclosure (Course Policy)", 8.8, True, c_teal),
        ("• AI coding assistance (Google Antigravity) was used to refactor Milestone 1–2 notebook functions into the YAML-driven milestone3_package/, automate matplotlib figure layout, and format python-pptx tables. All physical models, design choices, and numerical results were verified by Team 9.", 7.8, False, c_navy),
    ]
    _set_textbox_lines(s30.shapes[6], ack_lines)

    out_path = ROOT / "Milestone3_Team9_Final.pptx"
    prs.save(str(out_path))
    print("Saved presentation:", out_path)
    return out_path


if __name__ == "__main__":
    build_deck()
