# TR-9 Final Tiltrotor Design & End-to-End Mission Package (Milestone 3 — Team 9)

Modular, user-input-driven Python framework for **Milestone 3: Final Tiltrotor Design and Mission Demonstration** (Rotary-Wing Aerodynamics Course Project).

All aircraft geometry, mass breakdown, rotor/wing/empennage aerodynamics, control bounds, engine lapse/SFC parameters, and mission segment schedules are read from YAML configuration files (`config/aircraft_final.yaml` and `config/mission_final.yaml`) with **zero hard-coded operating conditions or design constants** inside the physics solvers.

---

## 1. Package Structure

```text
milestone3_package/
├── config/
│   ├── aircraft_final.yaml         # Frozen TR-9 geometry, mass, VR-12 rotor, wing/tail, engine & limits
│   └── mission_final.yaml          # 6-segment 750 km VTOL mission profile, winds, ISA & reserve policy
├── src/
│   ├── __init__.py                 # YAML / JSON configuration loader
│   ├── atmosphere.py               # Troposphere/stratosphere ISA model + ΔT offset & wind kinematics
│   ├── airfoil_bemt.py             # Milestone 1 VR-12 BEMT (helical Prandtl loss, Pitt-Peters, Gormont 3D stall)
│   ├── propulsion.py               # Twin-turboshaft ISA density/temp lapse, 4050 kW gearbox cap & part-load SFC
│   ├── aircraft_aero.py            # Milestone 2+3 6-DOF loads: counter-rotating rotors, wing flaperons, fountain, tail
│   ├── trim_solver.py              # 6-DOF equilibrium solver with smooth FCC cyclic/elevator washout & limit checks
│   └── mission_planner.py          # Quasi-steady time-marching mission integrator, fuel burn & normalized margins
├── scripts/
│   ├── run_performance.py          # Generates all Section 3 tables, hover/cruise sweeps & 475-state corridor map
│   ├── run_mission.py              # Generates all Section 4 telemetry, segment summaries & feasibility limit checks
│   ├── make_plots.py               # Generates all publication-grade PNG figures for Sections 1, 2, 3, and 4
│   └── build_presentation.py       # Populates the 30-slide Milestone 3 presentation from CSV/JSON outputs
├── results/                        # Auto-generated CSV tables, JSON summaries, and PNG figures
├── requirements.txt                # Exact Python dependencies (Python >= 3.10)
└── README.md                       # Setup & single-command reproduction instructions
```

*(Note: Cached Boeing VR-12 XFOIL/UIUC polars reside in `../airfoils/` at the repository root and are automatically resolved by `src/airfoil_bemt.py`, with analytical Viterna–Corrigan fallback if run standalone.)*

---

## 2. Environment Setup

From the repository root (`/Users/apple/Documents/courses/rotary/milestone 2` or extracted archive root):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r milestone3_package/requirements.txt
```

---

## 3. Reproducing All Milestone 3 Deliverables

Run the following commands from the repository root:

1. **Reproduce Section 3 Final Performance Tables & Conversion Corridor (`results/sec3_*.csv`, `performance_summary.json`):**
   ```bash
   python milestone3_package/scripts/run_performance.py
   ```

2. **Reproduce Section 4 End-to-End 750 km VTOL Mission Simulation (`results/sec4_*.csv`, `mission_summary_metrics.json`):**
   ```bash
   python milestone3_package/scripts/run_mission.py
   ```

3. **Generate All Section 1–4 Figures (`results/figures/*.png`):**
   ```bash
   python milestone3_package/scripts/make_plots.py
   ```

4. **Build the Final 30-Slide Presentation (`Milestone3_Team9_Final.pptx`):**
   ```bash
   python milestone3_package/scripts/build_presentation.py
   ```

To test custom aircraft geometries or mission profiles, edit `milestone3_package/config/aircraft_final.yaml` or `milestone3_package/config/mission_final.yaml` and re-run steps 1–3.
