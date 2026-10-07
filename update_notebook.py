import json

new_code = """import numpy as np

TRIM_FEASIBILITY_MODEL_VERSION = "aircraft-equilibrium-v2"


# ---------------------------------------------------------------------
# REQUIRED NOTEBOOK OBJECTS
# ---------------------------------------------------------------------
_required = [
    "TRIM_CFG",
    "TRIM_LO",
    "TRIM_HI",
    "THETA1C_BOUNDS",
    "THETA1S_BOUNDS",
    "_default_trim_seed",
    "_trim_scales",
    "aircraft_loads_total",
    "least_squares",
]
_missing = [name for name in _required if name not in globals()]
if _missing:
    raise RuntimeError(
        "Run the notebook through the Section 6/7 core trim-model cell first. "
        "Missing: " + ", ".join(_missing)
    )


# ---------------------------------------------------------------------
# NEW CLASSIFICATION SETTINGS
# ---------------------------------------------------------------------
# These are WARNING thresholds, not automatic trim-failure thresholds.
TRIM_CFG["rotor_stall_warning_fraction"] = float(
    TRIM_CFG.get("rotor_stall_warning_fraction", 0.05)
)
TRIM_CFG["rotor_stall_marginal_fraction"] = float(
    TRIM_CFG.get("rotor_stall_marginal_fraction", 0.25)
)

# A tiny reverse-flow patch can appear because of grid/discretization.
TRIM_CFG["reverse_flow_warning_fraction"] = float(
    TRIM_CFG.get("reverse_flow_warning_fraction", 0.005)
)
TRIM_CFG["reverse_flow_marginal_fraction"] = float(
    TRIM_CFG.get("reverse_flow_marginal_fraction", 0.15)
)

# How close to a bounded control should be called "near the limit".
TRIM_CFG["control_near_limit_fraction"] = float(
    TRIM_CFG.get("control_near_limit_fraction", 0.01)
)

# Keep the old key for compatibility with older plotting/metadata cells, but
# it is no longer used as a HARD rotor-stall feasibility limit.
TRIM_CFG["rotor_stall_fraction_limit"] = TRIM_CFG[
    "rotor_stall_warning_fraction"
]


# ---------------------------------------------------------------------
# AIRCRAFT PERFORMANCE / SUPPORT DIAGNOSTICS
# ---------------------------------------------------------------------
def aircraft_support_metrics(data, controls):
    \"\"\"
    Resolve non-gravity forces onto inertial UP.

    Body axes:
        +x forward, +y right, +z down
    Positive pitch = nose-up.

    The inertial-up unit vector expressed in body axes is:
        [sin(theta), 0, -cos(theta)]

    For steady non-accelerating flight, total non-gravity upward support
    should be approximately one aircraft weight.  The six-component trim
    residual remains the authoritative equilibrium test.
    \"\"\"
    controls = np.asarray(controls, float)
    pitch_deg = float(controls[3])
    theta = np.radians(pitch_deg)

    up_hat_b = np.array([
        np.sin(theta),
        0.0,
        -np.cos(theta),
    ])

    W = float(TRIM_CFG["mass_kg"] * TRIM_CFG["g_ms2"])
    components = data.get("components", {}) or {}

    def component_up_fraction(name):
        try:
            F = np.asarray(components[name]["F_N"], float)
            return float(np.dot(F, up_hat_b) / W)
        except Exception:
            return np.nan

    # Everything except gravity contributes to aerodynamic/propulsive support.
    total_non_gravity = np.zeros(3)
    for name, item in components.items():
        if name == "gravity":
            continue
        try:
            total_non_gravity += np.asarray(item["F_N"], float)
        except Exception:
            pass

    total_up = float(np.dot(total_non_gravity, up_hat_b) / W)

    return {
        "up_support_fraction_W": total_up,
        "up_support_error_fraction_W": total_up - 1.0,
        "rotor_up_support_fraction_W": component_up_fraction("rotors"),
        "wing_up_support_fraction_W": component_up_fraction("wing"),
        "tail_up_support_fraction_W": component_up_fraction("horizontal_tail"),
        "vertical_tail_up_fraction_W": component_up_fraction("vertical_tail"),
        "parasite_up_fraction_W": component_up_fraction("parasite_drag"),
    }


# ---------------------------------------------------------------------
# COMMON HARD-FAILURE + WARNING CLASSIFIER
# ---------------------------------------------------------------------
def classify_trim_state(data, controls, solver_success=True):
    \"\"\"
    Classify one already-evaluated aircraft state.

    IMPORTANT:
    Rotor stall fraction and reverse flow are WARNINGS.
    They are not, on their own, hard failures.
    \"\"\"
    controls = np.asarray(controls, float)
    residual = np.asarray(data["residual"], float)

    W, Mscale = _trim_scales()
    force_fraction = np.abs(residual[:3]) / W
    moment_fraction = np.abs(residual[3:]) / Mscale

    hard_failures = []
    warnings = []

    # ---- actual aircraft equilibrium / numerical validity ----
    if (not bool(solver_success)) or (not np.all(np.isfinite(residual))):
        hard_failures.append("numerical_failure")

    if np.max(force_fraction) > TRIM_CFG["force_residual_limit_W"]:
        hard_failures.append("excessive_force_residual")

    if np.max(moment_fraction) > TRIM_CFG["moment_residual_limit"]:
        hard_failures.append("excessive_moment_residual")

    # ---- model-domain / real hard limits ----
    if float(data["coverage"]) < TRIM_CFG["database_coverage_min"]:
        hard_failures.append("database_coverage")

    if float(data["polar_extrap_fraction"]) > TRIM_CFG["polar_extrap_fraction_limit"]:
        hard_failures.append("polar_extrapolation")

    if float(data["advancing_tip_Mach"]) > TRIM_CFG["tip_mach_limit"]:
        hard_failures.append("tip_mach")

    if bool(data["wing_stall"]):
        hard_failures.append("wing_stall")

    if bool(data["tail_stall"]):
        hard_failures.append("tail_stall")

    if float(data["power_kW"]) > TRIM_CFG["power_available_kW"]:
        hard_failures.append("power_limit")

    # ---- rotor stall / reverse flow: DIAGNOSTICS, not automatic failure ----
    stall_fraction = float(data["rotor_stall_fraction"])
    reverse_fraction = float(data["reverse_flow_fraction"])

    if stall_fraction > TRIM_CFG["rotor_stall_marginal_fraction"]:
        warnings.append("high_rotor_stall_fraction")
    elif stall_fraction > TRIM_CFG["rotor_stall_warning_fraction"]:
        warnings.append("rotor_stall_warning")

    if reverse_fraction > TRIM_CFG["reverse_flow_marginal_fraction"]:
        warnings.append("high_reverse_flow_fraction")
    elif reverse_fraction > TRIM_CFG["reverse_flow_warning_fraction"]:
        warnings.append("reverse_flow_warning")

    # ---- controls: near-limit is a warning, not automatic failure ----
    span = np.maximum(TRIM_HI - TRIM_LO, 1.0)
    control_margin_fraction = np.minimum(
        controls - TRIM_LO,
        TRIM_HI - controls,
    ) / span

    minimum_control_margin = float(np.min(control_margin_fraction))
    if minimum_control_margin < TRIM_CFG["control_near_limit_fraction"]:
        warnings.append("control_near_limit")

    support = aircraft_support_metrics(data, controls)

    # The force residual is the authoritative check. This support metric is
    # mainly for physical interpretation and diagnostics.
    if abs(support["up_support_error_fraction_W"]) > 0.05:
        warnings.append("large_vertical_support_mismatch")

    hard_failures = list(dict.fromkeys(hard_failures))
    warnings = list(dict.fromkeys(warnings))

    if hard_failures:
        operating_class = "FAILED"
    elif (
        stall_fraction > TRIM_CFG["rotor_stall_marginal_fraction"]
        or reverse_fraction > TRIM_CFG["reverse_flow_marginal_fraction"]
        or minimum_control_margin < 0.002
    ):
        operating_class = "MARGINAL"
    elif warnings:
        operating_class = "CAUTION"
    else:
        operating_class = "NORMAL"

    return {
        "feasible": len(hard_failures) == 0,
        "hard_failures": hard_failures,
        "warnings": warnings,
        "operating_class": operating_class,
        "force_residual_fraction_W": force_fraction,
        "moment_residual_fraction": moment_fraction,
        "minimum_control_margin_fraction": minimum_control_margin,
        **support,
    }


# ---------------------------------------------------------------------
# REPLACEMENT TRIM SOLVER
# ---------------------------------------------------------------------
def solve_trim_case(
    V_ms, nacelle_angle_deg, x0=None,
    flight_path_deg=0.0, altitude_m=0.0, sideslip_deg=0.0,
    multistart=True, max_nfev=110,
):
    \"\"\"
    Bounded six-component aircraft trim.

    The objective prioritizes actual aircraft force/moment closure.
    Rotor stall/reverse-flow fractions are NOT optimizer penalties.
    \"\"\"
    if x0 is None:
        x0 = _default_trim_seed(nacelle_angle_deg, V_ms)

    x0 = np.clip(np.asarray(x0, float), TRIM_LO, TRIM_HI)
    W, Mscale = _trim_scales()

    def objective(u):
        try:
            data = aircraft_loads_total(
                V_ms,
                nacelle_angle_deg,
                u,
                flight_path_deg,
                altitude_m,
                sideslip_deg,
            )

            residual = np.asarray(data["residual"], float)
            if not np.all(np.isfinite(residual)):
                return np.full(14, 1e3)

            # Weak regularization only resolves redundant-control ambiguity.
            controls_regularization = np.array([
                0.002 * u[1] / max(abs(THETA1C_BOUNDS[1]), 1.0),
                0.002 * u[2] / max(abs(THETA1S_BOUNDS[1]), 1.0),
                0.001 * u[4] / 25.0,
                0.001 * u[5] / 20.0,
                0.001 * u[6] / 25.0,
            ])

            # Only hard-domain/installed-power penalties belong here.
            # NOTE: rotor stall and reverse flow are intentionally absent.
            coverage_penalty = 20.0 * max(
                0.0,
                TRIM_CFG["database_coverage_min"] - float(data["coverage"]),
            )
            power_penalty = 2.0 * max(
                0.0,
                float(data["power_kW"])
                / max(TRIM_CFG["power_available_kW"], 1.0)
                - 1.0,
            )
            tip_mach_penalty = 4.0 * max(
                0.0,
                float(data["advancing_tip_Mach"])
                / max(TRIM_CFG["tip_mach_limit"], 1e-6)
                - 1.0,
            )

            validity_penalties = np.array([
                coverage_penalty,
                power_penalty,
                tip_mach_penalty,
            ])

            return np.r_[
                residual[:3] / W,
                residual[3:] / Mscale,
                controls_regularization,
                validity_penalties,
            ]

        except Exception:
            return np.full(14, 1e3)

    seeds = [x0]

    if multistart:
        seeds.extend([
            _default_trim_seed(nacelle_angle_deg, V_ms),
            np.array([8.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]),
            np.array([28.0, 0.0, 0.0, 7.0, -5.0, 0.0, 0.0]),
        ])

    best = None

    for seed in seeds:
        solution = least_squares(
            objective,
            np.clip(np.asarray(seed, float), TRIM_LO, TRIM_HI),
            bounds=(TRIM_LO, TRIM_HI),
            max_nfev=int(max_nfev),
            xtol=2e-6,
            ftol=2e-6,
            gtol=2e-6,
            x_scale="jac",
        )

        # Rank candidate seeds primarily by actual six-component equilibrium.
        physical_score = float(np.linalg.norm(objective(solution.x)[:6]))

        if best is None or physical_score < best["score"]:
            best = {
                "solution": solution,
                "score": physical_score,
            }

    u = np.asarray(best["solution"].x, float)

    data = aircraft_loads_total(
        V_ms,
        nacelle_angle_deg,
        u,
        flight_path_deg,
        altitude_m,
        sideslip_deg,
    )

    classification = classify_trim_state(
        data,
        u,
        solver_success=best["solution"].success,
    )

    residual = np.asarray(data["residual"], float)
    feasible = bool(classification["feasible"])
    hard_failures = classification["hard_failures"]
    warnings = classification["warnings"]

    return {
        "V_ms": float(V_ms),
        "nacelle_deg": float(nacelle_angle_deg),
        "flight_path_deg": float(flight_path_deg),
        "altitude_m": float(altitude_m),

        "success": bool(best["solution"].success),
        "feasible": feasible,

        # Existing downstream code expects failure_codes/failure.
        "failure_codes": hard_failures,
        "failure": "OK" if feasible else "; ".join(hard_failures),

        # New diagnostic layer.
        "warning_codes": warnings,
        "warning": "NONE" if not warnings else "; ".join(warnings),
        "operating_class": classification["operating_class"],
        "feasibility_model_version": TRIM_FEASIBILITY_MODEL_VERSION,

        "u": u,
        "residual": residual,
        "force_residual_fraction_W": classification[
            "force_residual_fraction_W"
        ],
        "moment_residual_fraction": classification[
            "moment_residual_fraction"
        ],

        "rpm": data["rpm"],
        "power_kW": data["power_kW"],
        "power_margin_kW": (
            TRIM_CFG["power_available_kW"] - data["power_kW"]
        ),

        "alpha_body_deg": data["alpha_body_deg"],
        "alpha_wing_deg": data["alpha_wing_deg"],
        "CL_wing": data["CL_wing"],
        "flow_angle_deg": data["flow_angle_deg"],

        "coverage": data["coverage"],
        "rotor_stall_fraction": data["rotor_stall_fraction"],
        "reverse_flow_fraction": data["reverse_flow_fraction"],
        "polar_extrap_fraction": data["polar_extrap_fraction"],
        "advancing_tip_Mach": data["advancing_tip_Mach"],

        # Existing body-z support diagnostics.
        "rotor_support_fraction": data["rotor_support_fraction"],
        "wing_support_fraction": data["wing_support_fraction"],
        "tail_support_fraction": data["tail_support_fraction"],

        # New physically intuitive inertial-up support diagnostics.
        "up_support_fraction_W": classification["up_support_fraction_W"],
        "up_support_error_fraction_W": classification[
            "up_support_error_fraction_W"
        ],
        "rotor_up_support_fraction_W": classification[
            "rotor_up_support_fraction_W"
        ],
        "wing_up_support_fraction_W": classification[
            "wing_up_support_fraction_W"
        ],
        "tail_up_support_fraction_W": classification[
            "tail_up_support_fraction_W"
        ],
        "parasite_up_fraction_W": classification[
            "parasite_up_fraction_W"
        ],

        "minimum_control_margin_fraction": classification[
            "minimum_control_margin_fraction"
        ],

        "components": data["components"],
        "nfev": int(best["solution"].nfev),
        "optimizer_message": str(best["solution"].message),
    }


# ---------------------------------------------------------------------
# REPLACEMENT FLATTENING / CSV ROW FUNCTION
# ---------------------------------------------------------------------
def trim_result_record(result):
    \"\"\"Flatten one solution into an auditable database/report row.\"\"\"
    u = np.asarray(result["u"], float)
    r = np.asarray(result["residual"], float)

    return {
        "V_ms": result["V_ms"],
        "nacelle_deg": result["nacelle_deg"],

        # Keep FEASIBLE/FAILED for compatibility with old Section 7/15 code.
        "status": "FEASIBLE" if result["feasible"] else "FAILED",

        # New layer: NORMAL/CAUTION/MARGINAL/FAILED.
        "operating_class": result.get(
            "operating_class",
            "NORMAL" if result["feasible"] else "FAILED",
        ),
        "warning_or_margin": result.get("warning", "NONE"),
        "failure_or_active_constraint": result["failure"],
        "feasibility_model_version": result.get(
            "feasibility_model_version",
            TRIM_FEASIBILITY_MODEL_VERSION,
        ),

        "pitch_deg": u[3],
        "theta0_deg": u[0],
        "theta1c_lateral_deg": u[1],
        "theta1s_longitudinal_deg": u[2],
        "elevator_deg": u[4],
        "aileron_deg": u[5],
        "rudder_deg": u[6],

        "rpm": result["rpm"],
        "power_kW": result["power_kW"],
        "power_margin_kW": result["power_margin_kW"],

        "Fx_N": r[0],
        "Fy_N": r[1],
        "Fz_N": r[2],
        "Mx_Nm": r[3],
        "My_Nm": r[4],
        "Mz_Nm": r[5],

        "max_force_residual_pct_W": (
            100.0 * np.max(result["force_residual_fraction_W"])
        ),
        "max_moment_residual_pct_ref": (
            100.0 * np.max(result["moment_residual_fraction"])
        ),

        # Original body-z support fractions.
        "rotor_support_pct_W": 100.0 * result["rotor_support_fraction"],
        "wing_support_pct_W": 100.0 * result["wing_support_fraction"],
        "tail_support_pct_W": 100.0 * result["tail_support_fraction"],

        # Better "is the aircraft actually holding itself up?" quantities.
        "total_up_support_pct_W": (
            100.0 * result.get("up_support_fraction_W", np.nan)
        ),
        "up_support_error_pct_W": (
            100.0 * result.get("up_support_error_fraction_W", np.nan)
        ),
        "rotor_up_support_pct_W": (
            100.0 * result.get("rotor_up_support_fraction_W", np.nan)
        ),
        "wing_up_support_pct_W": (
            100.0 * result.get("wing_up_support_fraction_W", np.nan)
        ),
        "tail_up_support_pct_W": (
            100.0 * result.get("tail_up_support_fraction_W", np.nan)
        ),
        "parasite_up_pct_W": (
            100.0 * result.get("parasite_up_fraction_W", np.nan)
        ),

        "minimum_control_margin_pct_range": (
            100.0 * result.get("minimum_control_margin_fraction", np.nan)
        ),

        "alpha_wing_deg": result["alpha_wing_deg"],
        "rotor_stall_fraction": result["rotor_stall_fraction"],
        "rotor_stall_pct": 100.0 * result["rotor_stall_fraction"],
        "reverse_flow_fraction": result["reverse_flow_fraction"],
        "reverse_flow_pct": 100.0 * result["reverse_flow_fraction"],
        "advancing_tip_Mach": result["advancing_tip_Mach"],
        "database_coverage": result["coverage"],
        "flow_angle_deg": result["flow_angle_deg"],
        "solver_evaluations": result["nfev"],
    }


# ---------------------------------------------------------------------
# OPTIONAL PATCH OF CELL-15 TRIM-DATABASE LOOKUP
# ---------------------------------------------------------------------
# If the old lookup already exists, preserve its interpolation machinery but
# replace the final physics-verification rule.
if "trim_database_lookup" in globals():
    _old_trim_database_lookup_v1 = globals()["trim_database_lookup"]

    # Avoid wrapping our own wrapper if this file is run repeatedly.
    if not getattr(
        _old_trim_database_lookup_v1,
        "_aircraft_equilibrium_v2",
        False,
    ):

        def trim_database_lookup(
            V_ms,
            nacelle_deg,
            verify_physics=True,
        ):
            \"\"\"
            V2 trim-database lookup.

            Uses the notebook's existing safe interpolation/support-node logic,
            then verifies the interpolated controls with the SAME aircraft-
            equilibrium classifier used by solve_trim_case().
            \"\"\"
            # Let the old function do domain/support/interpolation only.
            result = _old_trim_database_lookup_v1(
                V_ms,
                nacelle_deg,
                verify_physics=False,
            )

            if not result.get("database_feasible", False):
                return result

            if not verify_physics:
                result["physics_verified"] = False
                result["surrogate_usable"] = True
                result["reason"] = "database_support_only"
                result["feasibility_model_version"] = (
                    TRIM_FEASIBILITY_MODEL_VERSION
                )
                return result

            controls = np.asarray(
                result.get("controls", np.full(7, np.nan)),
                float,
            )

            if not np.all(np.isfinite(controls)):
                result.update({
                    "physics_verified": False,
                    "surrogate_usable": False,
                    "reason": "invalid_interpolated_controls",
                    "feasibility_model_version": (
                        TRIM_FEASIBILITY_MODEL_VERSION
                    ),
                })
                return result

            try:
                check = aircraft_loads_total(
                    float(V_ms),
                    float(nacelle_deg),
                    controls,
                    flight_path_deg=float(
                        globals().get("TRIM_FLIGHT_PATH_DEG", 0.0)
                    ),
                    altitude_m=float(
                        globals().get("TRIM_ALTITUDE_M", 0.0)
                    ),
                )

                classification = classify_trim_state(
                    check,
                    controls,
                    solver_success=True,
                )

                hard_failures = classification["hard_failures"]
                warnings = classification["warnings"]
                verified = len(hard_failures) == 0

                result.update({
                    "physics_verified": verified,
                    "surrogate_usable": verified,
                    "reason": (
                        "OK"
                        if verified
                        else "; ".join(hard_failures)
                    ),
                    "warning": (
                        "NONE"
                        if not warnings
                        else "; ".join(warnings)
                    ),
                    "warning_codes": warnings,
                    "operating_class": classification[
                        "operating_class"
                    ],
                    "feasibility_model_version": (
                        TRIM_FEASIBILITY_MODEL_VERSION
                    ),

                    "verified_residual": check["residual"],
                    "verified_force_residual_fraction_W":
                        classification["force_residual_fraction_W"],
                    "verified_moment_residual_fraction":
                        classification["moment_residual_fraction"],
                    "verified_power_kW": check["power_kW"],
                    "verified_rotor_stall_fraction":
                        check["rotor_stall_fraction"],
                    "verified_reverse_flow_fraction":
                        check["reverse_flow_fraction"],
                    "verified_advancing_tip_Mach":
                        check["advancing_tip_Mach"],
                    "verified_database_coverage":
                        check["coverage"],

                    "verified_total_up_support_fraction_W":
                        classification["up_support_fraction_W"],
                    "verified_rotor_up_support_fraction_W":
                        classification["rotor_up_support_fraction_W"],
                    "verified_wing_up_support_fraction_W":
                        classification["wing_up_support_fraction_W"],
                    "verified_tail_up_support_fraction_W":
                        classification["tail_up_support_fraction_W"],
                    "verified_minimum_control_margin_fraction":
                        classification[
                            "minimum_control_margin_fraction"
                        ],
                })

            except Exception as exc:
                result.update({
                    "physics_verified": False,
                    "surrogate_usable": False,
                    "reason": "verification_failure",
                    "detail": str(exc),
                    "feasibility_model_version": (
                        TRIM_FEASIBILITY_MODEL_VERSION
                    ),
                })

            return result

        trim_database_lookup._aircraft_equilibrium_v2 = True

        print(
            "Patched trim_database_lookup(): "
            "rotor stall/reverse flow are warnings, not automatic failures."
        )


print("=" * 72)
print("AIRCRAFT-EQUILIBRIUM TRIM FEASIBILITY V2 INSTALLED")
print("=" * 72)
print("Hard feasibility now uses:")
print("  - six-component aircraft force/moment closure")
print("  - rotor-database coverage / model validity")
print("  - advancing-tip Mach")
print("  - wing/tail stall")
print("  - installed power")
print("  - numerical convergence")
print()
print("Warnings only:")
print(
    f"  - rotor stall > "
    f"{100*TRIM_CFG['rotor_stall_warning_fraction']:.1f}%"
)
print(
    f"  - high rotor stall > "
    f"{100*TRIM_CFG['rotor_stall_marginal_fraction']:.1f}%"
)
print(
    f"  - reverse flow > "
    f"{100*TRIM_CFG['reverse_flow_warning_fraction']:.1f}%"
)
print("  - controls near their allowed bounds")
print()
print("New CSV diagnostics include total/rotor/wing/tail upward support as %W.")
print("Rebuild any trim CSV generated with the old feasibility logic.")
"""

with open("milestone2_final.ipynb", "r", encoding="utf-8") as f:
    nb = json.load(f)

# Find the cell containing 'def solve_trim_case'
target_idx = -1
for i, cell in enumerate(nb["cells"]):
    if cell["cell_type"] == "code":
        source = "".join(cell["source"])
        if "def solve_trim_case(" in source:
            target_idx = i
            break

if target_idx != -1:
    lines = new_code.split("\n")
    # Add newlines back to match standard ipynb format
    source_lines = [line + "\n" for line in lines[:-1]] + [lines[-1]]
    nb["cells"][target_idx]["source"] = source_lines
    with open("milestone2_final.ipynb", "w", encoding="utf-8") as f:
        json.dump(nb, f, indent=1)
    print(f"Updated cell {target_idx}")
else:
    print("Could not find cell with 'def solve_trim_case'")

