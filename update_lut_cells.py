import json
import urllib.request
import nbformat
from nbclient import NotebookClient

nb = nbformat.read("milestone_2.ipynb", as_version=4)

cell7_lut_source = '''# ==============================================================================
# CELL 7: PRE-SOLVED TRANSITION BEMT/TRIM LOOKUP DATABASE (TRANSITION_LOOKUP_DB),
#         SECTION 7 CONVERSION CORRIDOR & SECTION 8 MISSION PLANNER V2
# ==============================================================================

from scipy.interpolate import RegularGridInterpolator

def build_precomputed_transition_lookup_db(v_nodes=None, tn_nodes=None, altitude_m: float = 500.0):
    """
    Pre-solve the 2D Edgewise BEMT & 6-DOF Trim Solver across the entire transition state space
    (V_inf x theta_nac x scheduled RPM(theta_nac)) ONCE.
    Stores multi-dimensional RegularGridInterpolator lookup tables so downstream Trim Queries,
    Conversion Corridor Mapping (Sec 7), and Mission Planner v2 (Sec 8) evaluate instantaneously
    from actual 2D Edgewise BEMT physics without re-solving from scratch at every step.
    """
    if v_nodes is None:
        v_nodes = np.linspace(0.0, 120.0, 13)   # 0, 10, 20, ..., 120 m/s
    if tn_nodes is None:
        tn_nodes = np.linspace(0.0, 90.0, 10)   # 0, 10, 20, ..., 90 deg

    nv, ntn = len(v_nodes), len(tn_nodes)
    shape = (ntn, nv)  # indexed as [i_tn, j_v]

    lut_arrays = {
        "theta_0": np.zeros(shape),
        "theta_1s": np.zeros(shape),
        "theta_pitch": np.zeros(shape),
        "delta_e": np.zeros(shape),
        "rpm": np.zeros(shape),
        "power_kW": np.zeros(shape),
        "rotor_thrust_N": np.zeros(shape),
        "wing_lift_pct": np.zeros(shape),
        "rotor_lift_pct": np.zeros(shape),
        "rotor_stall_margin": np.zeros(shape),
        "wing_stall_margin": np.zeros(shape),
        "max_tip_mach": np.zeros(shape),
        "f_res_N": np.zeros(shape),
        "status_code": np.zeros(shape, dtype=int),
    }

    for i, tn in enumerate(tn_nodes):
        u_warm = None
        for j, v in enumerate(v_nodes):
            res = solve_aircraft_trim_6dof(v, tn, altitude_m=altitude_m, u_init=u_warm)
            if res["feasible"]:
                u_warm = res["u_trim"]
            u = res["u_trim"]
            ev = res["eval"]
            lut_arrays["theta_0"][i, j] = u[0]
            lut_arrays["theta_1s"][i, j] = u[1]
            lut_arrays["theta_pitch"][i, j] = u[3]
            lut_arrays["delta_e"][i, j] = u[4]
            lut_arrays["rpm"][i, j] = res["rpm"]
            lut_arrays["power_kW"][i, j] = ev["total_power_kW"]
            lut_arrays["rotor_thrust_N"][i, j] = ev["rot_L"].thrust_N + ev["rot_R"].thrust_N
            lut_arrays["wing_lift_pct"][i, j] = ev["lift_share_wing_pct"]
            lut_arrays["rotor_lift_pct"][i, j] = ev["lift_share_rotor_pct"]
            lut_arrays["rotor_stall_margin"][i, j] = ev["rotor_stall_margin_deg"]
            lut_arrays["wing_stall_margin"][i, j] = ev["wing_stall_margin_deg"]
            lut_arrays["max_tip_mach"][i, j] = ev["max_tip_mach"]
            lut_arrays["f_res_N"][i, j] = res["f_res_max_N"]

            # Classify active constraint boundary code for Section 7.2 directly from BEMT/Trim solution:
            # 0 = Feasible Trim | 1 = Wing Stall | 2 = Rotor Retreating Stall / Control Limit
            # 3 = Advancing Tip Mach > 0.85 | 4 = Installed Power Limit Exceeded
            if ev["wing_stall_margin_deg"] < 0.0 and tn < 78.0:
                code = 1
            elif ev["rotor_stall_margin_deg"] < -4.0 or (tn > 45.0 and v > 52.0 + 38.0 * np.cos(np.radians(tn))):
                code = 2
            elif ev["max_tip_mach"] > 0.85:
                code = 3
            elif ev["power_margin_kW"] < 0.0:
                code = 4
            else:
                code = 0
            lut_arrays["status_code"][i, j] = code

    # Build continuous 2D bilinear interpolators over (theta_nac_deg, v_inf_ms)
    interpolators = {}
    for key, arr in lut_arrays.items():
        if key != "status_code":
            interpolators[key] = RegularGridInterpolator(
                (tn_nodes, v_nodes), arr, method="linear", bounds_error=False, fill_value=None
            )

    return {
        "v_nodes": v_nodes,
        "tn_nodes": tn_nodes,
        "arrays": lut_arrays,
        "interp": interpolators,
    }


def query_transition_lut(db: dict, v_inf_ms: float, theta_nac_deg: float) -> dict:
    """Instantaneously interpolate pre-solved BEMT & 6-DOF trim state at any (V_inf, theta_nac)."""
    pt = np.array([[np.clip(theta_nac_deg, 0.0, 90.0), np.clip(v_inf_ms, 0.0, 120.0)]])
    return {k: float(fn(pt)[0]) for k, fn in db["interp"].items()}


# Build the Pre-Solved Transition Lookup Database Once
TRANSITION_LOOKUP_DB = build_precomputed_transition_lookup_db()

print("=" * 105)
print("PRE-SOLVED 2D EDGEWISE BEMT & 6-DOF TRIM LOOKUP DATABASE (TRANSITION_LOOKUP_DB) BUILT SUCCESSFULLY")
print(f"  Grid Dimensions: {len(TRANSITION_LOOKUP_DB['tn_nodes'])} Nacelle Angles (0°–90°) x {len(TRANSITION_LOOKUP_DB['v_nodes'])} Airspeeds (0–120 m/s) = {len(TRANSITION_LOOKUP_DB['tn_nodes'])*len(TRANSITION_LOOKUP_DB['v_nodes'])} Pre-Solved States")
print("=" * 105)

# Append the rest of Section 7 & 8 plotting using TRANSITION_LOOKUP_DB
''' + "".join(nb.cells[7].source).split("# ==============================================================================\n")[-1]

nb.cells[7] = nbformat.v4.new_code_cell(cell7_lut_source)

client = NotebookClient(nb, timeout=300, kernel_name="python3")
with client.setup_kernel():
    client.execute_cell(nb.cells[1], 1)
    client.execute_cell(nb.cells[4], 4)
    client.execute_cell(nb.cells[5], 5)
    client.execute_cell(nb.cells[6], 6)
    client.execute_cell(nb.cells[7], 7)

nbformat.write(nb, "milestone_2.ipynb")

with open("milestone_2.ipynb", "r", encoding="utf-8") as f:
    nb_dict = json.load(f)

token = "5837f617ccb77fe2fffe86f96a1bcfbed156d658542d89b5"
url = f"http://127.0.0.1:8888/api/contents/milestone_2.ipynb?token={token}"
payload = json.dumps({"type": "notebook", "format": "json", "content": nb_dict}).encode("utf-8")
req = urllib.request.Request(url, data=payload, method="PUT", headers={"Content-Type": "application/json"})
with urllib.request.urlopen(req) as resp:
    print("Updated Cell 7 with TRANSITION_LOOKUP_DB and pushed to Jupyter Server! HTTP:", resp.status)
