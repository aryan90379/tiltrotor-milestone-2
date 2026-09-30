# Section 2: Algorithm Architecture & Solver Pipelines

This document breaks down the core architecture of our codebase into the exact pipelines defined in the Milestone presentation. It serves as a mathematical and algorithmic roadmap for how the Aeromechanics Engine, Trim Solver, and Mission Planner function.

---

## 2.1 Edgewise-Flight Performance Estimator (BEMT)
*The 5-stage iterative solver linking non-uniform Coleman inflow with 2D blade-element dynamics.*

### STEP 1: Inputs & Mesh
*   **Flight Inputs:** The function receives Airspeed ($V_{\infty}$), Altitude ($h$), Body Angle of Attack ($\alpha_B$), Nacelle Tilt ($\theta_{\text{nac}}$), and Rotor RPM.
*   **Controls:** The pilot inputs the collective ($\theta_0$) and cyclic pitches ($\theta_{1c}, \theta_{1s}$).
*   **Frame Projection:** The incoming wind is transformed from the Aircraft Body frame into the Rotor Shaft frame using $\alpha_{\text{eff}} = \theta_{\text{nac}} - \alpha_B$.
    *   $V_{\text{axial}} = V_{\infty} \cos(\alpha_{\text{eff}})$
    *   $V_{\text{edge}} = V_{\infty} \sin(\alpha_{\text{eff}})$
*   **2D Polar Mesh:** We slice the rotor disk into an optimized grid of $N_r = 30$ radial rings and $N_\psi = 72$ azimuthal slices ($\Delta\psi = 5^\circ$).

### STEP 2: Inflow Loop
*   **Inflow Model:** We use the **Glauert Momentum Theory** combined with the **Pitt-Peters Skew Model** to calculate the skewed aerodynamic wake.
*   **Fixed-Point Iteration:** Because the inflow ($\lambda_i$) depends on Thrust ($C_T$), and Thrust depends on inflow, we solve this using a `while` loop with an under-relaxation factor of $0.25$. It loops until the error drops below $\Delta\lambda_i < 10^{-5}$.
*   **Tip-Loss & Balance:** We apply the **Prandtl Tip-Loss function $F(r)$** to account for air escaping around the tips of the blades.
*   **Inflow Gradient:** Once the average inflow converges, we apply the longitudinal gradient ($K_x = \frac{4}{3} \frac{\mu / \lambda}{(1.2 + \mu / \lambda)}$).

### STEP 3: Blade Kinematics
*   **Pitch Law (1st-Harmonic):** 
    $$ \theta(r, \psi) = \theta_0 + \theta_{\text{tw}}\left(\frac{r}{R} - 0.75\right) + \theta_{1c} \cos\psi + \theta_{1s} \sin\psi $$
*   **Flapping Kinematics:** We treat the rotor as a **Rigid Disk** ($\beta = 0$). No flapping dynamics are simulated.
*   **Reverse-Flow Region:** The code scans for areas where $U_T \le 0$ (wind hitting the trailing edge) and applies the Viterna-Corrigan $360^\circ$ extrapolation.

### STEP 4: Airfoil & Loads
*   **Sectional Polars:** The solver calculates local Angle of Attack ($\alpha = \theta - \phi$) and queries the **Boeing-Vertol VR-12** tabular database to find Lift ($C_l$) and Drag ($C_d$).
*   **Compressibility:** The code calculates the local Mach number ($M = U/a$). If $M > 0.75$, it applies the **Prandtl-Glauert** scaling correction.
*   **Elemental Loads [N/m]:** We convert the coefficients into raw physical forces using $dF = \frac{1}{2}\rho U^2 c C_{(l,d)}$.

### STEP 5: Integration & CG
*   **Azimuthal Integration:** The code uses the **2D Periodic Trapezoidal Rule** to integrate forces across all $30 \times 72$ cells. 
*   **Body Frame Transform:** A 3D Euler Rotation Matrix ($R_{B \leftarrow S}$) maps the shaft forces ($T, Q, P$) directly onto the Aircraft Center of Gravity (CG).

---

## 2.2 Trim Solver
*Bounded Trust-Region Dogleg root-finder solving 6-DOF aircraft equilibrium by matching dual counter-rotating BEMT proprotors and airframe aerodynamics.*

### STEP 1: Conversion & Trim
*   **Fixed Flight Inputs:** $V_\infty, h(\text{ISA}), \rho, \theta_{\text{nac}}, \text{RPM}, \text{GW}$.
*   **Unknown Trim Vector (u):** We solve for 6 unknowns to balance 6 degrees of freedom:
    $$ u = [\theta_0, \theta_{1s}, \theta_{1c}, \theta_{\text{pitch}}, \delta_e, \delta_a]^T $$
*   **Control Bounds:** The solver is physically constrained: Collective $\theta_0 \in [-8^\circ, 40^\circ]$, Pitch $\theta_{\text{pitch}} \in [-12^\circ, 30^\circ]$, Elevator $\delta_e \in [-25^\circ, 25^\circ]$.

### STEP 2: Dual Rotor BEMT
*   **Wingtip Proprotors:** The code mounts two engines. The Port rotor spins CCW ($k=+1$), and the Starboard rotor spins CW ($k=-1$).
*   **Symmetric Flight:** To fly straight, the lateral cyclic ($\theta_{1c}$) and ailerons ($\delta_a$) are driven to $0$. Because the rotors are mirrored, the massive 800 kN-m Roll Moment ($M_X$) and Yaw Moment ($M_Z$) perfectly cancel each other out ($F_Y=0, M_X=0, M_Z=0$).
*   **Wash Interference:** Downwash from the rotors onto the wings ($\Delta\alpha_{\text{wash}}$) is accounted for.

### STEP 3: Airframe Aero
*   **Wing Aerodynamics:** The code calculates the wing's angle of attack $\alpha_w = \theta_{\text{pitch}} - \gamma + i_w - \Delta\alpha_{\text{wash}}$. Lift and Drag are evaluated using $L_w = \frac{1}{2} \rho V_\infty^2 S_w C_{L,w}$.
*   **Parasite Drag:** The bluff-body drag of the fuselage is calculated using the equivalent flat plate area: $D_{\text{par}} = \frac{1}{2} \rho V_\infty^2 f_{\text{eq}}$.
*   **Empennage & Pitch Control:** The horizontal tail provides pitch stability. The dynamic pressure at the tail is reduced to $95\%$ due to the fuselage wake ($q_{\text{tail}} = q_\infty \times 0.95$).

### STEP 4: Residual Vector R(u)
*   **6-DOF Force & Moment Residuals:** The code sums all aerodynamic forces and subtracts the weight vector $W$:
    *   $F_X: F_{X,\text{rot}} + F_{X,\text{aero}} - W \sin(\theta_{\text{pitch}}) = 0$
    *   $F_Z: F_{Z,\text{rot}} + F_{Z,\text{aero}} - W \cos(\theta_{\text{pitch}}) = 0$
    *   $M_Y: M_{Y,\text{rot}} - L_h l_h - D_w l_{D,w} = 0$

### STEP 5: Solver & Feasibility
*   **Root Solver:** Uses `scipy.optimize.root` (Levenberg-Marquardt or Dogleg) to drive $R(u) \to 0$.
*   **Convergence Gate:** Requires Force residuals $< 1.0$ N and Moment residuals $< 1.0$ N-m. Usually converges in 4-7 iterations.
*   **Feasibility Checks:** Once trimmed, the state is verified against engine limits ($P_{\text{req}} < P_{\text{avail}}$), stall limits ($\alpha_{\text{stall}} < 15.5^\circ$), and acoustic limits ($M_{\text{tip}} < 0.85$).

---

## 2.3 Mission Planner v2
*Quasi-steady time-stepped simulation integrating corridor-guided nacelle conversion, warm-started 6-DOF trim solutions, and SFC fuel-accounting.*

### STEP 1: Profile & Schedule
*   **Mission Architecture:** Hover Takeoff ($0$ m/s, $90^\circ$) $\to$ Outbound $\to$ Cruise ($0^\circ$) $\to$ Descent $\to$ Inbound $\to$ Hover Landing.
*   **Time-Stepped State Vector:** Every second, the state $S_k = [t_k, x_k, h_k, V_k, \gamma_k, a_k, \theta_{\text{nac}, k}, W_k, m_{\text{fuel}, k}]$ is recorded.
*   **Conversion Corridor:** The code enforces a Cosine-blended conversion schedule, ensuring RPM maps safely from 535 (Hover) down to 400 (Cruise) as the nacelles tilt from $90^\circ \to 0^\circ$.

### STEP 2: Accel & 6-DOF Trim
*   **Quasi-Steady Demand:** The kinematic demands $a_k = \frac{dV}{dt}$ and $\gamma_k = \arctan(\frac{dh}{dx})$ are used to inject fictitious inertial forces into the Trim Solver ($F_{\text{inertial}, X} = m a_k$).
*   **Online Warm-Start Trim:** To ensure the simulation runs in seconds instead of hours, the Trim Solver is *warm-started* by injecting the previous time-step's solution ($u_{k-1}^*$) as the initial guess for the current step. This drops convergence down to 2-4 iterations.

### STEP 3: Feasibility Gate
*   **Active Envelope Screening:** Every single second of the flight is screened against the physical bounds defined in Section 2.2.
*   **State Rejection Logic:** If a waypoint requires more power than the engines can produce, or stalls the rotor, the simulation rejects it and flags the constraint margin.

### STEP 4: Power & Mass Update
*   **Total Shaft Power:** $P_{\text{total}} = (P_{\text{rot},L} + P_{\text{rot},R}) / \eta_{\text{gb}} + P_{\text{acc}}$ (assuming a gearbox efficiency $\eta_{\text{gb}} = 0.96$).
*   **SFC Fuel Integration:** The Turboshaft Specific Fuel Consumption (SFC) is integrated over the time-step: 
    $$ \Delta m_{\text{fuel}, k} = \left(\frac{\text{SFC}}{3600}\right) \times P_{\text{total}}(t_k) \times \Delta t $$
*   **Gross Weight Update:** The aircraft physically gets lighter as fuel is burned! $W_{k+1} = (W_0 - \Delta m_{\text{fuel}, k}) \times g$. This makes hovering at the end of the mission much easier than at takeoff.

### STEP 5: Kinematics & Telemetry
*   **Trajectory Integration:** Forward Euler integration maps velocities to physical GPS coordinates: $x_{k+1} = V_k \cos(\gamma_k) \Delta t + x_k$.
*   **Boundary Continuity:** Enforces $C_0$ state and control continuity across all flight segment boundaries (no teleporting or jerky control inputs).
*   **Telemetry Output:** Exports the complete time-history arrays for visualization in Section 8.