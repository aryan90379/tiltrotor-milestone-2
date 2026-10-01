# Tiltrotor Aeromechanics & Mission Simulation (Milestone 2)

This repository contains the complete 6-Degrees-of-Freedom (6-DOF) Trim Solver, Edgewise BEMT Aerodynamics Engine, and Mission Planner for a tandem tiltrotor aircraft. The documentation below serves as a highly detailed mathematical and physical walkthrough of the code and the resulting aerodynamic phenomena.

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

---

# Section 3: Code-Level Walkthrough (The Verification Suite)

This document explains the rigorous mathematical and aerodynamic pipeline used to generate the graphs in **Section 3 (Verification)**. 

---

## The Theory: How "Edgewise BEMT" Actually Works
The `run_edgewise_bemt` function is the core physics engine of Milestone 2. In edgewise flight, the rotor disk experiences an asymmetric velocity field. The code solves this using a 5-step Blade Element Momentum Theory (BEMT) pipeline.

**1. The 2D Mesh Grid & Velocity Decomposition:**
The continuous rotor disk is discretized into a 2D mesh grid with $N_r = 30$ radial rings and $N_\psi = 72$ azimuthal sectors.
The incoming free-stream velocity $V_\infty$ is decomposed onto the tilted rotor disk (where $\alpha_{\text{eff}} = \theta_{\text{nac}} - \alpha_{\text{body}}$):
$$ V_{\text{edge}} = V_\infty \sin(\alpha_{\text{eff}}) \quad \text{(In-plane velocity)} $$
$$ V_{\text{axial}} = V_\infty \cos(\alpha_{\text{eff}}) \quad \text{(Perpendicular velocity)} $$
The advance ratios are defined as $\mu = V_{\text{edge}} / (\Omega R)$ and $\mu_z = V_{\text{axial}} / (\Omega R)$.

**2. The Swashplate Kinematics:**
For every cell $(r, \psi)$, the local geometric blade pitch is determined by the collective ($\theta_0$), built-in twist ($\theta_{\text{tw}}$), and cyclic inputs ($\theta_{1c}, \theta_{1s}$):
$$ \theta(r, \psi) = \theta_0 + \theta_{\text{tw}}\left(\frac{r}{R} - 0.75\right) + \theta_{1c}\cos\psi + \theta_{1s}\sin\psi $$

**3. The Glauert & Pitt-Peters Inflow Model:**
Due to the skewed wake in forward flight ($\chi = \arctan(\frac{\mu}{\mu_z + \lambda_{i0}})$), the induced downwash is asymmetric. The code uses a fixed-point `while` loop to solve Glauert's momentum equation for the mean inflow $\lambda_{i0}$:
$$ \lambda_{i0} = \frac{C_T}{2 \sqrt{\mu^2 + (\mu_z + \lambda_{i0})^2}} $$
Once converged, the Pitt-Peters longitudinal gradient $K_x$ is applied to skew the downwash toward the rear of the disk:
$$ K_x = \frac{4}{3} \frac{1 - \cos\chi - 1.8\mu^2}{\sin\chi} $$

**4. Local Aerodynamic Environment ($U_T, U_P$):**
The local velocity components hitting the blade element are calculated as:
$$ U_T(r, \psi) = \Omega r + V_{\text{edge}}\sin\psi \quad \text{(Tangential)} $$
$$ U_P(r, \psi) = \Omega R \left( \mu_z + \lambda_{i0}\left(1 + K_x \frac{r}{R}\cos\psi\right) \right) \quad \text{(Perpendicular)} $$
The local angle of attack is $\alpha = \theta - \phi$, where the inflow angle is $\phi = \arctan(U_P / U_T)$.
The Mach number is evaluated as $M = \frac{\sqrt{U_T^2 + U_P^2}}{a}$. These variables query the VR-12 airfoil tables for $C_l$ and $C_d$.

**5. 2D Integration & Tip-Loss:**
Prandtl's tip-loss function $F(r)$ is applied to account for 3D spanwise flow:
$$ F(r) = \frac{2}{\pi}\arccos(e^{-f}), \quad f = \frac{N_b}{2} \frac{1 - r/R}{(r/R) \sin\phi} $$
The total thrust $T$ is integrated using the 2D Periodic Trapezoidal Rule over the entire domain:
$$ T = \frac{N_b}{2\pi} \int_{0}^{2\pi} \int_{R_{\text{root}}}^{R} \frac{1}{2}\rho (U_T^2 + U_P^2) c (C_l \cos\phi - C_d \sin\phi) F(r) \, dr \, d\psi $$

---

## 3.1 Recovery of Milestone 1 Limiting Cases
**The Goal:** Prove that the 2D edgewise solver mathematically reduces to the 1D axisymmetric solver when $V_{\text{edge}} = 0$.

### Graph 3.1(a) and 3.1(b): Spanwise Thrust & Power Loading
*   **X-axis:** `Radial Station r [m]`.
*   **Y-axis:** Thrust Loading $dT/dr$ [N/m] and Power Loading $dP/dr$ [kW/m].
    *   *Mathematical Cause of the Curve:* The curve drops sharply to zero at the tip due to the Prandtl Tip-Loss function $F(r) \to 0$ as $r \to R$.
*   **The Results:** 
    *   `Dashed Red Line` (2D Hover) perfectly overlays the `Solid Black Line` (1D Hover) with an error of $\Delta T = 0.0050\%$.
    *   `Dashed Cyan Line` (2D Cruise) perfectly overlays the `Solid Blue Line` (1D Cruise) with an error of $\Delta P = 0.0037\%$. This proves the azimuthal integration introduces zero spurious drift.

---

## 3.2 Azimuthal Loading & Periodicity
**The Goal:** Visualize the extreme aerodynamic asymmetry that occurs when the helicopter flies forward at **$V_\infty = 60$ m/s** (with Nacelles tilted to **$\theta_{\text{nac}} = 75^\circ$**, giving an advance ratio **$\mu = 0.25$**), creating the "Dissymmetry of Lift".

### Graph 3.2(a): Normal Sectional Load Contour $dF_z/dr$ [N/m]
*   **The Plot:** A top-down heatmap of the rotor disk. The center is the hub ($r=0$), and the outer edge is the blade tip ($r=4.58$ m). 
    *   **The Thick White Dashed Circle (Left Side):** If you look at the exact center of the crosshairs, you'll see a thick white dashed circle that touches the center and bulges out to the left side (the Retreating side). This is the **Reverse Flow Boundary** ($U_T = 0$). Inside this white dashed circle, the helicopter is flying forward so fast that the 60 m/s wind is actually blowing *backwards* over the retreating blade! This is exactly why the entire area inside that dashed circle is dark blue/purple—the lift has violently crashed and actually gone negative!
*   **The Math:** This plots the vertical force distribution:
$$ \frac{dF_z}{dr} = \frac{1}{2}\rho (U_T^2 + U_P^2) c (C_l \cos\phi - C_d \sin\phi) F(r) $$
*   **The Physics:** You can clearly see a massive red "hotspot" on the right side (the Advancing Side, $\psi = 90^\circ$). Here, the blade's rotation speed ($\Omega r$) adds directly to the helicopter's forward speed ($V_{\infty}$), resulting in a massive tangential velocity ($U_T$). Since Lift scales with $U_T^2$, the lift explodes. Conversely, the left side (Retreating, $\psi = 270^\circ$) is dark blue because the speeds subtract, killing the lift.

### Graph 3.2(b): In-Plane Torque Load Contour $dF_\psi/dr$ [N/m]
*   **The Plot:** A top-down heatmap showing the in-plane drag forces trying to slow the rotor down.
*   **The Math:** This plots the horizontal force distribution:
$$ \frac{dF_\psi}{dr} = \frac{1}{2}\rho (U_T^2 + U_P^2) c (C_l \sin\phi + C_d \cos\phi) F(r) $$
*   **The Physics:** Notice that the drag is also heavily biased to the advancing right side. The engine has to fight through this asymmetric "wall of air" on the right side every time a blade spins through it. The integration of this contour gives us the total Shaft Torque ($Q$).

### Graph 3.2(c): Azimuthal 2π Periodicity & Roll Moment Origin
*   **X-axis:** Blade Azimuth Angle $\psi$ [deg]. The graph tracks a single blade across two full revolutions ($0^\circ \to 720^\circ$).
*   **Y-axis:** Integrated Single-Blade Thrust $T_{\text{blade}}(\psi) = \int_{R_{\text{root}}}^R \frac{dF_z}{dr} \, dr$.
*   **The Physics:** This graph perfectly summarizes the physics of the contour maps. The thrust forms a heavily skewed $1\text{P}$ (once-per-revolution) harmonic sine wave. 
    *   At the **Advancing Sector** (Yellow highlight, $\psi=90^\circ$), the single blade generates a peak lift of **56.6 kN**.
    *   At the **Retreating Sector** (Red highlight, $\psi=270^\circ$), the lift crashes to a minimum of **0.6 kN**.
    *   The **Cycle-Averaged Mean Blade Lift** (the red dashed line) sits exactly at **22.64 kN**.
    *   Because the right side lifts 90x harder than the left side, it physically generates the massive $-127.9$ kN-m Roll Moment ($M_X$) that tries to violently flip the aircraft over. This proves why cyclic pitch is strictly required for forward flight!

## 3.3 Reverse Flow, Stall, and Mach Limits
**The Goal:** Map the physical boundaries where the aerodynamics break down in fast forward flight. 

### Graph 3.3(a): The Reverse Flow Boundary
*   **The Plot:** Shows a contour map of the Tangential Velocity ($U_T$). A thick dashed black circle is drawn exactly where $U_T = 0$.
*   **The Math:** Reverse flow occurs strictly when $U_T \le 0$. Solving the velocity equation $U_T(r, \psi) = \Omega r + V_{\text{edge}}\sin\psi = 0$ yields the geometric boundary of a perfect circle on the retreating side:
$$ r(\psi) = -\mu R \sin\psi $$
*   **The Physics:** The helicopter is flying forward at 60 m/s. But near the root of the blade, the rotation speed ($\Omega r$) is only 20 m/s. Because the blade is spinning backward at 20 m/s but the helicopter is moving forward at 60 m/s, the wind actually hits the *trailing edge* (the sharp back) of the blade at 40 m/s! 
*   **The Code Solution:** Inside this circle, standard airfoil tables break down. The solver applies the **Viterna-Corrigan $360^\circ$ extrapolation**, a mathematical trick that allows the code to calculate lift and drag even when the air hits the wing completely backward.

### Graph 3.3(b): The Stall Boundary (The Figure-8)
*   **The Plot:** A contour map of the local Angle of Attack ($\alpha$). A thick **Green Contour Line** is drawn wherever $|\alpha| \ge 15.8^\circ$, perfectly outlining the regions that have exceeded the VR-12 airfoil stall limit.
*   **The Physics:** It forms a bizarre "Figure-8" (or $\infty$) shape because the rotor is stalling in two places for two totally different reasons:
    1.  **The Left Lobe (Retreating Stall):** On the left side, the blade is moving so slowly (due to reverse flow) that the downward wind (downwash, $U_P$) hits it almost completely vertically. This causes the inflow angle to approach $90^\circ$ ($\phi = \arctan(U_P/U_T) \to 90^\circ$), causing a massive, catastrophic stall.
    2.  **The Right Lobe (Advancing Root Stall):** Tiltrotor blades have a severe $-30^\circ$ twist built into them (so they can act like airplane propellers later). This means the root is permanently pitched up to an extreme $33.5^\circ$. On the advancing side, the root simply catches too much air and stalls before the inflow angle can reduce it.

### Graph 3.3(c): Advancing Tip Mach Number
*   **The Plot:** A contour map of the local Mach number.
*   **The Math:** $M = \frac{\sqrt{U_T^2 + U_P^2}}{a_{\text{sound}}}$. A **Yellow Dash-Dot Contour Line** specifically highlights the $M = 0.75$ Drag Divergence ($M_{dd}$) boundary and the peak $M = 0.84$ location at the extreme right tip ($r=R, \psi=90^\circ$).
*   **The Physics:** At the extreme right edge, the blade's rotation speed adds to the helicopter's forward speed. The tip velocity approaches the speed of sound ($M = 0.84$). 
*   **The Design Implication:** Crossing the $M = 0.75$ boundary means shockwaves are forming on the blade, causing massive Wave Drag. This is the primary physical reason helicopters cannot fly faster than ~200 mph! To go faster, our tiltrotor must tilt its engines forward and slow down its RPM (which we do during Conversion).


### 3.3(d) Under the Hood: The Airfoil Physics Engine
To generate the physics seen in the graphs above, the code (`AirfoilDatabase.evaluate`) dynamically switches between three different aerodynamic models depending on the local flow state:

1.  **Clean Flow (XFOIL Tabular Lookup):**
    For normal flight angles ($-15^\circ \le \alpha \le +20^\circ$), the code uses a massive folder of pre-computed **Boeing-Vertol VR-12** aerodynamic tables generated by XFOIL. It uses a 2D Bivariate Spline to instantly look up the exact Lift ($C_l$) and Drag ($C_d$) based on the current Angle of Attack and Reynolds Number.
2.  **Compressibility (Prandtl-Glauert):**
    To simulate the massive wave drag seen in **Graph 3.3(c)**, the code calculates the local Mach number ($M$) everywhere on the disk. It then applies the Prandtl-Glauert scaling law ($\beta = \sqrt{1 - M^2}$) to the tabular data. This physically accurate transformation correctly predicts the catastrophic drag spike near the speed of sound.
3.  **Reverse Flow & Deep Stall (Viterna-Corrigan $360^\circ$ Extrapolation):**
    XFOIL tables crash if you ask them for lift when the wind is blowing backwards! If the blade enters the Reverse Flow region (**Graph 3.3a**) or Deep Stall (**Graph 3.3b**), the `valid_mask` detects that $\alpha$ has exceeded $20^\circ$. The code instantly throws away the tables and falls back to the **Viterna-Corrigan flat-plate approximation**:
    $$ C_{L,\text{stall}} = (C_{L,\text{max}} \times 0.95) \sin(2\alpha) $$
    $$ C_{D,\text{stall}} = 0.015 + 1.25 \sin^2(\alpha) $$
    This flawlessly blends the standard aerodynamics into a $360^\circ$ analytical model, allowing the aircraft to fly through extreme reverse-flow wind without the simulation crashing!

## 3.4 Discretization Sensitivity (Grid Size Verification)
**The Goal:** Mathematically prove that our chosen 2D mesh grid size ($30 \times 72$) is dense enough to perfectly capture the physics without wasting computational time. We ran a massive `for` loop, testing dozens of grid sizes, and plotted the errors.

### Sec 3.4 Plots 1 & 2: Radial Sensitivity ($N_r$)
*   **The Plot:** The top two graphs show Total Thrust ($T$) and Total Torque ($Q$) on the Y-axis versus the number of radial rings ($N_r$) on the X-axis (from 8 to 80).
*   **The Results:** Both the Thrust (blue) and Torque (red) curves take a relatively long time to level out (converge). They don't approach the true mathematical asymptote (the horizontal dotted line) until around $N_r \ge 30$.
*   **The Physics / Math Cause:** Why is it so slow to converge? Because of the **Prandtl Tip-Loss function**. At the extreme outer edge of the blade ($r \to R$), the lift literally drops off a mathematical cliff. If your radial grid isn't dense enough, the code will accidentally draw a smooth hill instead of a sharp cliff, massively overestimating the thrust. We placed our Red Dashed line at $N_r = 30$ because it is dense enough to accurately map that cliff with less than $0.14\%$ error.

### Sec 3.4 Plots 3 & 4: Azimuthal Sensitivity ($N_\psi$)
*   **The Plot:** The bottom two graphs show Thrust and Torque versus the number of pie-slices around the circle ($N_\psi$).
*   **The Results:** Look at the curves—they crash straight down and perfectly hit the asymptote almost instantly! The math stabilizes at just $N_\psi \ge 12$. 
*   **The Physics / Math Cause:** Why does it converge so fast? Because as the blade spins around the circle, the lift changes in a perfectly smooth, predictable wave (dominated by $1\text{P}$ and $2\text{P}$ trigonometric harmonics like $\sin\psi$ and $\cos\psi$). The specific mathematical integration method we used (the **Periodic Trapezoidal Rule**) exhibits what mathematicians call *Exponential Spectral Convergence* when applied to perfectly smooth, periodic sine waves. It solves them flawlessly with very few slices. We chose $N_\psi = 72$ (Red Dashed line) not for accuracy, but just to make the contour heatmaps in Section 3.2 look smooth and pretty!





---

# Section 4: Parametric Sweeps & Database Architecture

This document breaks down the mathematical foundation of the 6D Aerodynamic Database and the equations governing the pilot-input parametric sweeps in **Section 4**.

---

## Part 1: The 6D Aerodynamic Database Architecture
To enable real-time 6-DOF aircraft trimming in later sections, the BEMT solver was executed offline $5,292$ times to populate `tiltrotor_rotor_database.csv`.

**The Transformation Equations:**
The BEMT solver calculates forces and moments in the **Shaft Frame ($S$)**. To plot them in Section 4, they are transformed into the **Aircraft Body Frame ($B$)** using the nacelle tilt angle $\theta_{\text{nac}}$:
$$ \mathbf{R}_{B \leftarrow S} = \begin{bmatrix} \cos\theta_{\text{nac}} & 0 & -\sin\theta_{\text{nac}} \\ 0 & 1 & 0 \\ \sin\theta_{\text{nac}} & 0 & \cos\theta_{\text{nac}} \end{bmatrix} $$
$$ \begin{bmatrix} F_X \\ F_Y \\ F_Z \end{bmatrix}_{\text{Body}} = \mathbf{R}_{B \leftarrow S} \begin{bmatrix} H \\ Y \\ -T \end{bmatrix}_{\text{Shaft}} $$

**The Integral Equations inside the Database:**
*   **Thrust ($T$):** $$ T = \frac{N_b}{2\pi} \int_{0}^{2\pi} \int_{R_{\text{root}}}^{R} \frac{dF_z}{dr} \, dr \, d\psi $$
*   **Roll Moment ($M_X$):** Induced heavily by the advancing/retreating lift asymmetry.
    $$ M_X = - \frac{N_b}{2\pi} \int_{0}^{2\pi} \int_{R_{\text{root}}}^{R} \left( \frac{dF_z}{dr} \right) r \sin\psi \, dr \, d\psi $$
*   **Pitch Moment ($M_Y$):** Induced by fore/aft lift asymmetry.
    $$ M_Y = - \frac{N_b}{2\pi} \int_{0}^{2\pi} \int_{R_{\text{root}}}^{R} \left( \frac{dF_z}{dr} \right) r \cos\psi \, dr \, d\psi $$
*   **Shaft Power ($P_{\text{req}}$):** Derived from the in-plane aerodynamic drag torque ($Q$).
    $$ Q = \frac{N_b}{2\pi} \int_{0}^{2\pi} \int_{R_{\text{root}}}^{R} \left( \frac{dF_\psi}{dr} \right) r \, dr \, d\psi \implies P_{\text{req}} = Q \cdot \Omega $$

---

## ## Part 2: The Parametric Sweeps (Rows 1 to 3)

### The Two Flight States (Solid vs. Dashed)
Every graph in this section plots two distinct aircraft states simultaneously to compare them:
*   **Solid Lines (Helicopter Mode):** Nacelles pointing straight up ($90^\circ$), $V=30$ m/s, High RPM (535).
*   **Dashed Lines (Intermediate Conversion):** Nacelles tilted half-forward ($45^\circ$), $V=60$ m/s, Low RPM (400). Because the engines are tilted $45^\circ$, the rotors now act partially like airplane propellers, generating massive Forward Thrust ($F_X$) alongside Vertical Lift ($F_Z$), while offloading weight to the wings to save Power.

The goal of Section 4 is to prove that our aerodynamic code responds correctly to pilot inputs before we hand the model over to the 6-DOF Trim Solver. We sweep each of the three main rotor controls (Collective, Longitudinal Cyclic, and Lateral Cyclic) from $-10^\circ$ to $+10^\circ$ while freezing the others.

### Row 1: The 4.1 Collective Sweep ($\theta_0$)
**The Control:** The collective pitch ($\theta_0$) physically rotates all 3 blades up or down by the exact same amount simultaneously. The pitch equation shifts uniformly:
$$ \theta(r, \psi) = \theta_0 + \theta_{\text{tw}}\left(\frac{r}{R} - 0.75\right) + \theta_{1c} \cos\psi + \theta_{1s} \sin\psi $$

This row contains 4 specific graphs detailing the aircraft's response to Collective input:

**Graph 1: Body Forces [kN]**
*   **X-axis:** Collective Pitch $\theta_0$ ($0^\circ \to 25^\circ$).
*   **Y-axis:** Force [kN] in the Body Frame.
*   **The Lines & Physics:** 
    *   **Solid Red Line (Helicopter $F_Z$):** Plummets downward linearly. In our coordinate system, $Z$ points DOWN. So a highly negative $F_Z$ means the rotor is generating massive upward Lift! 
    *   **Solid Blue Line (Helicopter $F_X$):** Stays perfectly flat at zero. In pure helicopter mode ($90^\circ$), pulling collective only lifts you up, it does not push you forward.
    *   **Dashed Blue Line (Conversion $F_X$):** Explodes upward into the positive! Because the engines are tilted $45^\circ$ forward, pulling collective now acts like an airplane propeller, yanking the aircraft violently forward.
    *   **Dashed Red Line (Conversion $F_Z$):** Goes downward, but much less steeply than the solid red line. Since the engines are tilted $45^\circ$, half of the thrust vector is being wasted on pushing the aircraft forward instead of lifting it up.

**Graph 2: Body Moments about CG [kN-m]**
*   **X-axis:** Collective Pitch $\theta_0$.
*   **Y-axis:** Moment [kN-m] in the Body Frame.
*   **The Lines & Physics:**
    *   **Solid Purple Line (Helicopter $M_X$):** Explodes massively into the positive. This beautifully illustrates the **Dissymmetry of Lift**. The advancing side grabs the extra collective pitch and multiplies it by the forward airspeed, generating exponentially more lift than the retreating side. This tries to violently roll the helicopter!
    *   **Dashed Orange Line (Conversion $M_Y$):** Goes negative (Nose Down pitch). Because the engines are tilted $45^\circ$, pulling collective generates massive forward thrust ($F_X$). Because the engines are mounted on the wingtips *above* the Center of Gravity, pushing forward from the top makes the nose pitch down.

**Graph 3: Rotor Shaft Power [kW]**
*   **X-axis:** Collective Pitch $\theta_0$.
*   **Y-axis:** Required Shaft Power [kW].
*   **The Lines & Physics:**
    *   **Solid Blue Line (Helicopter):** Takes a severe, non-linear parabolic shape. According to Momentum Theory, Induced Power relates to Thrust by $P_i \propto T^{3/2}$. As the collective (Graph 1) pushes thrust up linearly, aerodynamic drag and induced power explode exponentially.
    *   **Dashed Red Line (Conversion):** Stays perfectly flat at 0 kW, then skyrockets. Why doesn't it cost any power at low pitch? 
        *   **Windmill Mode ($0^\circ \to 10^\circ$):** In Conversion, the aircraft is flying fast (60 m/s) with the engines tilted $45^\circ$ forward. At low pitch, the blades are relatively flat. The incoming 60 m/s wind hits the blades and pushes them around on its own, just like a windmill! Because the wind is doing the work to keep the blades spinning at 400 RPM, the turboshaft engine doesn't have to provide any torque (0 kW). *(Note: While this costs 0 power, Graph 1 shows you are also generating 0 Lift, meaning the helicopter is in freefall!).*
        *   **Propeller Mode ($15^\circ+$):** As you pull the collective up, the blades angle sharply to bite into the air and generate lift/thrust. This creates massive aerodynamic drag trying to stop the blades from spinning. To keep them spinning at 400 RPM against that huge drag, the engine must suddenly kick in and burn fuel, causing the power curve to explode!

**Graph 4: Unstalled Disk Area [%]**
*   **X-axis:** Collective Pitch $\theta_0$.
*   **Y-axis:** Percentage of the rotor disk that is NOT stalled.
*   **The Lines & Physics:**
    *   **Solid Blue Line (Helicopter):** Notice how the line actually *increases* from 87% up to 95% before it crashes! Why? At exactly $0^\circ$ collective, the blades are totally flat, meaning the 30 m/s wind is actually hitting the *top* of the retreating blades, causing a small "negative" stall. Pulling the collective to $5^\circ$ tilts the blades up just enough to perfectly catch the wind, pushing the rotor into its most efficient, 95% clean aerodynamic zone. But once you pull past $10^\circ$, the angle gets too steep, triggering a massive "positive" stall that crashes the line. 
    *   **Dashed Red Line (Conversion):** Starts deeply stalled (35%) and actually *improves* as you pull collective! Why? At low collective ($0^\circ$), the fast $60$ m/s wind is hitting the *top* of the tilted blades (a massive negative angle of attack), causing a negative stall. As you pull collective up, you increase the blade pitch into the clean, positive aerodynamic region.

### Row 2: The 4.2 Longitudinal Cyclic Sweep ($\theta_{1s}$)
**The Control:** Longitudinal cyclic applies a sine-wave variation to the blade pitch as it spins: $\Delta\theta = \theta_{1s} \sin\psi$. Because $\sin(90^\circ) = 1$ and $\sin(270^\circ) = -1$, this control strictly changes the pitch on the left and right sides of the helicopter.

**Graph 5: Body Forces [kN]**
*   **The Lines & Physics:** Notice that the Thrust lines ($F_Z$, Red) stay perfectly flat horizontally. This proves **Thrust Decoupling**. Because we are mathematically adding pitch to the right side and subtracting the exact same amount of pitch from the left side, the total average lift of the helicopter remains completely unchanged. You can move the joystick without suddenly gaining or losing altitude!

**Graph 6: Body Moments about CG [kN-m]**
*   **The Lines & Physics:**
    *   **Solid Purple Line ($M_X$ Roll):** Stays massively high (around 800 kN-m) and slopes downward slightly as $\theta_{1s}$ increases. Why doesn't it cross zero? Because the single rotor being tested in Section 4 is mathematically mounted on the **Port (Left) Wingtip**. The engine is generating $-80$ kN of upward lift at a distance of $-10$ meters from the Center of Gravity. $(-80 \text{ kN} \times -10 \text{ m} = +800 \text{ kN-m})$. That massive purple line is literally just the left engine lifting the left wing! The cyclic stick actively changes the aerodynamic roll moment of the disk (causing the $\pm 100$ kN-m slope), but it will never cross zero because the engine is still holding up the wing. (In the real tiltrotor in Section 6, the Starboard engine generates $-800$ kN-m to perfectly cancel this out).
    *   **The "Rigid Rotor" Control Anomaly:** Notice that the Orange Line ($M_Y$ Pitch) stays totally flat! In a real helicopter, longitudinal cyclic pitches the nose up or down. But here, the stick strictly alters the Purple line (Roll). Why? Because our code uses a **Rigid Disk** ($\beta = 0$, no flapping hinges). Without hinges, there is no $90^\circ$ gyroscopic phase delay. The lift changes on the left and right sides exactly where the pitch is applied, causing pure Roll!

**Graph 7: Rotor Shaft Power [kW]**
*   **X-axis:** Longitudinal Cyclic $\theta_{1s}$.
*   **Y-axis:** Required Shaft Power [kW].
*   **The Lines & Physics:** 
    *   **Solid Blue Line (Helicopter):** Forms a steady upward slope, increasing from roughly 1300 kW to 1600 kW. As you push the stick forward (positive $\theta_{1s}$), you are actively adding blade pitch to the advancing right side of the rotor. Because that side is already flying into a 30 m/s headwind, adding pitch to it causes a massive spike in aerodynamic drag, forcing the engine to work much harder.
    *   **Dashed Red Line (Conversion):** Forms a much steeper upward slope (from 0 to over 600 kW). In conversion mode, the aircraft is flying even faster (60 m/s). Pushing the stick forward forces the advancing side to take a massive bite out of that 60 m/s wind, which creates extreme drag and violently spikes the engine power.

**Graph 8: Unstalled Disk Area [%]**
*   **X-axis:** Longitudinal Cyclic $\theta_{1s}$.
*   **Y-axis:** Percentage of the rotor disk that is NOT stalled.
*   **The Lines & Physics:** 
    *   **Solid Blue Line (Helicopter):** Notice that the line slopes *upward* from 85% at $-6^\circ$ to peak at 87% at $0^\circ$. At $-6^\circ$, the retreating side is pitched too high and experiencing localized stall. As you bring the stick back to the center ($0^\circ$), you remove that extreme pitch, "healing" the stall and increasing the clean area to 87%. But as you push the stick forward past $0^\circ$, you add extreme pitch to the advancing side, forcing it past the $15.8^\circ$ limit and crashing the clean area down to 74%.
    *   **Dashed Red Line (Conversion):** Plummets in a straight line as you push the stick forward. In conversion mode, the wind is already hitting the tilted blades at a weird angle. Forcing the cyclic pitch higher aggressively stalls the advancing blade.

### Row 3: The 4.3 Lateral Cyclic Sweep ($\theta_{1c}$)
**The Control:** Lateral cyclic applies a cosine-wave variation to the blade pitch: $\Delta\theta = \theta_{1c} \cos\psi$. Because $\cos(180^\circ) = -1$ (Nose) and $\cos(0^\circ) = 1$ (Tail), this control strictly alters the lift at the front and rear of the rotor disk.

**Graph 9: Body Forces [kN]**
*   **X-axis:** Lateral Cyclic $\theta_{1c}$.
*   **Y-axis:** Force [kN] in the Body Frame.
*   **The Lines & Physics:** 
    *   **Solid & Dashed Lines:** The Thrust lines ($F_Z$, Red) remain mostly flat, hovering around -90 kN and -80 kN. Because we are adding lift to the nose and subtracting from the tail, the total average lift remains relatively stable. You can pitch the aircraft up and down without accidentally gaining massive altitude.

**Graph 10: Body Moments about CG [kN-m]**
*   **X-axis:** Lateral Cyclic $\theta_{1c}$.
*   **Y-axis:** Moment [kN-m] in the Body Frame.
*   **The Lines & Physics:** 
    *   **Solid & Dashed Orange Lines ($M_Y$ Pitch):** Both cross zero with a steep positive slope. By altering the front/rear lift distribution, the pilot can actively pitch the nose of the aircraft up or down in both Helicopter and Conversion modes.
    *   **The Rigid Rotor Swap:** In a real helicopter, Lateral Cyclic is used to Roll left and right. But because of our rigid disk assumption, Lateral Cyclic gives us pure Pitch control! The controls are perfectly swapped by $90^\circ$!
    *   **Solid Purple Line ($M_X$ Roll):** Stays completely flat at +800 kN-m. Again, this proves the Port wingtip lever-arm physics. Using lateral cyclic changes lift front/back, which has absolutely zero effect on the massive left/right roll moment!

**Graph 11: Rotor Shaft Power [kW]**
*   **X-axis:** Lateral Cyclic $\theta_{1c}$.
*   **Y-axis:** Required Shaft Power [kW].
*   **The Lines & Physics:** 
    *   **Solid Blue Line (Helicopter):** Forms a steep, straight upward slope from 1150 kW to 1800 kW. As you pull the stick to pitch the aircraft, you are adding extreme pitch to the tail of the rotor disk. This creates a massive lift asymmetry, and that asymmetric drag means maneuvering the aircraft costs extreme engine power.
    *   **Dashed Red Line (Conversion):** Forms a very flat, shallow slope near 200 kW. Because the wings are providing most of the lift in conversion mode, pitching the aircraft nose up/down with the rotors creates significantly less aerodynamic drag than doing it in helicopter mode.

**Graph 12: Unstalled Disk Area [%]**
*   **X-axis:** Lateral Cyclic $\theta_{1c}$.
*   **Y-axis:** Percentage of the rotor disk that is NOT stalled.
*   **The Lines & Physics:** 
    *   **Solid Blue Line (Helicopter):** Notice the massive *increase* as the stick moves from $-6^\circ$ to $0^\circ$! 
        *   At $-6^\circ$ (pulling stick backward), the math adds extreme positive pitch to the *Nose* of the rotor. Because the Nose is taking the full, brutal force of the 30 m/s headwind, pitching it up forces it instantly past the $15.8^\circ$ stall limit, crashing the clean area to 72%.
        *   As you move the stick back to the center ($0^\circ$), you remove that extreme nose pitch. The nose "un-stalls" and the clean area shoots back up to 87%!
        *   When you push the stick forward ($+6^\circ$), it adds pitch to the *Tail*. Because the tail is shielded in the messy wake of the rotor, pitching it up doesn't cause nearly as severe of a stall, so the line stays relatively high at 86%.
    *   **Dashed Red Line (Conversion):** Remains completely flat and stable around 84%. Because the airspeed is high (60 m/s) and the rotor is tilted and unloaded, altering the front/back lift distribution doesn't cause any severe stalling!

