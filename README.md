# AE 667 — Milestone 2: Technical Reference & Implementation Guide

This document is the comprehensive, highly detailed technical manual for our Python code (`milestone_2.ipynb`). It documents every physical assumption, mathematical formula, coordinate transformation, algorithmic loop, and verification metric used to solve Milestone 2. 

You can use this document directly as a reference for writing the final report slides, as it maps exactly to the rubric sections.

---

## Section 1: Coordinate Systems, Transformations, and Assumptions

### 1.1 Reference Frames & Coordinate Systems
To correctly handle the tiltrotor converting from helicopter mode ($\theta_{\text{nac}} = 90^\circ$) to airplane mode ($\theta_{\text{nac}} = 0^\circ$) while the fuselage pitches, we strictly define four reference frames:

1. **Inertial Frame $\mathcal{F}_I$**: North ($X_I$), East ($Y_I$), Down ($Z_I$). Gravity acts along $+Z_I$.
2. **Body Frame $\mathcal{F}_B$**: Origin at the Aircraft Center of Gravity (CG). 
   - $+X_B$: Forward out the nose.
   - $+Y_B$: Out the starboard (right) wing.
   - $+Z_B$: Downward out the belly.
   - The aircraft pitch attitude is $\alpha_B$. Gravity in the body frame is $\mathbf{F}_{\text{grav}, B} = mg [-\sin\alpha_B, 0, \cos\alpha_B]^T$.
3. **Shaft / Nacelle Frame $\mathcal{F}_S$**: Origin at the rotor hub center ($\mathbf{r}_{\text{hub}, L} = [0, -b/2, -0.80]^T$ m).
   - The nacelle tilt angle $\theta_{\text{nac}}$ rotates the shaft around the Body $Y_B$ axis.
   - Rotation Matrix from Shaft to Body:
     $$ \mathbf{R}_{B \leftarrow S} = \begin{bmatrix} \cos\theta_{\text{nac}} & 0 & -\sin\theta_{\text{nac}} \\ 0 & 1 & 0 \\ \sin\theta_{\text{nac}} & 0 & \cos\theta_{\text{nac}} \end{bmatrix} $$
4. **Azimuthal Frame $\psi$**: Tracks the rotating blade in the disk plane.
   - $\psi = 0^\circ$: Blade pointing aft toward the tail.
   - $\psi = 90^\circ$: Advancing blade (moving forward into the freestream).
   - $\psi = 180^\circ$: Blade pointing forward toward the nose.
   - $\psi = 270^\circ$: Retreating blade (moving backward away from the freestream).

### 1.2 Aerodynamic & Modeling Assumptions
1. **Rigid Disk Assumption**: Blade flapping dynamics ($\beta_0, \beta_{1c}, \beta_{1s}$) are assumed small and are not integrated dynamically in this milestone. The rotor tip-path plane is assumed strictly perpendicular to the shaft.
2. **Glauert Skewed-Wake Inflow**: We assume the induced inflow across the disk is nonuniform due to forward flight, modeled linearly as $\lambda_i(r, \psi) = \lambda_{i0} (1 + K_x \frac{r}{R} \cos\psi)$ using Pitt-Peters gradients.
3. **360° Airfoil Polars**: Flow reversals ($U_T < 0$) on the retreating side are handled using the Viterna-Corrigan extrapolation, flipping the chordwise lift direction.
4. **Compressibility**: Prandtl-Glauert compressibility corrections are applied below Mach $0.75$, with wave drag penalties added when $M \ge M_{\text{crit}} = 0.75$.

---

## Section 2: Azimuth-Resolved Edgewise BEMT Solver (`run_edgewise_bemt`)

This is the core algorithm inside Cell 4. It integrates blade element forces over the 2D rotor disk.

### Step 2.1: Discretization
The continuous rotor disk is meshed into $N_r = 30$ radial rings (from root cutout $r_0 = 0.46$ m to tip $R = 4.58$ m) and $N_\psi = 72$ azimuthal sectors ($\Delta\psi = 5^\circ$), creating a $30 \times 72$ 2D meshgrid (`R_grid`, `PSI_grid`).

### Step 2.2: Free-Stream Velocity Projection
The freestream $V_\infty$ is projected onto the tilted rotor disk. The effective shaft angle of attack is $\alpha_{\text{eff}} = \theta_{\text{nac}} - \alpha_B$.
* **Edgewise Velocity (In-Plane)**: $V_{\text{edge}} = V_\infty \sin(\alpha_{\text{eff}})$
* **Axial Velocity (Perpendicular)**: $V_{\text{axial}} = V_\infty \cos(\alpha_{\text{eff}})$
* **Advance Ratios**: $\mu = \frac{V_{\text{edge}}}{\Omega R}$, and $\mu_z = \frac{V_{\text{axial}}}{\Omega R}$.

### Step 2.3: Swashplate Cyclic Kinematics
At every mesh point $(r_i, \psi_j)$, the local blade geometric pitch $\theta$ is defined by the collective $\theta_0$, the linear twist $\theta_{\text{tw}} = -30^\circ$, and the cyclic inputs $\theta_{1c}, \theta_{1s}$:
$$ \theta(r, \psi) = \theta_0 + \theta_{\text{tw}} \left( \frac{r}{R} - 0.75 \right) + \theta_{1c} \cos\psi + \theta_{1s} \sin\psi $$

### Step 2.4: Glauert Fixed-Point Inflow Loop
Because the rotor wake is blown backward at wake skew angle $\chi = \arctan\left(\frac{\mu}{\mu_z + \lambda_{i0}}\right)$, the rear of the disk sees more downwash. 
The code runs a `while` loop (relaxation factor $0.20$, tolerance $10^{-5}$) to solve Glauert's quartic momentum equation:
$$ \lambda_{i0} = \frac{C_T}{2 \sqrt{\mu^2 + (\mu_z + \lambda_{i0})^2}} $$
Once converged, the local inflow at every grid point is computed using the Pitt-Peters longitudinal gradient $K_x$:
$$ K_x = \frac{4}{3} \frac{1 - \cos\chi - 1.8\mu^2}{\sin\chi} \implies U_P(r, \psi) = \Omega R (\mu_z + \lambda_{i0}(1 + K_x \frac{r}{R} \cos\psi)) $$

### Step 2.5: Sectional Loads, Reverse Flow, and Mach Calculations
For every cell $(r_i, \psi_j)$:
1. **Tangential Velocity**: $U_T(r, \psi) = \Omega r + V_{\text{edge}} \sin\psi$. 
   - If $U_T \le 0$, the cell is inside the **Reverse Flow Circle**. The code flags this and reverses the flow direction logic.
2. **Local Mach Number**: $M = \sqrt{U_T^2 + U_P^2} / a_{\text{sound}}$.
3. **Inflow Angle and AoA**: $\phi = \text{atan2}(U_P, U_T)$, and $\alpha = \theta - \phi$.
4. **Airfoil Lookup**: The $C_l(\alpha, M)$ and $C_d(\alpha, M)$ values are queried from the VR-12 database. Prandtl's tip loss factor $F(r) = \frac{2}{\pi}\arccos(e^{-f})$ is applied.
5. **Sectional Forces (N/m)**:
   $$ \frac{dF_z}{dr} = F(r) \frac{1}{2} \rho (U_T^2 + U_P^2) c \cdot (C_l \cos\phi - C_d \sin\phi) $$
   $$ \frac{dF_\psi}{dr} = F(r) \frac{1}{2} \rho (U_T^2 + U_P^2) c \cdot (C_l \sin\phi + C_d \cos\phi) $$

### Step 2.6: Disk Integration to Hub Loads
The code uses 2D numerical integration (`np.trapezoid` over $r$ and $\psi$) to sum the sectional forces into whole-rotor loads at the hub:
* Thrust $T = \frac{N_b}{2\pi} \int \int \frac{dF_z}{dr} \,dr\,d\psi$
* Torque $Q = \frac{N_b}{2\pi} \int \int \frac{dF_\psi}{dr} r \,dr\,d\psi$
* Roll Moment $M_X = -\frac{N_b}{2\pi} \int \int \frac{dF_z}{dr} r \sin\psi \,dr\,d\psi$
Finally, these hub loads $(T, H, Y, M_X, M_Y, Q)$ are mapped through $\mathbf{R}_{B \leftarrow S}$ into the aircraft Body Frame to give $[F_X, F_Y, F_Z]$ and $[M_X, M_Y, M_Z]$.

---

## Section 3: Edgewise-Flight Verification

### 3.1 Recovery of Milestone 1 Limiting Cases
**Hypothesis**: If $V_{\text{edge}} = 0$, the complex 2D code must mathematically collapse to the simple 1D Milestone 1 code.
**Method**: We ran the 2D code in pure Hover ($V_\infty = 0$) and pure Axial Cruise ($\theta_{\text{nac}} = 0^\circ$), keeping cyclic $\theta_{1c} = \theta_{1s} = 0$.
**Result**: The difference between the 1D and 2D integrators is $\Delta T = 0.005\%$ and $\Delta P = 0.00002\%$. The exact overlay of $dT/dr$ and $dP/dr$ curves proves the azimuthal integration has zero spurious drift.

### 3.2 Azimuthal Loading & Periodicity
**Analysis**: In edgewise flight ($V_\infty = 60$ m/s, $\theta_{\text{nac}} = 75^\circ$), dynamic pressure $q \propto (\Omega r + V_\infty \sin\psi)^2$ varies wildly.
* The advancing side ($\psi = 90^\circ$) generates a peak single-blade lift of **$56.6$ kN**.
* The retreating side ($\psi = 270^\circ$) generates a minimum single-blade lift of **$0.6$ kN**.
* Integrating this $1\text{P}$ ($2\pi$-periodic) asymmetric load around the disk reveals why an uncycled rotor generates a massive aerodynamic hub roll moment ($M_X = -127.9 \text{ kN}\cdot\text{m}$). This physically necessitates the use of longitudinal cyclic $\theta_{1s}$ to flatten the lift distribution.

### 3.3 Reverse Flow, Stall boundaries, and Mach Limits
We extract three key physics limits directly from the 2D mesh arrays:
1. **Reverse Flow Circle**: Traced exactly where $U_T(r, \psi) = 0$. Mathematically, this is a circle of diameter $D_{\text{rev}} = \mu R = 1.16$ m on the retreating side, enclosing 4.1% of the disk.
2. **Two-Lobe "$\infty$" Stall Boundary**: We map all cells where $|\alpha| \ge \alpha_{\text{stall}} = 15.8^\circ$. It forms a figure-8 because:
   - *Left Lobe ($\psi \approx 270^\circ$)*: Retreating blade stall caused by massive inflow angles $\phi$ near the reverse-flow region.
   - *Right Lobe ($\psi \approx 90^\circ$)*: Inboard advancing blade stall. Because $\theta_{\text{tw}} = -30^\circ$, the root pitch is $\theta_{\text{root}} = 33.5^\circ$. Local $\Omega r$ is too small to lower $\alpha = 33.5^\circ - \phi$ below stall.
3. **Advancing Tip Mach**: The sum of tip speed and forward speed pushes $M(R, 90^\circ) = 0.844$, exceeding the drag divergence Mach number $M_{dd} = 0.75$, activating wave drag on the advancing tip.

### 3.4 Discretization Sensitivity
We looped the solver over varying mesh sizes $N_r \in [8, 80]$ and $N_\psi \in [8, 144]$ and tracked $T$ and $Q$.
* **Radial ($N_r$)**: Convergence is driven by capturing the steep $dF/dr \to -\infty$ gradient at the tip caused by the Prandtl tip-loss function. Error drops below $0.15\%$ at $N_r \ge 30$.
* **Azimuthal ($N_\psi$)**: Uniform trapezoidal integration of smooth $2\pi$-periodic trigonometric functions yields very rapid spectral convergence. Error drops below $0.02\%$ at $N_\psi \ge 72$ ($\Delta\psi = 5^\circ$).

---

## Section 4: Pilot-Input & Rotor-Tilt Control Sweeps

To avoid recalculating the aerodynamics from scratch for the thousands of iterations needed in Sections 4, 6, 7, and 8, we pre-computed a **6D Aerodynamic Database** (`tiltrotor_rotor_database.csv`, $5,292$ grid points) spanning $(V_\infty, \alpha_{\text{eff}}, \text{RPM}, \theta_0, \theta_{1c}, \theta_{1s})$.

Using `scipy.interpolate.RegularGridInterpolator`, we evaluate control sensitivities at a representative edgewise state ($V = 35$ m/s, $\theta_{\text{nac}} = 75^\circ$):
1. **Collective Sweep ($\theta_0$)**: $F_Z$ (Lift) and $F_X$ (Propulsion) increase linearly, but Shaft Power $P$ increases quadratically (induced power $P_i \propto T^{3/2}$). Stall margin decays linearly to zero.
2. **Longitudinal Cyclic Sweep ($\theta_{1s}$)**: By adding pitch $\Delta\theta = \theta_{1s}\sin\psi$, setting $\theta_{1s} \approx -4.2^\circ$ decreases pitch on the advancing side ($\sin 90^\circ = +1$) and increases it on the retreating side ($\sin 270^\circ = -1$), perfectly trimming the hub roll moment $M_X \to 0$ without significantly altering mean thrust.
3. **Nacelle Sweep ($\theta_{\text{nac}}$)**: Tilting the nacelle from $90^\circ \to 0^\circ$ rotates the thrust vector from $-F_Z \to +F_X$. Because in-plane velocity $V_{\text{edge}} = V_\infty \sin\theta_{\text{nac}}$ approaches zero, the reverse-flow circle shrinks and vanishes, restoring $14^\circ$ of stall margin.

---

## Section 5 & 6: 6-DOF Aircraft Trim Solver

### 6.1 Mathematical Formulation of the Trim Problem
For steady, unaccelerated flight, the 6-Degrees-of-Freedom (6-DOF) sum of forces and moments at the aircraft CG must equal zero:
$$ \sum \mathbf{F}_B = \mathbf{F}_{\text{wing}} + \mathbf{F}_{\text{fuse}} + \mathbf{F}_{\text{emp}} + \mathbf{F}_{\text{grav}, B} + \sum_{i=1}^2 \left[ \mathbf{R}_{B \leftarrow S_i} \mathbf{F}_{S_i} \right] = \mathbf{0} $$
$$ \sum \mathbf{M}_{\text{CG}} = \mathbf{M}_{\text{aero, CG}} + \sum_{i=1}^2 \left[ \mathbf{R}_{B \leftarrow S_i} \mathbf{M}_{S_i} + (\mathbf{r}_{\text{hub},i} - \mathbf{r}_{\text{CG}}) \times \mathbf{F}_{B,i} \right] = \mathbf{0} $$

### 6.2 Implementation of `solve_aircraft_trim_6dof`
* **State Variables (The Pilot Inputs)**: $\mathbf{u} = [\theta_0, \theta_{1s}, \theta_{1c}, \alpha_B, \delta_e, \delta_a/\delta_r]^T$.
* **Algorithm**: We use `scipy.optimize.least_squares` with the Trust Region Reflective (`trf`) algorithm. The optimizer is given strict physical bounds (e.g., $\theta_0 \in [0^\circ, 24^\circ]$, $\delta_e \in [-25^\circ, +25^\circ]$).
* **Execution**: It iteratively perturbs the control vector $\mathbf{u}$, queries the 6D rotor database, computes the wing/fuselage/tail aerodynamic drag and lift using $C_L(\alpha_w)$ and $C_D(\alpha_w)$ functions, and evaluates the residual vector $\mathbf{R}(\mathbf{u})$. It converges when $\|\mathbf{R}\|_2 \le 10^{-4}$.

### 6.3 Physical Trim Trends
The resulting 3x3 trim matrix demonstrates correct load handover:
* **Hover ($V=20$ m/s, $\theta_{\text{nac}}=85^\circ$)**: Wing lift is negligible ($q_\infty$ is tiny). Rotors carry 89% of aircraft weight.
* **Mid-Conversion ($V=55$ m/s, $\theta_{\text{nac}}=45^\circ$)**: Dynamic pressure increases. The wing now supports 64% of the weight, unloading the rotors and causing the required power to drop into the "Tiltrotor Power Bucket".
* **Airplane Cruise ($V=95$ m/s, $\theta_{\text{nac}}=0^\circ$)**: Wing carries 100% of weight. Rotors provide pure axial propulsion.

---

## Sections 7 & 8: Conversion Corridor & Mission Planner v2

### 7. The Conversion Corridor Map
By sweeping the Trim Solver across a grid of $(V_\infty, \theta_{\text{nac}})$ pairs, we map the boundaries of safe flight:
1. **Low-Speed Boundary (Left Side)**: Limited by Wing Stall ($\alpha_w > 15^\circ$) and Rotor Collective Limit ($\theta_0 > 24^\circ$). The nacelles cannot be tilted down until the airspeed is high enough for the wing to generate lift.
2. **High-Speed Boundary (Right Side)**: Limited by Engine Power ($P_{\text{req}} > 2800$ kW) and Advancing Tip Mach ($M > 0.88$). The nacelles must be tilted down as airspeed increases to reduce the in-plane edgewise velocity.

### 8. Mission Planner v2 Time-Integration
We simulate a full Outbound (`Hover → Airplane`) and Inbound (`Airplane → Hover`) mission segment.
* **Euler Integration**: At each discrete time step $\Delta t = 2.0$ s, the solver trims the aircraft, computes current power $P$, and updates the state variables:
  $$ \dot{V} = \frac{T \cos\theta_{\text{nac}} - D}{m}, \qquad \Delta h = V \sin\gamma \Delta t $$
  $$ m_{\text{fuel}}(t + \Delta t) = m_{\text{fuel}}(t) - \text{SFC} \cdot P_{\text{req}} \cdot \Delta t $$
* The results prove the transition schedule perfectly navigates the conversion corridor without violating any aerodynamic or control limits.
