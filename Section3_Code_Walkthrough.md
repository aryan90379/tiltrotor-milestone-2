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
**The Goal:** Visualize the extreme aerodynamic asymmetry that occurs when the helicopter flies forward at 60 m/s ($V_{\text{edge}}$), creating the "Dissymmetry of Lift".

### Graph 3.2(a): Normal Sectional Load Contour $dF_z/dr$ [N/m]
*   **The Plot:** A top-down heatmap of the rotor disk. The center is the hub ($r=0$), the outer edge is the blade tip ($r=4.58$ m).
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
    *   Because the right side lifts 90x harder than the left side, it physically generates the massive $-127.9$ kN-m Roll Moment ($M_X$) that tries to violently flip the aircraft over. This proves why cyclic pitch is strictly required for forward flight!

## 3.3 Reverse Flow, Stall, and Mach Limits
### Graph 3.3(1): The Reverse Flow Boundary
*   **The Math:** Reverse flow occurs strictly when the tangential velocity is negative ($U_T \le 0$). Solving $U_T(r, \psi) = \Omega r + V_{\text{edge}}\sin\psi = 0$ yields the geometric boundary of a circle on the retreating side:
$$ r(\psi) = -\mu R \sin\psi $$
*   **The Physics:** Inside this circle, air strikes the trailing edge. The solver applies the Viterna-Corrigan $360^\circ$ extrapolation to flip the $C_l$ and $C_d$ signs.

### Graph 3.3(2): The Stall Boundary (The Figure-8)
*   **The Math:** Shaded where $|\alpha| = |\theta - \phi| \ge 15.8^\circ$.
*   **The Physics:** The two lobes occur because $\alpha$ blows up for different reasons. Left Lobe (Retreating): $U_T \to 0$, causing $\phi = \arctan(U_P/U_T) \to 90^\circ$. Right Lobe (Advancing Root): Large built-in twist ($\theta_{\text{root}} = 33.5^\circ$) coupled with low local $\Omega r$ causes $\alpha$ to exceed stall before $\phi$ can reduce it.

### Graph 3.3(3): Advancing Tip Mach Number
*   **The Math:** $M = \frac{\sqrt{U_T^2 + U_P^2}}{a_{sound}}$. Contours highlight the $M = 0.75$ Drag Divergence ($M_{dd}$) boundary and the peak $M = 0.84$ location at $(r=R, \psi=90^\circ)$.

---

## 3.4 Discretization Sensitivity
*   **Radial $N_r$:** Convergence is slow and asymptotic (requires $N_r \ge 30$). The strict requirement is driven by the spatial resolution needed to evaluate the steep gradient of $dF_z/dr \to 0$ near $r=R$ caused by the $\arccos$ in the Prandtl tip-loss function.
*   **Azimuthal $N_\psi$:** Convergence is practically instantaneous ($N_\psi \ge 12$). The aerodynamic loading over the azimuth is dominated by $1\text{P}$ and $2\text{P}$ trigonometric harmonics ($\sin\psi, \cos\psi$). The Periodic Trapezoidal Rule used in the solver exhibits exponential (spectral) convergence for perfectly periodic functions.

8485284852

8485284852

8904089040

8904089040

