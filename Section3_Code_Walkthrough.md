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
The Mach number is evaluated as $M = \frac{\sqrt{U_T^2 + U_P^2}}{a}$.

**5. The Airfoil Physics Engine:**
To convert the local environment ($\alpha$, $M$) into lift and drag ($C_l, C_d$), the `AirfoilDatabase.evaluate` function dynamically switches between three aerodynamic models:
*   **Clean Flow (XFOIL Tabular Lookup):** For normal flight angles ($-15^\circ \le \alpha \le +20^\circ$), it uses a 2D Bivariate Spline to instantly look up the exact Lift and Drag from pre-computed **Boeing-Vertol VR-12** aerodynamic tables based on the local Reynolds Number.
*   **Compressibility (Prandtl-Glauert):** To simulate massive wave drag, the code applies the Prandtl-Glauert scaling law ($\beta = \sqrt{1 - M^2}$) to the tabular data, physically capturing the drag spike near the speed of sound.
*   **Reverse Flow & Deep Stall (Viterna-Corrigan $360^\circ$ Extrapolation):** 
    *What is it and why did we use it?* Wind tunnels and XFOIL simulations cannot easily calculate aerodynamics when a wing is flying perfectly backwards or sideways at 60 m/s. If you ask the tabular database for Lift at $\alpha = 180^\circ$ (Reverse Flow), the math simply crashes. To fix this, the aerospace industry (and our code) uses the **Viterna-Corrigan Method**—an algorithm originally developed by NASA to model wind turbines operating in extreme deep stall. 
    If the blade enters Reverse Flow or Deep Stall ($\alpha > 20^\circ$ or $\alpha < -15^\circ$), the code instantly throws away the XFOIL tables and mathematically models the blade as a blunt **flat plate** using these $360^\circ$ sine-wave approximations:
    $$ C_{L,\text{stall}} = (C_{L,\text{max}} \times 0.95) \sin(2\alpha) $$
    $$ C_{D,\text{stall}} = 0.015 + 1.25 \sin^2(\alpha) $$
    This flawlessly blends the standard tabular aerodynamics into a robust $360^\circ$ analytical model. Because $\sin(2\times180^\circ) = 0$, the lift correctly zeroes out when flying perfectly backward, saving the simulation from crashing!

**6. 2D Integration & Tip-Loss:**
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
*   **The Plot:** A top-down heatmap of the rotor disk. The center is the hub ($r=0$), and the outer edge is the blade tip ($r=4.58$ m). 
    *   **The Thick White Dashed Circle (Left Side):** If you look at the exact center of the crosshairs, you'll see a thick white dashed circle that touches the center and bulges out to the left side (the Retreating side). This is the **Reverse Flow Boundary** ($U_T = 0$). Inside this white dashed circle, the helicopter is flying forward so fast that the 30 m/s wind is actually blowing *backwards* over the retreating blade! This is exactly why the entire area inside that dashed circle is dark blue/purple—the lift has violently crashed and actually gone negative!
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



