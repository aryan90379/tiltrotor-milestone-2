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

## Part 2: The Parametric Sweeps (Rows 1 to 3)

### Row 1: The 4.1 Collective Sweep ($\theta_0$)
**The Control:** The collective pitch shifts the entire blade pitch up equally: $\theta(r, \psi) = \theta_0 + \dots$
*   **Power Curve (Graph 3):** The blue curve takes a severe non-linear shape. According to Momentum Theory, Induced Power $P_i$ relates to Thrust $T$ by $P_i = \frac{T \sqrt{T}}{\sqrt{2\rho A}} \propto T^{3/2}$. As $\theta_0$ increases thrust linearly, Power explodes exponentially.
*   **Stall Area (Graph 4):** As $\theta_0 > 15^\circ$, the local Angle of Attack $\alpha = \theta - \phi$ exceeds the VR-12 stall limit ($15.8^\circ$) across the majority of the disk, causing the unstalled area to crash below the $85\%$ safety limit.

### Row 2: The 4.2 Longitudinal Cyclic Sweep ($\theta_{1s}$)
**The Control:** Longitudinal cyclic applies a $\sin\psi$ variation to the blade pitch: $\Delta\theta = \theta_{1s}\sin\psi$.
*   **Roll Moment Cancellation (Graph 6):** Because $\sin(90^\circ) = 1$ and $\sin(270^\circ) = -1$, setting $\theta_{1s} \approx -4.2^\circ$ mathematically subtracts pitch from the advancing right side and adds it to the retreating left side. 
*   **The Physics:** By doing this, the integral equation for $M_X$ perfectly balances out, bringing the purple $M_X$ line precisely to zero. This demonstrates control decoupling in a rigid rotor system.

### Row 3: The 4.3 Lateral Cyclic Sweep ($\theta_{1c}$)
**The Control:** Lateral cyclic applies a $\cos\psi$ variation to the blade pitch: $\Delta\theta = \theta_{1c}\cos\psi$.
*   **Pitch Moment Control (Graph 10):** Because $\cos(180^\circ) = -1$ and $\cos(0^\circ) = 1$, this control alters the lift at the front and rear of the rotor disk. 
*   **The Physics:** By altering the front/rear lift distribution, the integral equation for $M_Y$ shifts linearly. The orange line ($M_Y$) crosses zero and allows the Trim Solver to actively pitch the nose of the aircraft up or down without significantly altering total thrust or shaft power.
