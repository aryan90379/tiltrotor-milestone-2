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

The goal of Section 4 is to prove that our aerodynamic code responds correctly to pilot inputs before we hand the model over to the 6-DOF Trim Solver. We sweep each of the three main rotor controls (Collective, Longitudinal Cyclic, and Lateral Cyclic) from $-10^\circ$ to $+10^\circ$ while freezing the others.

### Row 1: The 4.1 Collective Sweep ($\t\theta_0$)
**The Control:** The collective pitch ($	\theta_0$) physically rotates all 3 blades up or down by the exact same amount simultaneously. The pitch equation shifts uniformly: $\theta(r, \psi) = \t\theta_0 + \dots$

*   **Thrust Curve (Graph 1):** Thrust increases almost perfectly linearly with Collective pitch. This makes physical sense because increasing $\t\theta_0$ linearly increases the Angle of Attack ($\alpha$) everywhere on the disk, directly scaling the Lift coefficient ($C_l = a_0 \alpha$).
*   **Power Curve (Graph 3):** Unlike Thrust, the blue Power curve takes a severe, non-linear parabolic shape. 
    *   *The Physics:* According to Momentum Theory, the Induced Power ($P_i$) required to generate Thrust ($T$) is defined as $P_i = \frac{T \sqrt{T}}{\sqrt{2 \rho A}} \propto T^{3/2}$. As the collective pushes the thrust up linearly, the aerodynamic drag and induced power explode exponentially.
*   **Stall Area (Graph 4):** As $\t\theta_0 > 15^\circ$, the local Angle of Attack ($\alpha = \theta - \phi$) exceeds the VR-12 static stall limit ($15.8^\circ$) across the vast majority of the rotor disk. The green line crashes downward, indicating that less than 85% of the disk is generating clean lift.

### Row 2: The 4.2 Longitudinal Cyclic Sweep ($\theta_{1s}$)
**The Control:** Longitudinal cyclic applies a sine-wave variation to the blade pitch as it spins: $\Delta\theta = \theta_{1s} \sin\psi$. Because $\sin(90^\circ) = 1$ and $\sin(270^\circ) = -1$, this control strictly changes the pitch on the left and right sides of the helicopter.

*   **Thrust decoupling (Graph 5):** Notice that the Thrust line stays perfectly flat. Because we add pitch to the right side and subtract the exact same amount from the left side, the total average lift remains completely unchanged.
*   **Roll Moment Cancellation (Graph 6):** This is the most important graph in the row! In forward flight, the advancing right side naturally lifts $90\times$ harder than the retreating left side, creating a massive $-127.9$ kN-m Roll Moment ($M_X$) that tries to flip the aircraft. 
    *   *The Physics:* By setting $\theta_{1s} \approx -4.2^\circ$, we mathematically subtract pitch from the advancing right side and add it to the retreating left side. This perfectly balances the lift, bringing the purple $M_X$ line precisely to zero.
*   **The "Rigid Rotor" Control Coupling Anomaly:** In a real helicopter with hinged blades, pushing the stick forward (longitudinal cyclic) tilts the disk forward, causing a *Pitch* moment. But look at Graph 6: our longitudinal cyclic causes pure *Roll*! Why? Because our mathematical model (per Section 1.2 assumptions) uses a **Rigid Disk** ($\beta = 0$). Without flapping hinges, there is no $90^\circ$ gyroscopic phase lag. The force happens exactly where the pitch is applied.

### Row 3: The 4.3 Lateral Cyclic Sweep ($\theta_{1c}$)
**The Control:** Lateral cyclic applies a cosine-wave variation to the blade pitch: $\Delta\theta = \theta_{1c} \cos\psi$. Because $\cos(180^\circ) = -1$ (Nose) and $\cos(0^\circ) = 1$ (Tail), this control strictly alters the lift at the front and rear of the rotor disk.

*   **Pitch Moment Control (Graph 10):** By altering the front/rear lift distribution, the mathematical integral for the Pitch Moment ($M_Y$) shifts linearly. 
    *   *The Physics:* The orange line ($M_Y$) crosses zero with a steep slope. This proves that the Trim Solver can use $\theta_{1c}$ to actively pitch the nose of the aircraft up or down without significantly altering the total thrust or shaft power.
*   **The Power Cost (Graph 11):** Notice that applying cyclic (either forward or backward) always increases the Required Power ($P$). Any asymmetry in the rotor disk increases the overall aerodynamic drag, meaning it costs extra engine power just to maneuver!

9145991459

