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

### Row 1: The 4.1 Collective Sweep ($\theta_0$)
**The Control:** The collective pitch ($\theta_0$) physically rotates all 3 blades up or down by the exact same amount simultaneously. The pitch equation shifts uniformly:
$$ \theta(r, \psi) = \theta_0 + \theta_{\text{tw}}\left(\frac{r}{R} - 0.75\right) + \theta_{1c} \cos\psi + \theta_{1s} \sin\psi $$

This row contains 4 specific graphs detailing the aircraft's response to Collective input:

**Graph 1: Body Forces [kN]**
*   **X-axis:** Collective Pitch $\theta_0$ ($0^\circ \to 25^\circ$).
*   **Y-axis:** Force [kN] in the Body Frame.
*   **The Lines & Physics:** 
    *   **Solid Red Line ($F_Z$, Vertical):** Plummets downward linearly. In our coordinate system, $Z$ points DOWN. So a highly negative $F_Z$ means the rotor is generating massive upward Lift! It is perfectly linear because increasing collective linearly increases Angle of Attack ($\alpha$), directly scaling $C_l = a_0 \alpha$.
    *   **Blue/Green Lines ($F_X, F_Y$):** Stay near zero, as collective does not generate significant side/forward forces in hover.

**Graph 2: Body Moments about CG [kN-m]**
*   **X-axis:** Collective Pitch $\theta_0$.
*   **Y-axis:** Moment [kN-m] in the Body Frame.
*   **The Lines & Physics:**
    *   **Solid Purple Line ($M_X$, Roll):** Explodes massively into the positive. This beautifully illustrates the **Dissymmetry of Lift**. When you increase collective pitch across the whole disk, the advancing side (right) grabs that extra pitch and multiplies it by its massive $V_{\infty}$ velocity, creating exponentially more lift than the retreating side. This tries to violently roll the helicopter!
    *   **Dashed Green Line ($M_Z$, Yaw):** Increases slightly. As the blades grab more air, they create more drag. The engine has to twist harder (Torque) to keep them spinning, which tries to yaw the aircraft body in the opposite direction.

**Graph 3: Rotor Shaft Power [kW]**
*   **X-axis:** Collective Pitch $\theta_0$.
*   **Y-axis:** Required Shaft Power [kW].
*   **The Lines & Physics:**
    *   **Solid Blue Line (Helicopter, V=30):** Takes a severe, non-linear parabolic shape. According to Momentum Theory, Induced Power relates to Thrust by $P_i \propto T^{3/2}$. As the collective (Graph 1) pushes thrust up linearly, the aerodynamic drag and induced power explode exponentially.

**Graph 4: Unstalled Disk Area [%]**
*   **X-axis:** Collective Pitch $\theta_0$.
*   **Y-axis:** Percentage of the rotor disk that is NOT stalled.
*   **The Lines & Physics:**
    *   **Solid Blue Line:** Stays high (90%) until about $\theta_0 = 10^\circ$, then violently crashes downward. Because the collective increases the pitch of the *entire* blade, it quickly forces the local Angle of Attack ($\alpha = \theta - \phi$) to exceed the VR-12 static stall limit ($15.8^\circ$) across the vast majority of the rotor disk. The 85% dotted safety limit proves that you cannot pull more than $14^\circ$ of collective without stalling the helicopter and falling out of the sky!

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

