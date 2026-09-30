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
    *   **Dashed Red Line (Conversion):** Stays flat near zero, then rises. Why is it zero? Because the aircraft is flying fast ($60$ m/s) with the rotors facing forward. At low collective pitches, the wind blows *through* the rotors like a windmill, meaning they are practically autorotating for free! It only costs engine power once you pull collective past $15^\circ$.

**Graph 4: Unstalled Disk Area [%]**
*   **X-axis:** Collective Pitch $\theta_0$.
*   **Y-axis:** Percentage of the rotor disk that is NOT stalled.
*   **The Lines & Physics:**
    *   **Solid Blue Line (Helicopter):** Stays high (90%) until about $\theta_0 = 10^\circ$, then violently crashes downward. The collective pushes the entire blade too high, exceeding the static stall limit ($15.8^\circ$) across the vast majority of the rotor disk. 
    *   **Dashed Red Line (Conversion):** Starts deeply stalled (35%) and actually *improves* as you pull collective! Why? At low collective ($0^\circ$), the fast $60$ m/s wind is hitting the *top* of the tilted blades (a massive negative angle of attack), causing a negative stall. As you pull collective up, you increase the blade pitch into the clean, positive aerodynamic region.

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

