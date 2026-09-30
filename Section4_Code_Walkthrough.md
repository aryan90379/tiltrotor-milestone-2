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
    *   **Solid Blue Line (Helicopter):** Stays high (90%) until about $\theta_0 = 10^\circ$, then violently crashes downward. The collective pushes the entire blade too high, exceeding the static stall limit ($15.8^\circ$) across the vast majority of the rotor disk. 
    *   **Dashed Red Line (Conversion):** Starts deeply stalled (35%) and actually *improves* as you pull collective! Why? At low collective ($0^\circ$), the fast $60$ m/s wind is hitting the *top* of the tilted blades (a massive negative angle of attack), causing a negative stall. As you pull collective up, you increase the blade pitch into the clean, positive aerodynamic region.

### Row 2: The 4.2 Longitudinal Cyclic Sweep ($\theta_{1s}$)
**The Control:** Longitudinal cyclic applies a sine-wave variation to the blade pitch as it spins: $\Delta\theta = \theta_{1s} \sin\psi$. Because $\sin(90^\circ) = 1$ and $\sin(270^\circ) = -1$, this control strictly changes the pitch on the left and right sides of the helicopter.

**Graph 5: Body Forces [kN]**
*   **The Lines & Physics:** Notice that the Thrust lines ($F_Z$, Red) stay perfectly flat horizontally. This proves **Thrust Decoupling**. Because we are mathematically adding pitch to the right side and subtracting the exact same amount of pitch from the left side, the total average lift of the helicopter remains completely unchanged. You can move the joystick without suddenly gaining or losing altitude!

**Graph 6: Body Moments about CG [kN-m] (The Most Important Graph!)**
*   **The Lines & Physics:**
    *   **Solid Purple Line ($M_X$ Roll):** This line crosses exactly through zero at $\theta_{1s} \approx -4.2^\circ$. In forward flight, the advancing right side naturally generates way too much lift (Dissymmetry of Lift). By holding the stick at $-4.2^\circ$, we mathematically subtract pitch from the advancing right side and add it to the retreating left side. This perfectly balances the lift left-to-right, canceling the Roll Moment ($M_X = 0$) so the helicopter doesn't flip over!
    *   **The "Rigid Rotor" Control Anomaly:** In a real helicopter, pushing the stick forward tilts the disk forward, pitching the nose down. But look at the Orange Line ($M_Y$ Pitch); it stays totally flat! Instead, pushing the stick forward causes pure Roll (Purple line). Why? Because our code uses a **Rigid Disk** ($\beta = 0$, no flapping hinges). Without hinges, there is no $90^\circ$ gyroscopic phase delay. The physical force happens exactly where the pitch is applied.

**Graph 7: Rotor Shaft Power [kW]**
*   **The Lines & Physics:** The power curve forms a shallow "U" shape. The lowest point (the bottom of the "U") is exactly at $\theta_{1s} = -4.2^\circ$. Why? Because at $-4.2^\circ$, the lift is perfectly balanced left-to-right. If you move the stick away from that balanced point, you force one side of the rotor to lift way harder than the other, which creates massive asymmetric drag. The engine has to burn extra fuel to fight that drag!

**Graph 8: Unstalled Disk Area [%]**
*   **The Lines & Physics:** The blue line stays relatively flat, but dips on the extreme left and right edges. If you push the stick too far, you force one side of the disk to pitch up so high that it violently stalls.

### Row 3: The 4.3 Lateral Cyclic Sweep ($\theta_{1c}$)
**The Control:** Lateral cyclic applies a cosine-wave variation to the blade pitch: $\Delta\theta = \theta_{1c} \cos\psi$. Because $\cos(180^\circ) = -1$ (Nose) and $\cos(0^\circ) = 1$ (Tail), this control strictly alters the lift at the front and rear of the rotor disk.

**Graph 9: Body Forces [kN]**
*   **The Lines & Physics:** Just like in Graph 5, the Thrust lines ($F_Z$) remain perfectly flat. We are adding lift to the nose and subtracting from the tail, so total average lift remains exactly the same.

**Graph 10: Body Moments about CG [kN-m]**
*   **The Lines & Physics:** 
    *   **Solid Orange Line ($M_Y$ Pitch):** Crosses zero with a steep slope. By altering the front/rear lift distribution, we can actively pitch the nose of the aircraft up or down. 
    *   **The Rigid Rotor Swap:** Again, in a real helicopter, Lateral Cyclic is used to Roll left and right. But because of our rigid disk assumption, Lateral Cyclic gives us pure Pitch control! The controls are perfectly swapped by $90^\circ$!

**Graph 11: Rotor Shaft Power [kW]**
*   **The Lines & Physics:** Forms a "V" shape with the lowest power right at $0^\circ$. If the pilot pushes the stick left or right to pitch the nose, they create a massive lift asymmetry between the front and back of the rotor. This asymmetric drag means maneuvering the aircraft always costs extra engine power.

**Graph 12: Unstalled Disk Area [%]**
*   **The Lines & Physics:** Drops off very sharply at the edges. Pushing the stick too far aggressively stalls either the front or the rear of the rotor disk.

9481394813

