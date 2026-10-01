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

---

## 4.6 Consolidated Aerodynamic Observations (Summary)

*This table synthesizes the deep physical and mathematical phenomena observed in Sections 3 and 4, proving the solver correctly captures advanced tiltrotor aerodynamics.*

| Observation | Physical Cause & Aerodynamic Theory | Code Implementation & Evidence | Design Implication for Tiltrotor |
| :--- | :--- | :--- | :--- |
| **Dissymmetry of Lift** *(Advancing/Retreating Asymmetry)* | Forward speed ($V_\infty$) adds to advancing blade velocity and subtracts from retreating blade ($U_T = \Omega r + V_\infty \sin\psi$). | **Sec 3.2 Contour:** Advancing sector generates **56.6 kN** peak lift, retreating sector crashes to **0.6 kN**. Generates massive **-127.9 kN-m** Roll Moment ($M_X$). | Requires longitudinal cyclic pitch ($\theta_{1s}$) to feather the blades, balancing the lift across the disk to prevent the aircraft from rolling over. |
| **Control Coupling** *(Rigid Disk Aerodynamics)* | Because our BEMT assumes a perfectly rigid rotor disk ($\beta=0$), the standard $90^\circ$ gyroscopic flapping lag does not apply. The phase lag is purely aerodynamic. | **Sec 4.2 & 4.3 Sweeps:** Prove that inputs are swapped for a rigid rotor. Longitudinal cyclic ($\theta_{1s}$) causes pure roll, and lateral cyclic ($\theta_{1c}$) causes pure pitch. | The 6-DOF Trim Solver must mathematically couple all inputs using a matrix optimizer; the pilot cannot move one stick independently without affecting others. |
| **Reverse Flow** *(Air Hitting Trailing Edge)* | At high speeds (60 m/s), wind passes through the rotor faster than the retreating blade spins backward, hitting the trailing edge first ($\alpha = 180^\circ$). | **Sec 3.3(a):** To prevent XFOIL tables from crashing, the physics engine actively switches to the **Viterna-Corrigan 360° flat-plate extrapolation** inside the boundary circle. | The retreating blade produces negative lift and heavy drag. This severely limits the maximum forward airspeed achievable in helicopter mode. |
| **Stall Onset** *(Figure-8 Boundary)* | **Dual-Mechanism:** Retreating side stalls due to low airspeed/high downwash. Advancing root stalls because the extreme **-30° built-in tiltrotor twist** catches too much air. | **Sec 3.3(b) & 4.1:** Shows the signature "$\infty$" shape where $|\alpha| \ge 15.8^\circ$ (the VR-12 stall limit). Sweeps show unstalled area crashing at high collective. | Sets a hard collective limit ($\theta_0 < 15^\circ$) before catastrophic stall occurs, strictly limiting the maximum payload the aircraft can hover with. |
| **Tip-Mach Limitations** *(Wave Drag)* | Advancing tip velocity approaches the speed of sound ($M_{tip} = 0.84$) during fast forward flight. | **Sec 3.3(c):** Mach contour hits the **$M > 0.75$ Drag Divergence ($M_{dd}$)** limit. Code applies the **Prandtl-Glauert** ($\beta = \sqrt{1-M^2}$) scaling to calculate wave drag. | Causes severe wave drag. Requires tilting the nacelles forward and significantly **slowing down the rotor RPM** during conversion to keep tips subsonic. |
| **The Power Bucket** *(Induced Power Trends)* | Induced power scales exponentially with required thrust ($P \propto T^{1.5}$). Hovering requires massive engine output. | **Sec 4.1:** Solid Blue hover power curve rockets upward, while the Red Dashed line (Conversion flight) requires much less power. | The aircraft must transition to airplane mode quickly to shift lift generation to the wings. Unloading the rotors is required to stay under the **2800 kW** engine limit. |
