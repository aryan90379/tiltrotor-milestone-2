# Section 4: The 12-Graph Code Walkthrough

This document breaks down exactly how we get the data and plot **every single one of the 12 graphs** shown in Section 4. 

We generate a massive 3x4 grid in `matplotlib` (`axes[row, col]`). For every single point on these graphs, we are querying our 6-Dimensional SciPy Interpolator.

---

## Row 1: The 4.1 Collective Sweep ($\theta_0$)
**The Setup:** We sweep the collective pitch from $0^\circ$ to $24^\circ$. The cyclic controls ($\theta_{1s}$, $\theta_{1c}$) are locked at $0^\circ$. 
*   **The Code:** `theta0_array = np.linspace(0, 24, 25)`
*   **The Query:** `answers = interpolator([V_inf, alpha, RPM, theta0_array, 0, 0])`

### Graph 1 (Row 1, Col 1): Body Forces [kN]
*   **Data Extracted:** We pull the first 3 columns from the interpolator: $F_X, F_Y, F_Z$.
*   **What we plot:** 
    *   `ax.plot(theta0_array, F_X)` (Blue line, Forward)
    *   `ax.plot(theta0_array, F_Y)` (Green line, Side)
    *   `ax.plot(theta0_array, F_Z)` (Red line, Vertical Lift)
*   **The Physics:** The red line plummets to $-125$ kN. Because the Z-axis points into the ground, a negative number means massive upward lift. Pulling collective drastically increases lift.

### Graph 2 (Row 1, Col 2): Body Moments about CG [kN-m]
*   **Data Extracted:** We pull columns 4, 5, 6 from the interpolator: $M_X, M_Y, M_Z$.
*   **What we plot:** 
    *   `ax.plot(theta0_array, M_X)` (Purple line, Roll)
    *   `ax.plot(theta0_array, M_Y)` (Orange line, Pitch)
    *   `ax.plot(theta0_array, M_Z)` (Cyan line, Yaw)
*   **The Physics:** The purple roll moment skyrockets past $1000$ kN-m. Because we locked the cyclic controls to zero, increasing collective makes the advancing blade generate exponentially more lift than the retreating blade, creating a violent twisting force.

### Graph 3 (Row 1, Col 3): Rotor Shaft Power [kW]
*   **Data Extracted:** We pull column 7 from the interpolator: $P_{\text{req}}$.
*   **What we plot:** 
    *   `ax.plot(theta0_array, Power)`
*   **The Physics:** Shows the rocket-ship curve of Induced Power. It takes almost 4000 kW of power to spin the blades at $24^\circ$ collective. The red dashed line (airplane mode) requires way less power because the wings are helping to lift the aircraft.

### Graph 4 (Row 1, Col 4): Unstalled Disk Area [%]
*   **Data Extracted:** We pull column 8 from the interpolator (Stall Fraction). We calculate `100 * (1 - Stall_Fraction)`.
*   **What we plot:** 
    *   `ax.plot(theta0_array, Unstalled_Area)`
    *   `ax.axhline(85, color='black', linestyle=':')` (The Danger Line)
*   **The Physics:** If you pull the collective too hard (past $15^\circ$), the blades stall. The blue line crashes downward, meaning over 65% of the rotor disk is stalled and producing zero lift.

---

## Row 2: The 4.2 Longitudinal Cyclic Sweep ($\theta_{1s}$)
**The Setup:** We lock the collective to a constant hovering value (e.g., $10^\circ$). We sweep the longitudinal flight stick (forward/backward) from $-6^\circ$ to $+6^\circ$.
*   **The Code:** `theta1s_array = np.linspace(-6, 6, 25)`
*   **The Query:** `answers = interpolator([V_inf, alpha, RPM, 10, 0, theta1s_array])`

### Graph 5 (Row 2, Col 1): Body Forces [kN]
*   **Data Extracted:** $F_X, F_Y, F_Z$.
*   **What we plot:** The exact same red/blue/green lines as Graph 1.
*   **The Physics:** Pushing the stick doesn't drastically change the total lift (red line stays mostly flat), but it does slightly change the forward propulsion ($F_X$) as the thrust vector tilts forward.

### Graph 6 (Row 2, Col 2): Body Moments about CG [kN-m]
*   **Data Extracted:** $M_X, M_Y, M_Z$.
*   **What we plot:** The exact same purple/orange/cyan lines as Graph 2.
*   **The Physics:** *This is the most important graph in Section 4.* Look at the purple line (Roll Moment). At $0^\circ$ stick, the roll is massive ($800$ kN-m). By pushing the stick back to $-4^\circ$ (left side of the graph), the purple line crosses zero. This proves cyclic pitch perfectly cancels out rolling moments!

### Graph 7 (Row 2, Col 3): Rotor Shaft Power [kW]
*   **Data Extracted:** $P_{\text{req}}$.
*   **The Physics:** Power stays relatively flat. Changing the cyclic stick just shifts lift from one side of the disk to the other; it doesn't vastly increase the total power needed from the engines.

### Graph 8 (Row 2, Col 4): Unstalled Disk Area [%]
*   **Data Extracted:** `100 * (1 - Stall_Fraction)`.
*   **The Physics:** Pushing the stick too far forward (past $+2^\circ$) causes the retreating blade to stall aggressively, crashing the unstalled area curve below the 85% safe line.

---

## Row 3: The 4.3 Lateral Cyclic Sweep ($\theta_{1c}$)
**The Setup:** We lock collective to $10^\circ$, lock longitudinal cyclic to $0^\circ$, and sweep the lateral flight stick (left/right) from $-6^\circ$ to $+6^\circ$.
*   **The Code:** `theta1c_array = np.linspace(-6, 6, 25)`
*   **The Query:** `answers = interpolator([V_inf, alpha, RPM, 10, theta1c_array, 0])`

### Graph 9 (Row 3, Col 1): Body Forces [kN]
*   **Data Extracted:** $F_X, F_Y, F_Z$.
*   **The Physics:** Very little changes here. Lateral cyclic mainly affects twisting moments, not raw thrust.

### Graph 10 (Row 3, Col 2): Body Moments about CG [kN-m]
*   **Data Extracted:** $M_X, M_Y, M_Z$.
*   **The Physics:** Look at the orange line ($M_Y$, Pitch Moment). Sweeping the lateral cyclic changes the pitch moment, allowing the pilot to tilt the nose of the helicopter up or down.

### Graph 11 (Row 3, Col 3): Rotor Shaft Power [kW]
*   **Data Extracted:** $P_{\text{req}}$.
*   **The Physics:** Similar to longitudinal cyclic, shifting lift front-to-back using lateral cyclic causes a slight increase in power required, but nothing as extreme as pulling the main collective.

### Graph 12 (Row 3, Col 4): Unstalled Disk Area [%]
*   **Data Extracted:** `100 * (1 - Stall_Fraction)`.
*   **The Physics:** Moving the stick too far to the left (negative $\theta_{1c}$) causes a massive stall condition on the rotor (blue line crashes to 72%), proving the flight controls have strict boundaries the pilot cannot exceed.
