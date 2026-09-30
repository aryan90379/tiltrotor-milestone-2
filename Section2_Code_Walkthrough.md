# Section 2: Edgewise-Flight Performance Estimator (Algorithm Architecture)

This document breaks down the core architecture of our Aeromechanics Engine (`run_edgewise_bemt()`). 

The code strictly follows the 5-step pipeline outlined in **Section 2.1** to solve the complex physics of a tiltrotor in forward flight.

---

## STEP 1: Inputs & Mesh
**Goal:** Define the flight state and build the 2D integration grid over the rotor disk.

*   **Flight Inputs:** The function receives the helicopter's Airspeed ($V_{\infty}$), Altitude (density $\rho$), Body Angle of Attack ($\alpha_B$), Nacelle Tilt ($\theta_{\text{nac}}$), and Rotor RPM ($\Omega$).
*   **Controls:** The pilot inputs the collective ($\theta_0$) and cyclic pitches ($\theta_{1c}, \theta_{1s}$).
*   **Frame Projection:** The code mathematically transforms the incoming wind from the Aircraft Body frame into the Rotor Shaft frame using $\alpha_{\text{eff}} = \theta_{\text{nac}} - \alpha_B$.
    *   $V_{\text{axial}} = V_{\infty} \cos(\alpha_{\text{eff}})$ (Flow passing down through the disk)
    *   $V_{\text{edge}} = V_{\infty} \sin(\alpha_{\text{eff}})$ (Crosswind hitting the disk sideways)
*   **2D Polar Mesh:** We slice the rotor disk into a highly optimized grid of $N_r = 30$ radial rings and $N_\psi = 72$ azimuthal pizza slices ($\Delta\psi = 5^\circ$).

---

## STEP 2: Inflow Loop
**Goal:** Calculate how the air is sucked through the rotor disk (the induced downwash, $\lambda_i$).

*   **Inflow Model:** In forward flight, the air doesn't flow straight down; it gets swept backward. We use the **Glauert Momentum Theory** combined with the **Pitt-Peters Skew Model** to calculate this skewed wake.
*   **Fixed-Point Iteration:** Because the inflow ($\lambda_i$) depends on Thrust ($C_T$), and Thrust depends on inflow, it creates a circular dependency. We solve this using a `while` loop (fixed-point iteration) with an under-relaxation factor of $0.25$ to prevent the math from exploding. It loops until the error drops below $\Delta\lambda_i < 10^{-5}$.
*   **Tip-Loss & Balance:** We apply the **Prandtl Tip-Loss function $F(r)$** to account for air escaping around the tips of the blades, reducing the effective lifting area.
*   **Inflow Gradient:** Once the average inflow ($\lambda_{i0}$) converges, we apply the longitudinal gradient ($K_x$). This physically skews the downwash, making it stronger at the rear of the disk (tail) and weaker at the front (nose).

---

## STEP 3: Blade Kinematics
**Goal:** Calculate exactly how the wind hits every single tiny piece of the blade.

*   **Pitch Law (1st-Harmonic):** For every cell $(r, \psi)$, the local physical blade pitch is calculated:
    $$ \theta(r, \psi) = \theta_0 + \theta_{\text{tw}}\left(\frac{r}{R} - 0.75\right) + \theta_{1c} \cos\psi + \theta_{1s} \sin\psi $$
*   **Flapping Kinematics:** Per our assumptions in Section 1.2, we treat the rotor as a **Rigid Disk** ($\beta = 0$). No flapping dynamics are calculated.
*   **Element Velocities:** We calculate the Tangential ($U_T$) and Perpendicular ($U_P$) wind speeds hitting the blade element.
    *   $U_T = \Omega r + V_{\text{edge}} \sin\psi$
    *   $U_P = V_{\text{axial}} + v_i(r, \psi)$
*   **Reverse-Flow Region:** The code actively scans for any area where $U_T \le 0$ (meaning the wind is hitting the back of the blade). If found, it flips the math using the Viterna-Corrigan $360^\circ$ extrapolation to prevent the code from crashing.

---

## STEP 4: Airfoil & Loads
**Goal:** Look up the aerodynamics and calculate the physical forces (N/m).

*   **Sectional Polars:** The solver calculates the local Angle of Attack ($\alpha = \theta - \phi$) and queries the **Boeing-Vertol VR-12** tabular CSV database to find the exact Lift ($C_l$) and Drag ($C_d$) coefficients.
*   **Compressibility:** The code calculates the local Mach number ($M = U/a$). If $M > 0.75$, it applies the **Prandtl-Glauert** scaling correction to account for transonic shockwaves forming on the blade.
*   **Elemental Loads [N/m]:** We convert the coefficients into raw physical forces:
    *   Normal Force (Lift): $\frac{dF_z}{dr} = \frac{1}{2}\rho U^2 c (C_l \cos\phi - C_d \sin\phi)$
    *   In-Plane Force (Drag): $\frac{dF_\psi}{dr} = \frac{1}{2}\rho U^2 c (C_l \sin\phi + C_d \cos\phi)$

---

## STEP 5: Integration & CG
**Goal:** Sum up all the tiny forces across the entire disk and transfer them to the Helicopter Body.

*   **Azimuthal Integration:** The code uses the **2D Periodic Trapezoidal Rule** to add up the forces from all $30 \times 72$ cells. 
*   **Shaft Loads (FS):** This massive integration yields the total forces acting on the Rotor Shaft:
    *   Total Thrust ($T$)
    *   Total Shaft Torque ($Q$) and Power ($P$)
    *   Hub Pitch Moment ($M_Y$) and Hub Roll Moment ($M_X$)
*   **Body Frame Transform:** Finally, the code applies a 3D Euler Rotation Matrix ($R_{B \leftarrow S}$) to take the forces from the tilted Nacelle Engine and map them perfectly onto the Aircraft Center of Gravity (CG). This allows our 6-DOF Trim Solver in Section 6 to mathematically fly the helicopter!
