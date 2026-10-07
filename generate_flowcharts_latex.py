import urllib.request
import urllib.parse
import os
import re
import fitz

def compress_tex(tex_source):
    # Remove leading whitespace on each line
    lines = [line.strip() for line in tex_source.split('\n')]
    # Join them
    tex = '\n'.join(lines)
    # Remove empty lines
    tex = re.sub(r'\n+', '\n', tex)
    return tex

def compile_latex_to_png(tex_source, output_png):
    tex_source = compress_tex(tex_source)
    url = "https://latexonline.cc/compile?text=" + urllib.parse.quote(tex_source)
    print(f"Compiling {output_png} via latexonline.cc (Length: {len(url)})...")
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    
    try:
        with urllib.request.urlopen(req, timeout=60) as response:
            pdf_bytes = response.read()
    except Exception as e:
        print(f"Error compiling {output_png}: {e}")
        return False
        
    doc = fitz.open(stream=pdf_bytes, filetype="pdf")
    page = doc[0]
    pix = page.get_pixmap(matrix=fitz.Matrix(6.0, 6.0), alpha=False)
    pix.save(output_png)
    print(f"Successfully saved {output_png} (Size: {pix.width}x{pix.height})")
    return True

tex_2_2 = r"""\documentclass[11pt,landscape]{article}
\usepackage[margin=0.5in]{geometry}
\usepackage{tikz}
\usepackage{amsmath}
\usetikzlibrary{shapes.geometric, arrows.meta, positioning, shadows}
\pagestyle{empty}
\begin{document}
\begin{figure}[htbp]
\centering
\resizebox{\textwidth}{!}{%
\begin{tikzpicture}[
node distance=1.6cm and 2.5cm,
box/.style={rectangle, draw=black!70, thick, fill=blue!5, text width=7.2cm, minimum height=7.2cm, align=left, rounded corners=4pt, drop shadow, inner sep=10pt, font=\small, anchor=center},
arrow/.style={-{Stealth[scale=1.3]}, thick, draw=black!80},
label/.style={font=\footnotesize\itshape, above, align=center, text width=2.5cm, inner sep=1pt}
]
\node (box1) [box] {
\textbf{\large [1] Conversion Flight Condition \& Unknowns $u$}\\[2mm]
\textbf{Fixed Conversion State Inputs:}\\
\textbullet~ Airspeed $V_\infty$, Altitude $h$, Climb Angle $\gamma$, Nacelle Angle $\theta_{nac}$\\
\textbullet~ Scheduled Rotor Speed $\Omega(\theta_{nac})$, Gross Weight $W = mg$\\[1.5mm]
\textbf{Unknown Trim Control Vector $u$} (Symmetric 4-DOF):\\
\textbullet~ $u = [\theta_0, \theta_{1s}, \theta_{1c}, \theta_{pitch}, \delta_e, \phi_{roll}]^T$\\[1.5mm]
\textbf{Box Bounds:}\\
\textbullet~ $\theta_0 \in [-5^\circ, 45^\circ]$, $\theta_{1s,1c} \in [-12^\circ, 12^\circ]$, $\delta_e \in [-25^\circ, 25^\circ]$
};
\node (box2) [box, right=of box1] {
\textbf{\large [2] Dual Counter-Rotating Rotor Evaluation}\\[2mm]
\textbf{Call 2D Edgewise BEMT Solver} (Port CCW \& Stbd CW):\\
\textbullet~ $F_{rot,L}, M_{rot,L} = \text{bemt}(\theta_0, \theta_{1c}, \theta_{1s}, +1)$\\
\textbullet~ $F_{rot,R}, M_{rot,R} = \text{bemt}(\theta_0, -\theta_{1c}, \theta_{1s}, -1)$\\[1.5mm]
\textbf{Lateral/Yaw Symmetry Cancellation:}\\
\textbullet~ Counter-rotation cancels net torque $M_Z$, side force $F_Y$, and roll $M_X$ in steady flight.\\[1.5mm]
\textbf{Rotor Wake \& Download Interference:}\\
\textbullet~ Induced wash $v_i$ modifies wing incidence and hover download.
};
\node (box3) [box, right=of box2] {
\textbf{\large [3] Airframe Aero: Wing, Fuselage, Nacelle \& Tail}\\[2mm]
\textbf{Wing Lift, Drag \& Pitching Moment:}\\
\textbullet~ $\alpha_w = \alpha_B + i_w - \alpha_{wash}(\theta_{nac}, \lambda_i)$\\
\textbullet~ $L_w = q S_w C_L(\alpha_w)$, \quad $D_w = q S_w (C_{D0} + C_L^2 / \pi A e)$\\[1.5mm]
\textbf{Fuselage + Tilting Nacelle Drag:}\\
\textbullet~ $D_{fuse+nac} = q S_w (C_{D0,fuse} + \Delta C_{D,nac} \sin^2 \theta_{nac})$\\[1.5mm]
\textbf{Horizontal \& Vertical Empennage Loads:}\\
\textbullet~ $L_{ht} = q S_{ht} (a_{ht} \alpha_{ht} + \tau_e \delta_e)$, \quad $M_{Y,ht} = -x_{ht} L_{ht}$
};
\node (box4) [box, below=1.8cm of box3] {
\textbf{\large [4] 6-DOF Force \& Moment Residual Vector $R(u)$}\\[2mm]
\textbf{Sum Component Loads in Body Axes $F_B$ about CG:}\\
\textbullet~ $R_1 = F_{X,rot} + F_{X,aero} - W \sin\theta_{pitch} = 0$ \textit{(Axial $X$)}\\
\textbullet~ $R_2 = F_{Y,rot} + F_{Y,aero} + W \cos\theta_{pitch}\sin\phi = 0$ \textit{(Side $Y$)}\\
\textbullet~ $R_3 = F_{Z,rot} + F_{Z,aero} + W \cos\theta_{pitch}\cos\phi = 0$ \textit{(Vert $Z$)}\\[1.5mm]
\textbf{Moment Equilibrium Residuals about CG:}\\
\textbullet~ $R_4 = \sum M_X = 0$, $R_5 = \sum M_Y = 0$, $R_6 = \sum M_Z = 0$
};
\node (box5) [box, left=of box4] {
\textbf{\large [5] Bounded Trust-Region Newton / LM Solver}\\[2mm]
\textbf{Numerical Root-Finder} (scipy hybrid Powell/LM):\\
\textbullet~ Minimize $\|W \cdot R(u)\|_2^2$ subject to $u_{min} \le u \le u_{max}$\\[1.5mm]
\textbf{Finite-Difference Jacobian:}\\
\textbullet~ $J_{ij} = \partial R_i / \partial u_j$ evaluated via trust-region step $\Delta u$\\[1.5mm]
\textbf{Convergence Criterion Check:}\\
\textbullet~ $\|F_{res}\|_\infty < 5.0$ N \textbf{AND} $\|M_{res}\|_\infty < 5.0$ N$\cdot$m\\
\textbullet~ If not converged: update $u \leftarrow u + \Delta u$ and re-evaluate [2]$\to$[4]
};
\node (box6) [box, left=of box5] {
\textbf{\large [6] Post-Convergence Feasibility Classification}\\[2mm]
\textbf{Verify Trim Solution Against Active Physical Limits:}\\
1) \textbf{Power Margin:} $2 P_{rotor} / \eta_{xmsn} \le P_{installed}(h)$\\
2) \textbf{Rotor Stall Margin:} $\alpha_{stall} - P_{95}(|\alpha_{blade}|) \ge 0^\circ$\\
3) \textbf{Wing Stall Margin:} $|\alpha_w| \le 15.5^\circ$\\
4) \textbf{Advancing Tip Mach:} $M_{tip} \le 0.85$ \& Margin $>0^\circ$\\[1.5mm]
\textbf{Output Status:}\\
\textbullet~ FEASIBLE TRIM vs. Categorized Failure Mode
};
\draw [arrow] (box1) -- node[label] {Trial State $u$} (box2);
\draw [arrow] (box2) -- node[label] {Rotor Loads\\\& Wake} (box3);
\draw [arrow] (box3) -- node[label, left, text width=2.5cm, align=right] {Aero Loads} (box4);
\draw [arrow] (box4) -- node[label] {Residual $R(u)$} (box5);
\draw [arrow] (box5) -- node[label] {Converged $u^*$} (box6);
\draw [arrow, dashed, -{Stealth[scale=1.3]}] (box5.north) -- node[label, right, xshift=1mm] {Iterate $u + \Delta u$} (box2.south);
\end{tikzpicture}%
}
\end{figure}
\end{document}
"""

tex_2_3 = r"""\documentclass[11pt,landscape]{article}
\usepackage[margin=0.5in]{geometry}
\usepackage{tikz}
\usepackage{amsmath}
\usetikzlibrary{shapes.geometric, arrows.meta, positioning, shadows}
\pagestyle{empty}
\begin{document}
\begin{figure}[htbp]
\centering
\resizebox{\textwidth}{!}{%
\begin{tikzpicture}[
node distance=1.6cm and 2.5cm,
box/.style={rectangle, draw=black!70, thick, fill=blue!5, text width=7.2cm, minimum height=7.2cm, align=left, rounded corners=4pt, drop shadow, inner sep=10pt, font=\small, anchor=center},
arrow/.style={-{Stealth[scale=1.3]}, thick, draw=black!80},
label/.style={font=\footnotesize\itshape, above, align=center, text width=2.5cm, inner sep=1pt}
]
\node (box1) [box] {
\textbf{\large [1] Mission Profile \& Conversion Corridor}\\[2mm]
\textbf{Mission Segments:}\\
\textbullet~ Hover $\to$ Outbound Trans $\to$ Climb $\to$ Cruise $\to$ Descent $\to$ Inbound Trans $\to$ Landing\\[1.5mm]
\textbf{Time-Stepped Transition State Vector} at step $k$ ($\Delta t \approx 1$s):\\
\textbullet~ $[t_k, h_k, V_k, V_{ground,k}, \gamma_k, \theta_{nac}(t_k), \Omega(t_k), W_k]$\\[1.5mm]
\textbf{Corridor-Guided Schedule:}\\
\textbullet~ Smooth cosine/linear nacelle tilt $\theta_{nac}(V)$ from $90^\circ$ (Heli) to $0^\circ$ (Airplane) inside feasible $V$-$\theta_{nac}$ corridor.
};
\node (box2) [box, right=of box1] {
\textbf{\large [2] Quasi-Steady Longitudinal Acceleration \& Trim}\\[2mm]
\textbf{Include Longitudinal Inertial Acceleration Term:}\\
\textbullet~ Effective Axial Demand: $F_{X,net} = m \frac{dV_k}{dt} + W_k \sin\gamma_k$\\[1.5mm]
\textbf{Online 6-DOF Trim Solution at Time Step $t_k$:}\\
\textbullet~ Solve $u_k^* = [\theta_0, \theta_{1s}, \theta_{1c}, \theta_{pitch}, \delta_e, \phi]_k$\\
\textbullet~ Use warm-start guess $u_{k-1}^*$ for rapid (2-4 iter) convergence.\\[1.5mm]
\textbf{Extract Lift Sharing:}\\
\textbullet~ Rotor Lift Share $L_{rot}(t_k)$ vs. Wing Lift $L_{wing}(t_k)$
};
\node (box3) [box, right=of box2] {
\textbf{\large [3] Aerodynamic, Power \& Control Limit Gate}\\[2mm]
\textbf{Real-Time Envelope Verification at $t_k$:}\\
1) \textbf{Shaft Power:} $P_{req}(t_k) \le P_{avail}(h_k)$ (ISA lapse)\\
2) \textbf{Stall Margins:} $\Delta\alpha_{rot} > 0^\circ$, $|\alpha_w| < 15.5^\circ$\\
3) \textbf{Helical Advancing Tip Mach:} $M_{tip}(t_k) \le 0.85$\\
4) \textbf{Nacelle Tilt Rate:} $|d\theta_{nac}/dt| \le 8^\circ/\text{s}$ \& Control Bounds\\[1.5mm]
\textbf{Constraint Enforcement:}\\
\textbullet~ Flag/Reject any transition state violating the corridor boundaries.
};
\node (box4) [box, below=1.8cm of box3] {
\textbf{\large [4] Propulsion Power, SFC \& Fuel Burn Update}\\[2mm]
\textbf{Total Aircraft Shaft Power Demand at $t_k$:}\\
\textbullet~ $P_{total}(t_k) = (P_{rot,L} + P_{rot,R}) / \eta_{gearbox} + P_{acc}$\\[1.5mm]
\textbf{SFC Mass Integration over $\Delta t$:}\\
\textbullet~ $dm_{fuel,k} = (\frac{\text{SFC}}{3600}) P_{total}(t_k) \Delta t$\\[1.5mm]
\textbf{Fuel \& Gross Weight State Update:}\\
\textbullet~ $m_{fuel}(t_{k+1}) = m_{fuel}(t_k) - dm_{fuel,k}$\\
\textbullet~ $W_{k+1} = W_k - dm_{fuel,k} \cdot g$
};
\node (box5) [box, left=of box4] {
\textbf{\large [5] Trajectory Kinematics \& State Continuity}\\[2mm]
\textbf{Update Position, Altitude \& Ground Speed:}\\
\textbullet~ $V_{ground}(t_k) = V_k \cos\gamma_k - V_{wind}$\\
\textbullet~ $h_{k+1} = h_k + V_k \sin\gamma_k \Delta t$\\
\textbullet~ $x_{k+1} = x_k + V_{ground}(t_k) \Delta t$\\[1.5mm]
\textbf{Enforce $C^0$ State Continuity:}\\
\textbullet~ Across Hover $\leftrightarrow$ Transition $\leftrightarrow$ Cruise boundaries.\\
\textbullet~ Continuous $V, h, W, \theta_{nac}, \Omega$, and trim controls.
};
\node (box6) [box, left=of box5] {
\textbf{\large [6] Mission Telemetry \& Section 8 Deliverables}\\[2mm]
\textbf{Record Full Time-History Telemetry Arrays:}\\
\textbullet~ $t \in [0, T_{mission}]$\\
\textbullet~ $h(t), V(t), \theta_{nac}(t), \Omega(t), [\theta_0, \theta_{1s}, \delta_e](t)$\\
\textbullet~ $P_{req}(t)$ vs $P_{avail}(t), m_{fuel}(t)$, Stall Margins, Lift Share \%\\[1.5mm]
\textbf{Generate Deliverables:}\\
\textbullet~ Sec 8.2 Outbound and Inbound Conversion Plots\\
\textbullet~ Overall Mission Summary Table
};
\draw [arrow] (box1) -- node[label] {State($t_k$)} (box2);
\draw [arrow] (box2) -- node[label] {Trim Sol\\$u_k^*$} (box3);
\draw [arrow] (box3) -- node[label, left, text width=2.5cm, align=right] {Feasible Step} (box4);
\draw [arrow] (box4) -- node[label] {$W_{k+1}, m_{fuel}$} (box5);
\draw [arrow] (box5) -- node[label] {Telemetry} (box6);
\draw [arrow, dashed, -{Stealth[scale=1.3]}] (box5.north) -- node[label, right, xshift=1mm] {Next Step $k \leftarrow k+1$} (box2.south);
\end{tikzpicture}%
}
\end{figure}
\end{document}
"""

compile_latex_to_png(tex_2_2, "slide_2_2_trim_solver_flowchart.png")
compile_latex_to_png(tex_2_3, "slide_2_3_mission_planner_v2_flowchart.png")
