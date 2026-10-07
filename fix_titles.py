import json

def fix_titles():
    with open("milestone_2.ipynb", "r") as f:
        nb = json.load(f)

    cell = nb["cells"][8]
    if cell["cell_type"] != "code":
        print("Cell 8 is not code!")
        return

    src_lines = "".join(cell["source"]).split("\n")
    new_src = []
    
    changes = 0
    for line in src_lines:
        new_line = line
        
        # 3.1(a)
        if 'ax31a.set_title("Sec 3.1(a):' in new_line and "RPM" not in new_line:
            new_line = new_line.replace(
                'ax31a.set_title("Sec 3.1(a): Spanwise Thrust Distribution dT/dr [N/m]\\nExact Overlay of 1D Axisymmetric & 2D Azimuth-Resolved Solvers"',
                'ax31a.set_title(f"Sec 3.1(a): Spanwise Thrust Distribution dT/dr [N/m] (Hover RPM={rpm_hov:.0f})\\nExact Overlay of 1D Axisymmetric & 2D Azimuth-Resolved Solvers"'
            )
            
        # 3.1(b)
        elif 'ax31b.set_title("Sec 3.1(b):' in new_line and "RPM" not in new_line:
            new_line = new_line.replace(
                'ax31b.set_title("Sec 3.1(b): Spanwise Shaft Power Distribution dP/dr [kW/m]\\nConfirms Zero Spurious Azimuthal Drift in Axisymmetric Limits"',
                'ax31b.set_title(f"Sec 3.1(b): Spanwise Shaft Power Distribution dP/dr [kW/m] (Cruise RPM={rpm_cr:.0f})\\nConfirms Zero Spurious Azimuthal Drift in Axisymmetric Limits"'
            )

        # 3.2(a)
        elif 'style_rotor_polar(ax32a, f"Sec 3.2(a):' in new_line and "RPM=" not in new_line:
            new_line = new_line.replace(
                'm/s, θ_nac={tn_rep:.0f}°, μ={m2_rep.mu_edge:.2f})"',
                'm/s, θ_nac={tn_rep:.0f}°, μ={m2_rep.mu_edge:.2f}, RPM={rpm_rep:.0f})"'
            )

        # 3.2(b) - wait, this was already fixed earlier, but check
        elif 'style_rotor_polar(ax32b, f"Sec 3.2(b):' in new_line and "RPM=" not in new_line:
            new_line = new_line.replace(
                'Lift)")',
                'Lift, RPM={rpm_rep:.0f})")'
            )

        # 3.3(a)
        elif 'style_rotor_polar(ax33a, f"Sec 3.3(a):' in new_line and "RPM=" not in new_line:
            new_line = new_line.replace(
                '(Min = {m2_rep.min_ut_ms:.1f} m/s)")',
                '(RPM={rpm_rep:.0f}, Min = {m2_rep.min_ut_ms:.1f} m/s)")'
            )

        if new_line != line:
            changes += 1
            print(f"Fixed: {new_line.strip()[:100]}")
            
        new_src.append(new_line)

    cell["source"] = [s + "\n" for s in new_src[:-1]] + [new_src[-1]]
    
    print(f"Total changes: {changes}")
    
    with open("milestone_2.ipynb", "w") as f:
        json.dump(nb, f, indent=1, ensure_ascii=False)
    print("Saved to milestone_2.ipynb")

if __name__ == "__main__":
    fix_titles()
