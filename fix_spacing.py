import json

with open("milestone2_final.ipynb", "r") as f:
    nb = json.load(f)

for i, cell in enumerate(nb["cells"]):
    if "disk_heatmaps = go.FigureWidget" in "".join(cell.get("source", [])):
        source = cell["source"]
        new_source = []
        for line in source:
            if "horizontal_spacing=0.10" in line:
                new_source.append(line.replace("0.10", "0.22"))
            elif "(\"RdBu_r\", \"qT<br>[N/m²]\", 0.45, 0.79)," in line:
                new_source.append(line.replace("0.45", "0.45")) # actually 0.45 might be fine if spacing is 0.22, domain ends at 0.39
            elif "(\"Viridis\", \"UT<br>[m/s]\", 0.45, 0.21)," in line:
                new_source.append(line.replace("0.45", "0.45"))
            elif "(\"RdBu_r\", \"qθ<br>[N/m²]\", 1.02, 0.79)," in line:
                new_source.append(line.replace("1.02", "1.05"))
            elif "(\"RdBu_r\", \"UP<br>[m/s]\", 1.02, 0.21)," in line:
                new_source.append(line.replace("1.02", "1.05"))
            elif "margin=dict(l=35, r=35, t=120, b=35)," in line:
                new_source.append(line.replace("r=35", "r=85"))
            else:
                new_source.append(line)
        cell["source"] = new_source
        break

with open("milestone2_final.ipynb", "w") as f:
    json.dump(nb, f, indent=1)
