"""Audit final Milestone 3 deliverables and build Team9_Milestone3_CodePackage.zip."""

from pathlib import Path
import zipfile
from pptx import Presentation


ROOT = Path(__file__).resolve().parents[2]
PKG = Path(__file__).resolve().parents[1]


def audit_pptx(pptx_path: Path) -> None:
    prs = Presentation(str(pptx_path))
    assert len(prs.slides) == 30, f"Expected 30 slides, got {len(prs.slides)}"
    leftover_flags = []
    for s_idx, slide in enumerate(prs.slides, 1):
        for sh in slide.shapes:
            if sh.has_text_frame:
                txt = sh.text_frame.text
                if "____" in txt or "[replace with" in txt or "Paste Milestone" in txt or "Paste final" in txt:
                    leftover_flags.append((s_idx, txt[:80]))
            if sh.has_table:
                for r_i, row in enumerate(sh.table.rows):
                    for c_i, cell in enumerate(row.cells):
                        ct = cell.text.strip()
                        if "____" in ct:
                            leftover_flags.append((s_idx, f"Table({r_i},{c_i}): {ct}"))
                        elif ct == "" and s_idx not in (2,):
                            leftover_flags.append((s_idx, f"Empty cell Table({r_i},{c_i})"))
    if leftover_flags:
        raise RuntimeError(f"PPTX audit found unresolved placeholders: {leftover_flags}")
    print(f"PPTX audit PASSED: {pptx_path.name} (30 slides, 0 unresolved placeholders, 0 empty table cells)")


def build_zip() -> Path:
    zip_path = ROOT / "Team9_Milestone3_CodePackage.zip"
    include_paths = []
    for p in sorted(PKG.rglob("*")):
        if "__pycache__" in p.parts or p.suffix == ".pyc" or p.name == ".DS_Store":
            continue
        if p.is_file():
            include_paths.append(p)
    airfoils_dir = ROOT / "airfoils"
    if airfoils_dir.exists():
        for p in sorted(airfoils_dir.rglob("*")):
            if p.is_file() and p.name != ".DS_Store":
                include_paths.append(p)
    nb_path = ROOT / "milestone3_team9.ipynb"
    if nb_path.exists():
        include_paths.append(nb_path)

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for p in include_paths:
            zf.write(p, arcname=str(p.relative_to(ROOT)))

    size_mb = zip_path.stat().st_size / (1024 * 1024)
    print(f"Created reproducible code archive: {zip_path.name} ({len(include_paths)} files, {size_mb:.2f} MB)")
    return zip_path


if __name__ == "__main__":
    audit_pptx(ROOT / "Milestone3_Team9_Final.pptx")
    build_zip()
