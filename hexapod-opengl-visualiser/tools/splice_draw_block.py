#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
mainp = ROOT / "src/app/legacy_main.cpp"
blockp = ROOT / "src/app/draw_modern_block.cpp"
main = mainp.read_text()
block = blockp.read_text()
s = main.index("void DrawWireCube(")
e = main.index("void LogJointPositions", s)
mainp.write_text(main[:s] + block + main[e:])
print("spliced", s, e)
