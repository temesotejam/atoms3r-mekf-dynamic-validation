from pathlib import Path

p = Path("tools/test_v46_motor_validation_source_guards.py")
text = p.read_text(encoding="utf-8")
old = '    assert "R_y(pi)=diag(-1,+1,-1)" in runner\n'
new = '    assert "return {-r.ax_g, r.ay_g, -r.az_g};" in runner\n'
if text.count(old) != 1:
    raise RuntimeError(f"expected one stale Ry180 comment guard, got {text.count(old)}")
p.write_text(text.replace(old, new, 1), encoding="utf-8")
print("V46f guard updated for implementation-based Ry180 check")
