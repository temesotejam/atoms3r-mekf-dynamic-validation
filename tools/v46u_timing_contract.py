"""Strict inverse of reviewed timing-only changes for retained baseline tests."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1]
def original_timing_file(path):
    data=(ROOT/path).read_text()
    edits=json.loads((ROOT/'tools/v46u_timing_delta.json').read_text())
    for edit in reversed(edits):
        if edit['path'] != path:continue
        if data.count(edit['new']) != 1:raise ValueError('Timing delta changed: '+path)
        data=data.replace(edit['new'],edit['old'],1)
    return data.encode()
