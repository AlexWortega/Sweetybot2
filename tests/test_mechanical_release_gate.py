"""Software success cannot override a failed mechanical audit."""
import json
import pytest
from scripts.common import ROOT,require_mechanical_gate

def test_confirmed_moving_part_interference_blocks_release():
    report=json.loads((ROOT/'reports/v04_motion_audit.json').read_text())
    assert report['complete'] and len(report['poses'])==39
    home=report['poses'][0]
    assert any(set(p['parts'])=={'fl_thigh_cheek_5','fl_shin_cheek_front'} and p['overlap_mm3']>3000 for p in home['intersections'])
    with pytest.raises(ValueError,match='interpenetrating'):require_mechanical_gate()
