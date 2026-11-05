"""The OpenDRIVE checker on small hand-written maps with known answers."""
import csv
import math

from uga_terrain import xodr_check as X

GOOD = """<?xml version="1.0"?>
<OpenDRIVE>
 <header revMajor="1" revMinor="6"><geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0]]></geoReference></header>
 <road id="1" length="100" junction="-1">
  <link><successor elementType="junction" elementId="9"/></link>
  <planView>
   <geometry s="0" x="0" y="0" hdg="0" length="50"><line/></geometry>
   <geometry s="50" x="50" y="0" hdg="0" length="50"><paramPoly3 aU="0" bU="50" cU="0" dU="0" aV="0" bV="0" cV="0" dV="0" pRange="normalized"/></geometry>
  </planView>
  <elevationProfile><elevation s="0" a="1" b="0.01" c="0" d="0"/></elevationProfile>
  <lanes>
   <laneSection s="0">
    <left><lane id="1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
    <center><lane id="0" type="none"/></center>
    <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
           <lane id="-2" type="sidewalk"><width sOffset="0" a="2" b="0" c="0" d="0"/></lane></right>
   </laneSection>
  </lanes>
 </road>
 <road id="2" length="15.707963" junction="9">
  <link><predecessor elementType="road" elementId="1" contactPoint="end"/></link>
  <planView><geometry s="0" x="100" y="0" hdg="0" length="15.707963"><arc curvature="0.1"/></geometry></planView>
  <elevationProfile><elevation s="0" a="2" b="0" c="0" d="0"/></elevationProfile>
  <lanes><laneSection s="0">
    <center><lane id="0" type="none"/></center>
    <right><lane id="-1" type="driving"><link><predecessor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
  </laneSection></lanes>
 </road>
 <junction id="9">
  <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start"><laneLink from="-1" to="-1"/></connection>
 </junction>
</OpenDRIVE>
"""


def _write(tmp_path, text, name="m.xodr"):
    p = tmp_path / name
    p.write_text(text)
    return p


def test_good_map_has_no_errors(tmp_path):
    o = X.check(_write(tmp_path, GOOD))
    assert o["counts"]["error"] == 0, o["findings"]
    assert not [f for f in o["findings"] if f["code"].startswith("lane_width")]
    assert o["topology"]["network_pieces"] == 1 and not o["topology"]["unconnected_roads"]


def test_arc_geometry_is_exact(tmp_path):
    _root, roads, _j = X.read(_write(tmp_path, GOOD))
    ref = X.reference(roads[1])
    # quarter... 15.7 m of a 10 m radius circle is a 90 deg turn ending at (110, 10)
    assert math.hypot(ref[-1, 1] - 110, ref[-1, 2] - 10) < 0.01
    assert abs(ref[-1, 3] - math.pi / 2) < 1e-3


def test_broken_maps_are_caught(tmp_path):
    bad = (GOOD.replace('length="100" junction="-1"', 'length="90" junction="-1"')       # wrong length
               .replace('<geometry s="50" x="50"', '<geometry s="50" x="50.5"')          # 50 cm gap
               .replace('<lane id="-2" type="sidewalk"', '<lane id="-3" type="sidewalk"')  # id skips
               .replace('to="-1"/>', 'to="-4"/>'))                                        # missing lane
    codes = {f["code"] for f in X.check(_write(tmp_path, bad))["findings"] if f["level"] == "error"}
    assert {"road_length", "geometry_gap", "lane_ids", "junction"} <= codes


def test_unsteady_lane_width_is_flagged(tmp_path):
    wobbly = GOOD.replace('<lane id="1" type="driving"><width sOffset="0" a="3.5" b="0"',
                          '<lane id="1" type="driving"><width sOffset="0" a="2" b="0.03"')
    codes = {f["code"] for f in X.check(_write(tmp_path, wobbly))["findings"]}
    assert "lane_width_steady" in codes and "lane_width_range" in codes


def test_replay_finds_lane_and_direction(tmp_path):
    xo = _write(tmp_path, GOOD)
    tr = tmp_path / "t.csv"
    with open(tr, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "yaw_deg"])
        for x in range(2, 98, 2):
            w.writerow([x, -1.75, 1 + 0.01 * x, 0.0])          # centre of lane -1, driving +x
    r = X.check(xo, [tr])["replay"][0]
    assert r["in_driving_lane_pct"] == 100 and r["lanes_used"] == ["1:-1"]
    assert r["lane_centre_offset_median_m"] < 0.01 and r["wrong_way_pct"] == 0
    assert abs(r["height_vs_road_median_m"]) < 0.01
    # the same path driven backwards is in the wrong direction
    with open(tr, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "yaw_deg"])
        for x in range(2, 98, 2):
            w.writerow([x, -1.75, 180.0])
    o = X.check(xo, [tr])
    assert o["replay"][0]["wrong_way_pct"] == 100
    assert "replay_direction" in {f["code"] for f in o["findings"]}


def test_lane_geometry_for_the_map(tmp_path):
    g = X.lane_geometry(_write(tmp_path, GOOD))
    drv = [l for l in g["lanes"] if l["type"] == "driving"]
    assert {(l["road"], l["lane"]) for l in drv} == {("1", 1), ("1", -1), ("2", -1)}
    assert any(l["junction"] for l in drv) and len(g["centre"]) == 2
    # lane -1 of road 1 lies 3.5 m right of the centre line (y = 0 .. -3.5)
    l = next(l for l in drv if l["road"] == "1" and l["lane"] == -1)
    assert abs(l["outer"][0][1] + 3.5) < 1e-6 and abs(l["inner"][0][1]) < 1e-6
