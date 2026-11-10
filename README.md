# UGA Terrain Toolkit

A standalone app that turns ROS 2 `.mcap` lidar recordings into a clean 3D terrain map of everywhere a car can drive or a person can walk, and exports it as **OpenDRIVE (.xodr)**.

- Builds one merged map from any number of recordings. The GNSS/INS and lidar poses are fused, and every drive shares one geographic origin.
- Removes cars, pedestrians, poles, street lights, vegetation and other objects. Only the ground surface is kept.
- Maps roads, curbs, grass verges and sidewalks along every drive.
- Measures elevation, grade, banking, cross slope, curvature, curb height, sidewalk width and height, and surface roughness.
- Writes OpenDRIVE 1.6 with real geometry (paramPoly3), an elevation profile, superelevation (banking), driving lanes, curb and verge borders, and sidewalks with their heights. It is georeferenced (tmerc) and opens in CARLA, RoadRunner and esmini.

## Install (once)

```bash
git clone https://github.com/shao-lab-uga/uga_terrain_toolkit.git
cd uga_terrain_toolkit
conda env create -f environment.yml      # creates the "uga_terrain" environment
conda activate uga_terrain
```

Without conda, run `pip install -e ".[fast]"`. The `fast` extras are KISS-ICP lidar odometry and Patchwork++ ground segmentation. Both are optional. Without them the app uses GNSS/INS poses and a built-in ground filter.

## Run the app

```bash
conda activate uga_terrain
cd uga_terrain_toolkit
python -m uga_terrain app        # or: scripts/run_app.sh
```

Your browser opens at http://127.0.0.1:8090. If an older copy of the app is still running (from another terminal, say), it is stopped first, so you always get the version you just pulled. If the port is held by something else, the app moves to the next free port and prints it. Then:

1. **Settings** → set *Recording folders*. The default is `~/Downloads/ros2support`. Any `.mcap` files inside, including sub-folders, appear on the left.
2. Tick one or more recordings, type a map name, and press **Create**.
3. Press **Build map**. Progress shows at the top, and **Stop** cancels.
4. Look at the results:
   - **Map**: shaded relief (height colours with hill shading), surface type, elevation, slope, roughness and intensity. Hover over a spot for its values, or click a road.
   - **3D**: the cleaned ground as a lit, shadowed relief model, coloured by height or by surface type (or as points). Heights can be exaggerated so gentle slopes show.
   - **Road profile**: grade, banking, widths, curb heights and roughness along each road.
   - **Accuracy**: press **Check accuracy** to see how far the map can be trusted (see below), and look at the raw lidar points across any road, coloured by scan.
   - **Files**: download the outputs.
5. After changing Settings, use **Re-run from** to redo later steps without reading the recordings again.

To name a street, pick it in **Road profile** (or click it on the map), type a name and press **Save name**. The name is shown on the map and written into the .xodr straight away.

To delete a map, select it and press **Delete map** at the bottom of the left panel. This deletes its outputs, scan cache and snapshots, but never the recordings. Use **Export map** first if you may want it back.

To merge more drives into an existing map, tick them and press **Add ticked recordings**, then **Build map**.

**Mapping only part of a recording:** press **Part…** next to a recording in the map's panel. Enter a start and end in seconds (from the first lidar scan) or in lidar frame numbers, press **Use this part**, then **Build map**. Leave a box empty to mean the start or end of the recording. To map less of the surroundings, lower **Settings → Max ground range** (for example 15 m).

**Moving around the map and 3D views:** drag with the left button to pan, hold the right button and drag to rotate, and scroll to zoom. **Fit** turns north back up. The legend shows the elevation of each colour, both relative to the start of the first drive and as GPS altitude.

**Saving and sharing.** Maps save automatically. In addition:
- **Save snapshot** (or Ctrl+S) keeps a copy of the map and the settings as they are right now. **Restore** puts both back later.
- **Export map** downloads the whole map as one `.zip`. **Import map…** loads that zip on another computer. Tick *with scan cache* to include the per-frame scan data. That makes the file bigger but lets the other computer rebuild from *Elevation map* without the recordings.
- **Settings → Export settings / Import settings…** moves your settings between computers as a `.yaml` file.

## Outputs

Outputs are written to `~/uga_terrain_work/projects/<map>/outputs/`:

| File | Contents |
|---|---|
| `<map>.xodr` | OpenDRIVE road network |
| `road_profile.csv` | one row per metre of road: elevation, grade, banking, cross slopes, curvature, road/verge/sidewalk widths, curb and sidewalk heights, roughness, and the car's own pitch and roll |
| `ground.ply` | merged ground surface, one point per 20 cm cell, coloured by surface type |
| `rasters/` (zip in app) | elevation, slope, roughness, label and intensity as float TIFF tiles with world files |
| `trajectory.csv` | final 3D pose of every lidar frame used |
| `summary.json` | origin (lat/lon/alt), areas, road list, processing notes |

## Recording requirements

| What | Default topic | Type |
|---|---|---|
| Lidar | `/lidar/points` | `sensor_msgs/PointCloud2` |
| GNSS | `/oxts/fix` | `sensor_msgs/NavSatFix` |
| INS orientation | `/oxts/imu` | `sensor_msgs/Imu` |
| Lidar mounting | `/tf_static` | `base_footprint → lidar` |

All of these can be changed in **Settings**. Press **Info** next to a recording to check that its topics are present. Recordings that were cut off before the end (no MCAP footer) are still read.

## Command line

```bash
python -m uga_terrain inspect drive.mcap
python -m uga_terrain build --name campus drive1.mcap drive2.mcap
python -m uga_terrain build --name campus --start roads     # redo roads + export only
```

## How it works

1. **Poses.** The GNSS/INS track (WGS84 → local ENU) is fused with KISS-ICP lidar odometry. ICP is checked against GNSS/INS every frame.
2. **Motion correction.** Each Ouster scan takes 0.1 s, and the car moves during it. Every point is moved to where the lidar was at the moment that point was measured, using its per-point time stamp (Settings: lidar.deskew).
3. **Ground.** Patchwork++ separates ground from objects in each scan, and the car's own body is masked out.
4. **Map.** Ground points from all drives go into a 20 cm × 10 cm voxel grid. Each cell keeps its dominant height, which drops cars and people that passed by. Raised blobs left over (parked cars, bins) are removed, and small holes are filled.
5. **Surface layers.** Elevation, slope, aspect and roughness (plane-fit RMS plus point spread) are computed per cell.
6. **Lining up drives.** When a map has several recordings, each one is matched to the ones before it on the ground they share: sideways and heading from the lane paint and crosswalks (lidar intensity), height and tilt from the road surface. GNSS alone leaves drives of the same street tens of centimetres and about a degree apart (`register:` in the settings; result in `align.json`).
7. **Junctions.** A drive that turns marks a junction where the straight lines before and after the turn meet; a road edge that opens up on one or both sides marks a side street nobody turned into. Around each junction the paved surface is found in the elevation map (smooth ground bounded by curbs), and the junction's edge is put where the corner curves end and each street settles to its own width. Every street at the junction that no drive went into (the street continuing on the far side, cross streets) gets a short road so turns have somewhere to lead; its width is measured from its curbs. Drives are cut at the junction's edge.
8. **Roads.** The pieces outside junctions become roads; the same street driven more than once, in either direction, is one road. Cross-sections every metre find the road edges (curb, drop, rough or steep ground) and the curb height. Past each edge they find the verge and the sidewalk (a smooth, flat, raised strip). Banking and grade come from the surface itself, and the car's own roll and pitch are kept for comparison.
9. **Export.** OpenDRIVE, CSV, PLY and raster files are written.

All tuning values are in `configs/default.yaml`. Changes made in the app go to `configs/local.yaml`.

## Random errors on Intel 13th/14th-gen CPUs

Some i7/i9 13th and 14th-gen CPUs were damaged by a voltage fault and give random, different errors on every build, even with Intel's microcode fix. If that happens, set **Settings > CPU cores to use** to **e-cores**. The app then runs only on the slower efficiency cores, which are not affected. Builds are slower, and nothing else changes. Set it back to **all** once the CPU is replaced.

## Lanes and OpenDRIVE checks

**Lanes.** Lane paint reflects the lidar much more strongly than asphalt. The app builds a road-aligned image of relative lidar intensity, finds the thin bright stripes in it, links them into lines along the road and fits each with a smooth curve. Lanes are the strips between neighbouring boundaries (curb, lines, curb). The painted line nearest the middle of the road is taken as the centre line between the two directions, and the .xodr reference line is placed on it. Driving lanes keep a steady width; where the curb moves away (turn pockets, side streets, parking) the extra width becomes a shoulder lane. Without paint, the paved width is split evenly. The lidar cannot see paint colour, so the centre line is written as yellow and other lines as white.

**Stop lines and crosswalks.** Near each junction the lidar paint is searched for bands across the road: a crosswalk (bars or two lines, 1.8-6 m deep, across most of the road; it may be at an angle) or a stop line (a thin band across the lanes heading into the junction). They are written as road objects (`crosswalk`, and `roadMark` with subtype `stopLine`) with their outline, and drawn on the Map tab.
Crosswalks are also found straight from the paint around the junction, at any angle: two parallel side lines 1.8-4.5 m apart (with or without bars between), and a lone line just outside one as its stop line. These replace the road-by-road guess where both found the same crosswalk.

**Lidar layer (3D tab).** Tick **Lidar** to see every lidar point of the map on top of the terrain: buildings, poles, signal wires, trees, signs and passing cars. Colour by intensity, by height above the ground, or ground / above ground; show only what is above the ground or everything; **steady only** hides points seen in fewer than 4 scans (mostly moving cars and people). The same cloud is saved as `lidar.ply` (0.1 m voxels with intensity, a ground flag and how many scans hit each one). Maps built before v0.9.0 need a rebuild from the start (Scan) to get it.

**Road model view (Map and 3D).** Pick **Layer → Road model (.xodr)** on the Map tab or **Colour → Road model** in 3D to see the map the way a driving simulator would: asphalt lanes, white lane and edge lines, yellow centre lines, zebra crosswalks and stop lines, shoulders, curbs and sidewalks, all taken from the OpenDRIVE file, with the junction filled out to its curbs. In 3D it is laid on the measured ground, so grades, banking and curb steps still show; other ground is drawn as grass. Signal poles stay on the Surface layer only.

**Tidied surface.** After the roads are found, gaps in the scan are filled (up to 6 m across anywhere, such as under trees and parked cars, and up to 15 m on roads and in junctions), the junction is labelled road, and single-cell specks in the surface labels are cleaned up. Measured heights are never changed; filled cells are marked with 2 in the `filled` layer and emptied again before the roads are found on the next run. The limits are under `tidy` in the settings file. Re-run from **Roads** to tidy an existing map.

**Signal poles, span wires and traffic lights.** At each junction the lidar is searched for traffic signals hung on wires: thin straight wires 4-9 m up (told apart from tree canopy by their shape), the pole each wire runs to (a narrow column from the ground up to the wire, found along the wire's line even where it disappears into a tree), one pole per corner, and the signal heads hanging under the wires. They are drawn on the Surface layer of the Map tab and with Surface colouring in 3D (not on the other layers for now), saved in `signals.json`, and written to the .xodr: poles as `<object type="pole">` with height and radius, heads as `<signal type="1000001">` (traffic light) on the road whose incoming lanes they are over, with their real position in `<positionInertial>`. Setting `signals.detect` turns it off. On the test intersection all four poles are found on both clips and agree to within 0.9 m (the GPS difference between the two drives).

**Copying markings from another map.** When a drive saw a crosswalk badly or not at all (a corner cut short, paint seen from far away), another map of the same intersection can lend it its markings: on the Map tab pick the map next to **Crosswalks from** and press **Copy**. They are placed by GPS, then moved by the shift and turn that best lines up the paint both maps saw (usually well under a metre). Markings this map already has are not doubled. The .xodr is rewritten and checked right away; no rebuild is needed, and later rebuilds copy them again. The chip "from <map> ×" takes the copied ones off.

**Road details and the lane editor.** Each road gets a type and speed limit (`xodr.road_defaults`, 25 mph town roads unless changed; connecting roads take the incoming road's). On the Map tab, click a lane to change its road's type, speed limit and direction (two-way, or one-way either way), the lane's type and width, or delete the road; click a connecting lane in a junction to remove that turn. **Save and rewrite .xodr** applies the changes and re-runs the OpenDRIVE check in a few seconds, without a rebuild. Changes are kept with the map (`project.json`, `edits`) and applied again after every rebuild; lane numbers refer to the lanes as built, so check them after a rebuild that changes the lanes.

**Junctions.** Every road that ends at a junction is linked to it, and every movement between two of its roads (straight on, left, right; no U-turns) gets a connecting road with one lane. Which lane connects to which: right turns from the rightmost lane to the rightmost lane, left turns from the leftmost to the leftmost, straight on lane by lane from the right. A connecting lane starts and ends exactly on the edges of the lanes it joins. Where a drive made the movement, the lane follows the driven path; otherwise it is a smooth curve, and its name ends in "(not driven)" so turns that are not allowed can be found and deleted (or set `junctions.movements: driven`). The elevation inside the junction follows the measured ground. Two roads that continue each other straight through a junction are one street: their lanes are found together, so the centre line and lane widths match on both sides.

**Checks.** After every export the .xodr is read back by the app's own OpenDRIVE reader (`uga_terrain/xodr_check.py`) and checked:

- **Format:** ids, geometry lengths and joins, profile records, lane ids and widths, links between roads, junction connections and lane links.
- **Lanes:** driving lane widths are plausible (2.5–4.6 m) and steady along the road.
- **Network:** roads with no connection, separate pieces of network.
- **Lane links:** linked lanes really meet where their roads join (no gap between a lane and the connecting lane it links to).
- **Replay:** each recording's trajectory is placed onto the map lane by lane, following the map's lane links through junctions: share of poses inside a driving lane, the route of lanes used, lane changes, moves between lanes the map does not connect, distance from the lane centre, heading against the lane direction, and vehicle height against the road surface.

Results are in `xodr_check.json` and on the **Accuracy** tab. The **Map** tab draws the lanes from the .xodr (blue along the road, green the other way, orange the connecting lanes through junctions, yellow dashes the centre line; checkbox **Lanes**). Any .xodr can be checked from the command line, with any trajectory in the map's x/y frame:

```bash
python -m uga_terrain check map.xodr --trajectory trajectory.csv
```

## Accuracy check

There is no surveyed ground truth, so the check compares the recordings with themselves:

- **Drive path.** How often lidar odometry disagreed with GNSS/INS, and how far the final path is from GNSS.
- **Height error by distance.** Each ground point against the same cell seen from under 12 m away. This shows how far out the lidar heights can be trusted.
- **Repeatability.** Two maps from the first and second half of the scans, and every road measurement repeated on each. The difference is how repeatable each value is.
- **Cross-section viewer.** The raw points of every scan in a 50 cm slice across a road. Scans that line up form one thin band.

The findings list says what to change when something is off (for example, a lower max ground range).

## Tests

```bash
pytest
```

The tests build a synthetic street (4 % grade, 3° bank, 15 cm curbs, verge and sidewalk), simulate a lidar drive past a parked car, and check the measurements and the OpenDRIVE output.
