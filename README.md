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

Your browser opens at http://127.0.0.1:8090. Then:

1. **Settings** → set *Recording folders*. The default is `~/Downloads/ros2support`. Any `.mcap` files inside, including sub-folders, appear on the left.
2. Tick one or more recordings, type a map name, and press **Create**.
3. Press **Build map**. Progress shows at the top, and **Stop** cancels.
4. Look at the results:
   - **Map**: surface type, elevation, slope, roughness and intensity. Hover over a spot for its values, or click a road.
   - **3D**: the cleaned ground point cloud.
   - **Road profile**: grade, banking, widths, curb heights and roughness along each road.
   - **Files**: download the outputs.
5. After changing Settings, use **Re-run from** to redo later steps without reading the recordings again.

To merge more drives into an existing map, tick them and press **Add ticked recordings**, then **Build map**.

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
2. **Ground.** Patchwork++ separates ground from objects in each scan, and the car's own body is masked out.
3. **Map.** Ground points from all drives go into a 20 cm × 10 cm voxel grid. Each cell keeps its dominant height, which drops cars and people that passed by. Raised blobs left over (parked cars, bins) are removed, and small holes are filled.
4. **Surface layers.** Elevation, slope, aspect and roughness (plane-fit RMS plus point spread) are computed per cell.
5. **Roads.** Cross-sections every metre along each drive find the road edges (curb, drop, rough or steep ground) and the curb height. Past each edge they find the verge and the sidewalk (a smooth, flat, raised strip). Banking and grade come from the surface itself, and the car's own roll and pitch are kept for comparison.
6. **Export.** OpenDRIVE, CSV, PLY and raster files are written.

All tuning values are in `configs/default.yaml`. Changes made in the app go to `configs/local.yaml`.

## Tests

```bash
pytest
```

The tests build a synthetic street (4 % grade, 3° bank, 15 cm curbs, verge and sidewalk), simulate a lidar drive past a parked car, and check the measurements and the OpenDRIVE output.
