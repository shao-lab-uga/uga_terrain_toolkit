"""Synthetic ROS 2 MCAP recordings for tests (needs ``mcap-ros2-support``).

Schemas are written the way rosbag2 embeds them: the type's own fields,
then each dependency after a ``====`` separator and a ``MSG:`` line.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

SEP = "=" * 80

TIME = "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec"
HEADER = "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id"


def _join(main: str, *deps: str) -> str:
    return ("\n" + SEP + "\n").join([main, *deps]) + "\n"


POINTFIELD = """MSG: sensor_msgs/PointField
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8
string name
uint32 offset
uint8  datatype
uint32 count"""

SCHEMAS: Dict[str, str] = {
    "sensor_msgs/msg/PointCloud2": _join("""std_msgs/Header header
uint32 height
uint32 width
PointField[] fields
bool    is_bigendian
uint32  point_step
uint32  row_step
uint8[] data
bool is_dense""", POINTFIELD, HEADER, TIME),
    "sensor_msgs/msg/Image": _join("""std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data""", HEADER, TIME),
    "sensor_msgs/msg/CompressedImage": _join("""std_msgs/Header header
string format
uint8[] data""", HEADER, TIME),
    "sensor_msgs/msg/CameraInfo": _join("""std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9]  k # 3x3 row-major
float64[9]  r
float64[12] p
uint32 binning_x
uint32 binning_y
RegionOfInterest roi""", """MSG: sensor_msgs/RegionOfInterest
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify""", HEADER, TIME),
    "nav_msgs/msg/Odometry": _join("""std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist""", """MSG: geometry_msgs/PoseWithCovariance
Pose pose
float64[36] covariance""", """MSG: geometry_msgs/Pose
Point position
Quaternion orientation""", """MSG: geometry_msgs/Point
float64 x
float64 y
float64 z""", """MSG: geometry_msgs/Quaternion
float64 x 0
float64 y 0
float64 z 0
float64 w 1""", """MSG: geometry_msgs/TwistWithCovariance
Twist twist
float64[36] covariance""", """MSG: geometry_msgs/Twist
Vector3  linear
Vector3  angular""", """MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z""", HEADER, TIME),
    "novatel_oem7_msgs/msg/BESTPOS": _join("""std_msgs/Header header
Oem7Header nov_header
SolutionStatus sol_status
PositionOrVelocityType pos_type
float64 lat
float64 lon
float64 hgt
float32 undulation
uint32 datum_id
float32 lat_stdev
float32 lon_stdev
float32 hgt_stdev
string stn_id""", """MSG: novatel_oem7_msgs/Oem7Header
string message_name
uint16 message_id
uint8 message_type
uint32 sequence_number
uint8 time_status
uint16 gps_week_number
uint32 gps_week_milliseconds""", """MSG: novatel_oem7_msgs/SolutionStatus
uint32 SOL_COMPUTED = 0
uint32 status""", """MSG: novatel_oem7_msgs/PositionOrVelocityType
uint32 type""", HEADER, TIME),
    "sensor_msgs/msg/NavSatFix": _join("""std_msgs/Header header
NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type""", """MSG: sensor_msgs/NavSatStatus
int8 STATUS_NO_FIX =  -1
int8 status
uint16 service""", HEADER, TIME),
    "sensor_msgs/msg/Imu": _join("""std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance""", """MSG: geometry_msgs/Quaternion
float64 x 0
float64 y 0
float64 z 0
float64 w 1""", """MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z""", HEADER, TIME),
    "tf2_msgs/msg/TFMessage": _join("""geometry_msgs/TransformStamped[] transforms""",
                                    """MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
Transform transform""", """MSG: geometry_msgs/Transform
Vector3 translation
Quaternion rotation""", """MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z""", """MSG: geometry_msgs/Quaternion
float64 x 0
float64 y 0
float64 z 0
float64 w 1""", HEADER, TIME),
}


def header(t: float, frame: str = "base") -> dict:
    sec = int(t)
    return {"stamp": {"sec": sec, "nanosec": int(round((t - sec) * 1e9))},
            "frame_id": frame}


def pointcloud(t: float, pts) -> dict:
    import numpy as np
    pts = np.asarray(pts, dtype=np.float32)
    fields = [{"name": n, "offset": 4 * k, "datatype": 7, "count": 1}
              for k, n in enumerate(("x", "y", "z", "intensity"))]
    return {"header": header(t, "lidar"), "height": 1, "width": len(pts),
            "fields": fields, "is_bigendian": False, "point_step": 16,
            "row_step": 16 * len(pts), "data": pts.tobytes(), "is_dense": True}


def image_rgb(t: float, img) -> dict:
    h, w, _ = img.shape
    return {"header": header(t, "cam"), "height": h, "width": w,
            "encoding": "rgb8", "is_bigendian": 0, "step": w * 3,
            "data": img.tobytes()}


def odom(t: float, xyz, quat) -> dict:
    return {"header": header(t, "map"), "child_frame_id": "base_link",
            "pose": {"pose": {"position": dict(zip("xyz", xyz)),
                              "orientation": dict(zip("xyzw", quat))},
                     "covariance": [0.0] * 36},
            "twist": {"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
                      "covariance": [0.0] * 36}}


def camera_info(t: float, w: int, h: int, d: List[float]) -> dict:
    k = [100.0, 0, 50.0, 0, 110.0, 40.0, 0, 0, 1]
    return {"header": header(t, "camera2"), "height": h, "width": w,
            "distortion_model": "plumb_bob", "d": d, "k": k,
            "r": [1.0, 0, 0, 0, 1, 0, 0, 0, 1],
            "p": [100.0, 0, 50.0, 0, 0, 110.0, 40.0, 0, 0, 0, 1, 0],
            "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0,
                    "do_rectify": False}}


def bestpos(t: float, lat: float, lon: float, hgt: float) -> dict:
    return {"header": header(t, "gps"),
            "nov_header": {"message_name": "BESTPOS", "message_id": 42,
                           "message_type": 0, "sequence_number": 7,
                           "time_status": 180, "gps_week_number": 2300,
                           "gps_week_milliseconds": 123456},
            "sol_status": {"status": 0}, "pos_type": {"type": 56},
            "lat": lat, "lon": lon, "hgt": hgt, "undulation": -30.5,
            "datum_id": 61, "lat_stdev": 0.5, "lon_stdev": 0.25,
            "hgt_stdev": 1.0, "stn_id": "0"}


def tf(t: float, edges=None) -> dict:
    """edges: [(parent, child, (tx, ty, tz), (qx, qy, qz, qw))]."""
    edges = edges or [("map", "base_link", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))]
    return {"transforms": [{
        "header": header(t, parent), "child_frame_id": child,
        "transform": {"translation": dict(zip("xyz", tr)),
                      "rotation": dict(zip("xyzw", q))}}
        for parent, child, tr, q in edges]}


def navsatfix(t: float, lat: float, lon: float, alt: float, status: int = 2) -> dict:
    return {"header": header(t, "navsat_link"),
            "status": {"status": status, "service": 1},
            "latitude": lat, "longitude": lon, "altitude": alt,
            "position_covariance": [0.0] * 9, "position_covariance_type": 0}


def imu(t: float, quat) -> dict:
    z3 = {"x": 0.0, "y": 0.0, "z": 0.0}
    return {"header": header(t, "imu_link"),
            "orientation": dict(zip("xyzw", quat)),
            "orientation_covariance": [0.0] * 9,
            "angular_velocity": z3, "angular_velocity_covariance": [0.0] * 9,
            "linear_acceleration": z3, "linear_acceleration_covariance": [0.0] * 9}


def write_mcap(path, messages: List[Tuple[str, str, float, dict]],
               chunk_size: int = 4096, compression: str = "zstd") -> None:
    """messages: [(topic, ros2 type, log_time_s, msg_dict)]."""
    from mcap.writer import CompressionType
    from mcap_ros2.writer import Writer
    comp = {"zstd": CompressionType.ZSTD, "lz4": CompressionType.LZ4,
            "none": CompressionType.NONE}[compression]
    with open(path, "wb") as f:
        w = Writer(f, chunk_size=chunk_size, compression=comp)
        schemas = {}
        for topic, mtype, t, msg in messages:
            if mtype not in schemas:
                schemas[mtype] = w.register_msgdef(mtype, SCHEMAS[mtype])
            ns = int(round(t * 1e9))
            w.write_message(topic, schemas[mtype], msg, log_time=ns, publish_time=ns)
        w.finish()
