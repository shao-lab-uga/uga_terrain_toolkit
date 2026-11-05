"""Copy part of an MCAP recording into a smaller file.

Usage:  python3 trim_mcap.py big.mcap clip.mcap [seconds=5] [start=0] [--terrain]
        python3 trim_mcap.py big.mcap clip.mcap --frames FIRST LAST [--terrain]
  seconds    length of the clip
  start      seconds from the beginning of the recording to start at
  --frames   cut by lidar frame number instead of seconds: keeps lidar frames
             FIRST to LAST (both included, counted from 0 like the terrain
             app), plus everything else recorded during them
  --terrain  keep only what the terrain app needs (lidar, OXTS, tf),
             which makes the clip much smaller
Needs:  pip install mcap zstandard lz4

Reads the input with its own small parser (it does not use mcap's reader),
then writes the output with mcap's Writer.
"""
import struct
import sys

from mcap.writer import CompressionType, Writer

OP_SCHEMA, OP_CHANNEL, OP_MESSAGE, OP_CHUNK = 0x03, 0x04, 0x05, 0x06
OP_FOOTER, OP_DATA_END = 0x02, 0x0F


def _str(b, i):
    n = struct.unpack_from("<I", b, i)[0]
    return b[i + 4:i + 4 + n].decode("utf-8", "replace"), i + 4 + n


def _decompress(raw, comp, size):
    if comp in ("", "none"):
        return raw
    if comp == "zstd":
        import zstandard
        return zstandard.ZstdDecompressor().decompress(raw, max_output_size=size)
    if comp == "lz4":
        import lz4.frame
        return lz4.frame.decompress(raw)
    raise SystemExit("unsupported chunk compression: %s" % comp)


def records(f):
    """Yield (opcode, body) for every record, looking inside chunks."""
    if f.read(8)[1:5] != b"MCAP":
        raise SystemExit("not an MCAP file")
    while True:
        head = f.read(9)
        if len(head) < 9:
            return
        op, n = struct.unpack("<BQ", head)
        body = f.read(n)
        if op in (OP_FOOTER, OP_DATA_END):
            return
        if op != OP_CHUNK:
            yield op, body
            continue
        start_ns = struct.unpack_from("<Q", body, 0)[0]
        yield "chunk_start", start_ns
        size = struct.unpack_from("<Q", body, 16)[0]
        comp, i = _str(body, 28)
        rlen = struct.unpack_from("<Q", body, i)[0]
        data = _decompress(body[i + 8:i + 8 + rlen], comp, size)
        j = 0
        while j + 9 <= len(data):
            op2, n2 = struct.unpack_from("<BQ", data, j)
            yield op2, data[j + 9:j + 9 + n2]
            j += 9 + n2


TERRAIN_TOPICS = ("/lidar/points", "/oxts/", "/tf_static", "/tf")


def _keep(topic, terrain):
    return not terrain or any(topic == t or (t.endswith("/") and topic.startswith(t))
                              for t in TERRAIN_TOPICS)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    terrain = "--terrain" in sys.argv
    frames = "--frames" in sys.argv
    src, dst = args[0], args[1]
    if frames:
        first, last = int(args[2]), int(args[3])
        secs, skip = None, None
    else:
        secs = float(args[2]) if len(args) > 2 else 5.0
        skip = float(args[3]) if len(args) > 3 else 0.0
    lidar_n = 0                      # lidar frames seen so far
    t0 = t1 = None                   # frame mode: kept time window [t0, t1)
    schemas, channels = {}, {}       # input id -> (fields)
    out_schema, out_chan = {}, {}    # input id -> output id
    start = None
    n = 0
    with open(src, "rb") as f, open(dst, "wb") as out:
        w = Writer(out, compression=CompressionType.ZSTD)
        w.start(profile="ros2", library="trim_mcap")
        for op, body in records(f):
            if op == "chunk_start":
                if frames:
                    if t1 is not None and body > t1:
                        break
                elif start is not None and body - start > (skip + secs) * 1e9:
                    break
                continue
            if op == OP_SCHEMA:
                sid = struct.unpack_from("<H", body, 0)[0]
                name, i = _str(body, 2)
                enc, i = _str(body, i)
                dlen = struct.unpack_from("<I", body, i)[0]
                schemas[sid] = (name, enc, body[i + 4:i + 4 + dlen])
            elif op == OP_CHANNEL:
                cid, sid = struct.unpack_from("<HH", body, 0)
                topic, i = _str(body, 4)
                enc, i = _str(body, i)
                channels[cid] = (sid, topic, enc)
            elif op == OP_MESSAGE:
                cid, seq, log_t, pub_t = struct.unpack_from("<HIQQ", body, 0)
                if start is None:
                    start = log_t
                if cid not in channels:
                    continue
                topic = channels[cid][1]
                if frames and topic == "/lidar/points":
                    idx = lidar_n
                    lidar_n += 1
                    if idx == first:
                        t0 = log_t
                    if idx == last + 1:
                        t1 = log_t
                    if not first <= idx <= last:
                        continue
                if not _keep(topic, terrain):
                    continue
                latched = topic == "/tf_static"  # sent once, at the start
                if frames:
                    if not latched and topic != "/lidar/points" and (
                            t0 is None or log_t < t0 or (t1 is not None and log_t >= t1)):
                        continue
                else:
                    rel = (log_t - start) / 1e9
                    if not latched and not (skip <= rel <= skip + secs):
                        continue
                if cid not in out_chan:
                    sid, topic, enc = channels[cid]
                    if sid and sid not in out_schema:
                        out_schema[sid] = w.register_schema(*schemas[sid])
                    out_chan[cid] = w.register_channel(
                        topic, enc, out_schema.get(sid, 0), {})
                w.add_message(out_chan[cid], log_t, body[22:], pub_t, seq)
                n += 1
        w.finish()
    if frames:
        got = max(0, min(last, lidar_n - 1) - first + 1)
        print("wrote %d messages (lidar frames %d to %d, %d frames) to %s"
              % (n, first, first + got - 1, got, dst))
        if got < last - first + 1:
            print("note: the recording only has %d lidar frames" % lidar_n)
    else:
        print("wrote %d messages (%.1f s from %.1f s) to %s" % (n, secs, skip, dst))


if __name__ == "__main__":
    main()
