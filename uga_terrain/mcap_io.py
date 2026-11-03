"""Read ROS 2 MCAP recordings, including ones cut off before the footer."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, Iterator, Optional, Tuple


@dataclass
class ChannelInfo:
    topic: str
    schema: str
    count: Optional[int] = None


@dataclass
class McapInfo:
    path: str
    size: int
    channels: Dict[str, ChannelInfo] = field(default_factory=dict)
    start: Optional[float] = None
    end: Optional[float] = None
    indexed: bool = True

    @property
    def duration(self) -> Optional[float]:
        if self.start is None or self.end is None:
            return None
        return self.end - self.start

    def to_dict(self) -> dict:
        return {
            "path": self.path, "size": self.size, "indexed": self.indexed,
            "start": self.start, "end": self.end, "duration": self.duration,
            "topics": [{"topic": c.topic, "type": c.schema, "count": c.count}
                       for c in sorted(self.channels.values(), key=lambda c: c.topic)],
        }


def stamp_of(msg, fallback: float) -> float:
    h = getattr(msg, "header", None)
    st = getattr(h, "stamp", None)
    if st is None:
        return fallback
    t = float(getattr(st, "sec", 0)) + float(getattr(st, "nanosec", 0)) * 1e-9
    return t if t > 0 else fallback


def _summary(path: str):
    from mcap.reader import make_reader
    with open(path, "rb") as f:
        try:
            return make_reader(f).get_summary()
        except Exception:
            return None


def inspect(path: str, scan: bool = True) -> McapInfo:
    """Topics, message counts and time range.

    Indexed files answer from the summary. A file without one (recording cut
    off) is scanned when ``scan`` is true.
    """
    info = McapInfo(path=str(path), size=os.path.getsize(path))
    s = _summary(path)
    if s is not None and s.channels:
        counts = (s.statistics.channel_message_counts if s.statistics else {}) or {}
        for cid, ch in s.channels.items():
            sch = s.schemas.get(ch.schema_id)
            info.channels[ch.topic] = ChannelInfo(
                ch.topic, sch.name if sch else "", counts.get(cid))
        if s.statistics and s.statistics.message_count:
            info.start = s.statistics.message_start_time / 1e9
            info.end = s.statistics.message_end_time / 1e9
        return info
    info.indexed = False
    if not scan:
        return info
    from mcap.records import Channel, Message, Schema
    schemas: Dict[int, str] = {}
    chans: Dict[int, str] = {}
    for rec in _stream_records(path):
        if isinstance(rec, Schema):
            schemas[rec.id] = rec.name
        elif isinstance(rec, Channel):
            chans[rec.id] = rec.topic
            info.channels.setdefault(rec.topic, ChannelInfo(
                rec.topic, schemas.get(rec.schema_id, ""), 0))
        elif isinstance(rec, Message):
            t = rec.log_time / 1e9
            info.start = t if info.start is None else min(info.start, t)
            info.end = t if info.end is None else max(info.end, t)
            c = info.channels.get(chans.get(rec.channel_id, ""))
            if c is not None:
                c.count = (c.count or 0) + 1
    return info


def _stream_records(path: str) -> Iterator:
    from mcap.exceptions import McapError
    from mcap.stream_reader import StreamReader
    with open(path, "rb") as f:
        try:
            for rec in StreamReader(f, skip_magic=False, emit_chunks=False).records:
                yield rec
        except (McapError, EOFError, ValueError, OSError):
            return  # truncated recording: keep what was read


def iter_messages(path: str, topics: Iterable[str]) -> Iterator[Tuple[float, str, object]]:
    """Yield (log_time_s, topic, decoded ROS message) for ``topics``.

    Indexed files are read in log-time order. Unindexed ones in file order,
    which for a rosbag2 recording is close to time order.
    """
    from mcap_ros2.decoder import DecoderFactory
    wanted = set(topics)
    s = _summary(path)
    if s is not None and s.chunk_indexes:
        from mcap.reader import make_reader
        with open(path, "rb") as f:
            r = make_reader(f, decoder_factories=[DecoderFactory()])
            for _sch, ch, msg, dec in r.iter_decoded_messages(topics=sorted(wanted)):
                yield msg.log_time / 1e9, ch.topic, dec
        return
    from mcap.records import Channel, Message, Schema
    factory = DecoderFactory()
    schemas: Dict[int, object] = {}
    chans: Dict[int, object] = {}
    decoders: Dict[int, object] = {}
    for rec in _stream_records(path):
        if isinstance(rec, Schema):
            schemas[rec.id] = rec
        elif isinstance(rec, Channel):
            chans[rec.id] = rec
        elif isinstance(rec, Message):
            ch = chans.get(rec.channel_id)
            if ch is None or ch.topic not in wanted:
                continue
            dec = decoders.get(ch.id)
            if dec is None:
                dec = factory.decoder_for(ch.message_encoding, schemas.get(ch.schema_id))
                if dec is None:
                    continue
                decoders[ch.id] = dec
            try:
                yield rec.log_time / 1e9, ch.topic, dec(rec.data)
            except Exception:
                continue
