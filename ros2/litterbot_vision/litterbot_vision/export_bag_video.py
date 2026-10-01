"""Export a ROS2 bag compressed camera topic to an MP4 file."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export /camera/image/compressed from a ROS2 bag to MP4."
    )
    parser.add_argument("bag", help="Path to the ROS2 bag directory")
    parser.add_argument(
        "--topic",
        default="/camera/image/compressed",
        help="Compressed image topic to export",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output MP4 path; defaults to <bag>/camera.mp4",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Override output FPS; defaults to timestamp-derived rate",
    )
    return parser.parse_args()


def build_topic_type_map(reader: SequentialReader) -> dict[str, str]:
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def open_reader(bag_path: Path) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else bag_path / "camera.mp4"
    )

    reader = open_reader(bag_path)
    topic_types = build_topic_type_map(reader)
    msg_type_name = topic_types.get(args.topic)
    if not msg_type_name:
        raise RuntimeError(f"Topic not found in bag: {args.topic}")

    msg_type = get_message(msg_type_name)

    frames: list[np.ndarray] = []
    stamps: list[int] = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, msg_type)
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        frames.append(image)
        stamps.append(timestamp)

    if not frames:
        raise RuntimeError(f"No decodable frames found on {args.topic}")

    if args.fps > 0:
        fps = args.fps
    elif len(stamps) >= 2:
        duration_s = max((stamps[-1] - stamps[0]) / 1e9, 1e-3)
        fps = max(1.0, min(60.0, len(stamps) / duration_s))
    else:
        fps = 10.0

    height, width = frames[0].shape[:2]
    normalized_frames: list[np.ndarray] = []
    for frame in frames:
        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        normalized_frames.append(frame)

    if shutil.which("gst-launch-1.0"):
        with tempfile.TemporaryDirectory(prefix="litterbot_mp4_") as temp_dir:
            temp_path = Path(temp_dir)
            for idx, frame in enumerate(normalized_frames):
                frame_path = temp_path / f"frame_{idx:06d}.jpg"
                if not cv2.imwrite(str(frame_path), frame):
                    raise RuntimeError(f"Failed to write temp frame {frame_path}")

            gst_cmd = [
                "gst-launch-1.0",
                "-q",
                "multifilesrc",
                f"location={temp_path / 'frame_%06d.jpg'}",
                "index=0",
                f"caps=image/jpeg,framerate={int(round(max(1.0, fps) * 1000))}/1000",
                "!",
                "jpegdec",
                "!",
                "videoconvert",
                "!",
                "x264enc",
                "tune=zerolatency",
                "speed-preset=ultrafast",
                "key-int-max=30",
                "!",
                "h264parse",
                "!",
                "mp4mux",
                "faststart=true",
                "!",
                "filesink",
                f"location={output_path}",
            ]
            subprocess.run(gst_cmd, check=True)
    else:
        writer = cv2.VideoWriter(
            str(output_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {output_path}")
        for frame in normalized_frames:
            writer.write(frame)
        writer.release()

    print(f"Exported {len(frames)} frames to {output_path} at {fps:.2f} fps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
