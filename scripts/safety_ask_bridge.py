#!/usr/bin/env python3
"""Local ROS bridge for the SafetyErrorAsk event shown in Groot."""

import argparse
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

TOPIC = "/sura_bt_runner/safety_ask"
WORKSPACE = Path(os.environ.get("SURA_WS_META", Path(__file__).resolve().parents[3]))


def listen():
    rclpy.init()
    node = rclpy.create_node("sura_safety_ask_listener")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(String, TOPIC, lambda msg: print(msg.data or "{}", flush=True), qos)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def resolve(action):
    rclpy.init()
    node = rclpy.create_node("sura_safety_ask_client")
    service = "/sura_bt_runner/resume_safety_ask" if action == "resume" else "/sura_bt_runner/abort_safety_ask"
    try:
        client = node.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Service unavailable: {service}")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Service timed out: {service}")
        print(json.dumps({"success": bool(future.result().success), "message": future.result().message}), flush=True)
        if not future.result().success:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def image_topics(robot):
    rclpy.init()
    node = rclpy.create_node("sura_safety_camera_discovery")
    try:
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        prefix = "/" + robot.strip("/") + "/"
        return sorted(
            topic for topic, types in node.get_topic_names_and_types()
            if topic.startswith(prefix) and "sensor_msgs/msg/Image" in types
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def capture(robot, requested_topic=""):
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    topics = image_topics(robot)
    if not topics:
        raise RuntimeError(f"No image topics found for {robot}")
    if requested_topic and requested_topic not in topics:
        raise ValueError(f"Camera topic is not available for {robot}: {requested_topic}")
    topic = requested_topic or topics[0]
    rclpy.init()
    node = rclpy.create_node("sura_safety_photo_capture")
    images = []
    subscription = node.create_subscription(Image, topic, lambda msg: images.append(msg), 10)
    try:
        end = time.monotonic() + 8.0
        while not images and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not images:
            raise RuntimeError(f"No image received from {topic}")
        frame = CvBridge().imgmsg_to_cv2(images[0], desired_encoding="passthrough")
        if frame.dtype in (np.float32, np.float64):
            valid = np.isfinite(frame) & (frame > 0)
            if not valid.any():
                raise RuntimeError("Depth image has no valid pixels")
            low, high = np.percentile(frame[valid], [5, 95])
            frame = np.where(valid, np.clip((frame - low) * 255 / max(high - low, 0.01), 0, 255), 0).astype(np.uint8)
        elif frame.dtype == np.uint16:
            frame = cv2.convertScaleAbs(frame, alpha=255.0 / max(float(frame.max()), 1.0))
        elif images[0].encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif images[0].encoding == "rgba8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGRA)
        output = WORKSPACE / "photos"
        output.mkdir(parents=True, exist_ok=True)
        path = output / f"safety_{robot}_{int(time.time() * 1000)}.png"
        if not cv2.imwrite(str(path), frame):
            raise RuntimeError(f"Could not write image to {path}")
        print(json.dumps({"photo": str(path), "topic": topic}), flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["listen", "cameras", "capture", "resume", "abort"])
    parser.add_argument("--robot", default="")
    parser.add_argument("--topic", default="")
    args = parser.parse_args()
    try:
        if args.command == "listen":
            listen()
        elif args.command in {"cameras", "capture"}:
            if not args.robot or "/" in args.robot:
                raise ValueError("--robot must be a robot namespace")
            if args.command == "cameras":
                print(json.dumps({"topics": image_topics(args.robot)}), flush=True)
            else:
                capture(args.robot, args.topic)
        else:
            resolve(args.command)
    except Exception as error:
        print(json.dumps({"error": str(error)}), flush=True)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
