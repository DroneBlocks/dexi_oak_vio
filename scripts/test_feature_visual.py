#!/usr/bin/env python3
"""
Oak-D Lite Feature Tracker Visual Test

Shows camera feed with tracked features overlaid.
Requires OpenCV for display.

Install:
    pip install depthai numpy opencv-python

Usage:
    python test_feature_visual.py
"""

import depthai as dai
import cv2
import numpy as np
from collections import defaultdict


def main():
    print("Oak-D Lite Feature Tracker - Visual Mode")
    print("Press 'q' to quit")
    print()

    # Create pipeline
    pipeline = dai.Pipeline()

    # Mono camera
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setCamera("left")
    mono_left.setFps(30)

    # Feature tracker
    feature_tracker = pipeline.create(dai.node.FeatureTracker)
    feature_tracker.setHardwareResources(1, 2)

    # Outputs
    xout_left = pipeline.create(dai.node.XLinkOut)
    xout_left.setStreamName("left")

    xout_features = pipeline.create(dai.node.XLinkOut)
    xout_features.setStreamName("features")

    # Links
    mono_left.out.link(feature_tracker.inputImage)
    mono_left.out.link(xout_left.input)
    feature_tracker.outputFeatures.link(xout_features.input)

    # Track feature history for trails
    feature_history = defaultdict(lambda: [])
    max_history = 10

    print("Connecting...")

    with dai.Device(pipeline) as device:
        print(f"Device: {device.getDeviceName()}")
        print(f"USB: {device.getUsbSpeed().name}")
        print()

        q_left = device.getOutputQueue("left", maxSize=4, blocking=False)
        q_features = device.getOutputQueue("features", maxSize=4, blocking=False)

        prev_features = {}
        flow_vectors = []

        while True:
            img_data = q_left.tryGet()
            features_data = q_features.tryGet()

            if img_data is not None:
                frame = img_data.getCvFrame()
                frame_color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

                if features_data is not None:
                    features = features_data.trackedFeatures

                    # Current features
                    current = {}
                    for f in features:
                        current[f.id] = (int(f.position.x), int(f.position.y))
                        feature_history[f.id].append((f.position.x, f.position.y))

                        # Limit history
                        if len(feature_history[f.id]) > max_history:
                            feature_history[f.id].pop(0)

                    # Draw feature trails
                    for fid, history in feature_history.items():
                        if len(history) > 1:
                            for i in range(1, len(history)):
                                alpha = i / len(history)
                                color = (0, int(255 * alpha), int(255 * (1 - alpha)))
                                pt1 = (int(history[i-1][0]), int(history[i-1][1]))
                                pt2 = (int(history[i][0]), int(history[i][1]))
                                cv2.line(frame_color, pt1, pt2, color, 1)

                    # Draw current features
                    for fid, (x, y) in current.items():
                        cv2.circle(frame_color, (x, y), 4, (0, 255, 0), -1)

                    # Compute flow if we have previous frame
                    if prev_features:
                        flow_vectors = []
                        for fid, (x, y) in current.items():
                            if fid in prev_features:
                                px, py = prev_features[fid]
                                flow_vectors.append((x - px, y - py))

                        if len(flow_vectors) >= 10:
                            flow = np.array(flow_vectors)
                            median_dx = np.median(flow[:, 0])
                            median_dy = np.median(flow[:, 1])

                            # Draw overall flow vector
                            cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
                            scale = 5
                            end_x = int(cx + median_dx * scale)
                            end_y = int(cy + median_dy * scale)
                            cv2.arrowedLine(frame_color, (cx, cy), (end_x, end_y),
                                          (255, 0, 255), 3, tipLength=0.3)

                    prev_features = current

                    # Stats overlay
                    cv2.putText(frame_color, f"Features: {len(features)}",
                               (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(frame_color, f"Matched: {len(flow_vectors)}",
                               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    if len(flow_vectors) >= 10:
                        flow = np.array(flow_vectors)
                        cv2.putText(frame_color, f"Flow: ({np.median(flow[:,0]):+.1f}, {np.median(flow[:,1]):+.1f})",
                                   (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Clean up old features
                current_ids = set(current.keys()) if features_data else set()
                for fid in list(feature_history.keys()):
                    if fid not in current_ids:
                        del feature_history[fid]

                cv2.imshow("Oak-D Feature Tracker", frame_color)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
