#!/usr/bin/env python3
"""
Rosbridge Relay - Bridges /oak/flow/twist from Pi5 to Docker simulator

Connects to Pi5's rosbridge, subscribes to /oak/flow/twist,
and republishes to Docker's rosbridge for PX4 to receive.
"""

import asyncio
import json
import websockets

PI5_ROSBRIDGE = "ws://192.168.7.31:9090"
DOCKER_ROSBRIDGE = "ws://localhost:9090"

async def relay():
    print(f"Connecting to Pi5: {PI5_ROSBRIDGE}")
    print(f"Connecting to Docker: {DOCKER_ROSBRIDGE}")

    async with websockets.connect(PI5_ROSBRIDGE) as pi5_ws, \
               websockets.connect(DOCKER_ROSBRIDGE) as docker_ws:

        # Subscribe to /oak/flow/twist on Pi5
        subscribe_msg = json.dumps({
            "op": "subscribe",
            "topic": "/oak/flow/twist",
            "type": "geometry_msgs/msg/TwistWithCovarianceStamped"
        })
        await pi5_ws.send(subscribe_msg)
        print("Subscribed to /oak/flow/twist on Pi5")

        # Advertise on Docker's rosbridge
        advertise_msg = json.dumps({
            "op": "advertise",
            "topic": "/oak/flow/twist",
            "type": "geometry_msgs/msg/TwistWithCovarianceStamped"
        })
        await docker_ws.send(advertise_msg)
        print("Advertising /oak/flow/twist on Docker")

        msg_count = 0
        async for message in pi5_ws:
            data = json.loads(message)
            if data.get("op") == "publish" and data.get("topic") == "/oak/flow/twist":
                # Republish to Docker
                publish_msg = json.dumps({
                    "op": "publish",
                    "topic": "/oak/flow/twist",
                    "msg": data["msg"]
                })
                await docker_ws.send(publish_msg)
                msg_count += 1
                if msg_count % 30 == 0:  # Print every ~1 second at 30Hz
                    twist = data["msg"]["twist"]["twist"]["linear"]
                    print(f"Relayed {msg_count} msgs | x={twist['x']:.3f} y={twist['y']:.3f}")

if __name__ == "__main__":
    print("=== Rosbridge Relay ===")
    print("Bridging /oak/flow/twist from Pi5 to Docker simulator")
    print("Press Ctrl+C to stop\n")

    try:
        asyncio.run(relay())
    except KeyboardInterrupt:
        print("\nStopped")
    except Exception as e:
        print(f"Error: {e}")
