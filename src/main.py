#!/usr/bin/env python3
"""
Main entry point for the Robot Control Flask Server
"""

import argparse
import os
import sys

from src.flask_server import create_app
from src.mujoco_controller import MuJoCoController


def check_mujoco_model():
    """Check if MuJoCo model file exists and return the path"""
    model_path = "e:\\桌面\\SoftRob\\Mujoco_0610\\robot\\model\\spiral_3D.xml"

    if os.path.exists(model_path):
        return model_path

    # Try alternative paths
    possible_paths = [
        "model\\spiral_3D.xml",
        "..\\model\\spiral_3D.xml",
        "model/spiral_3D.xml",
        "../model/spiral_3D.xml",
        "robot\\model\\spiral_3D.xml",
    ]

    for path in possible_paths:
        if os.path.exists(path):
            print(f"✅ Found model at: {path}")
            return path

    print("❌ No MuJoCo model file found. Please ensure spiral_3D.xml exists.")
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Robot Control Flask Server with MuJoCo Integration"
    )
    parser.add_argument(
        "--host", default="localhost", help="Server host (default: localhost)"
    )
    parser.add_argument(
        "--port", type=int, default=5000, help="Server port (default: 5000)"
    )
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument(
        "--no-viewer", action="store_true", help="Disable MuJoCo viewer (headless mode)"
    )
    parser.add_argument("--model", help="Path to MuJoCo model file (optional)")

    args = parser.parse_args()

    print("=" * 60)
    print("🤖 Robot Control Flask Server with MuJoCo")
    print("=" * 60)
    print(f"Host: {args.host}")
    print(f"Port: {args.port}")
    print(f"Debug: {args.debug}")
    print(f"Viewer: {'Disabled' if args.no_viewer else 'Enabled'}")
    print("=" * 60)

    # Check MuJoCo model availability
    model_path = args.model or check_mujoco_model()
    if not model_path:
        print("❌ Cannot start server without MuJoCo model file")
        sys.exit(1)

    print(f"🎯 Using MuJoCo model: {model_path}")

    # Create and configure server
    server = create_app(model_path=model_path, enable_viewer=not args.no_viewer)
    server.host = args.host
    server.port = args.port

    try:
        print("🚀 Starting Robot Control Server...")
        server.run(debug=args.debug)
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    except Exception as e:
        print(f"💥 Server error: {e}")
        sys.exit(1)
    finally:
        print("🧹 Shutting down server...")
        server.shutdown()
        print("✅ Server shutdown complete.")


if __name__ == "__main__":
    main()
