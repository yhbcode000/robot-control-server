#!/usr/bin/env python3
"""
Main entry point for the Robot Control Flask Server
"""

import argparse
import os
import sys

from src.flask_server import create_app


def main():
    parser = argparse.ArgumentParser(description="Robot Control Flask Server")
    parser.add_argument(
        "--host", default="localhost", help="Server host (default: localhost)"
    )
    parser.add_argument(
        "--port", type=int, default=5000, help="Server port (default: 5000)"
    )
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")

    args = parser.parse_args()

    print("=" * 50)
    print("Robot Control Flask Server")
    print("=" * 50)
    print(f"Host: {args.host}")
    print(f"Port: {args.port}")
    print(f"Debug: {args.debug}")
    print("=" * 50)

    # Create and run server
    server = create_app()
    server.host = args.host
    server.port = args.port

    try:
        server.run(debug=args.debug)
    except KeyboardInterrupt:
        print("\nShutdown requested by user...")
    except Exception as e:
        print(f"Server error: {e}")
        sys.exit(1)
    finally:
        server.shutdown()
        print("Server shutdown complete.")


if __name__ == "__main__":
    main()
