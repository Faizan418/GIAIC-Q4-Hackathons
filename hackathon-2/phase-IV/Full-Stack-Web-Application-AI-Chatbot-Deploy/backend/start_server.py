#!/usr/bin/env python3
"""
Start the backend server with the correct Python path configuration.
"""
import os
import sys
from pathlib import Path

# Add the current directory to Python path to make imports work
current_dir = Path(__file__).parent
os.environ['PYTHONPATH'] = str(current_dir)

if __name__ == "__main__":
    import subprocess
    import sys

    # Set up the environment with PYTHONPATH
    env = os.environ.copy()
    env['PYTHONPATH'] = str(current_dir)

    # Run uvicorn with the application as a string to enable reload
    cmd = [
        sys.executable, "-m", "uvicorn",
        "src.main:app",
        "--host", "127.0.0.1",
        "--port", "8000",
        "--reload"
    ]

    print("Starting server on http://127.0.0.1:8000")
    subprocess.run(cmd, env=env)