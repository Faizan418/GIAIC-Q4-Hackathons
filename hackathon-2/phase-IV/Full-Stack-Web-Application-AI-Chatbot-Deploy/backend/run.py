import os
import subprocess
import sys
from pathlib import Path

# Set the PYTHONPATH to include the current directory for proper imports
backend_dir = Path(__file__).parent
env = os.environ.copy()
env['PYTHONPATH'] = str(backend_dir) + os.pathsep + env.get('PYTHONPATH', '')

# Change working directory to backend
os.chdir(backend_dir)

if __name__ == "__main__":
    # Run uvicorn with the proper PYTHONPATH
    cmd = [
        sys.executable, "-c",
        "import uvicorn; from backend.src.main import app; uvicorn.run(app, host='127.0.0.1', port=8000, reload=True)"
    ]

    # Set the PYTHONPATH in the environment
    env = os.environ.copy()
    env['PYTHONPATH'] = str(backend_dir)

    # Execute the command
    subprocess.run(cmd, env=env, cwd=backend_dir)