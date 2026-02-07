import uvicorn
import sys
import os
from pathlib import Path

# Add the parent directory to Python path so backend is treated as a package
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

# Change working directory to backend
os.chdir(Path(__file__).parent)

if __name__ == "__main__":
    # Import and run the app
    from backend.src.main import app
    uvicorn.run(app, host="127.0.0.1", port=8000, reload=True)