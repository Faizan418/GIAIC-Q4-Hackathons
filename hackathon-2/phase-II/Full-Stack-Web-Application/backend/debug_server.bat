@echo off
echo Starting backend server...
cd /d "E:\Hackathon 2\Phase-II\Full-Stack-Web-Application\backend"
echo Current directory: %cd%
dir src 2>nul || echo ERROR: src directory not found
dir src\main.py 2>nul || echo ERROR: src\main.py not found
python --version
echo Attempting to start server...
python -c "from src.main import app; print('App imported successfully')"
echo Trying to start uvicorn...
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --timeout-keep-alive 30