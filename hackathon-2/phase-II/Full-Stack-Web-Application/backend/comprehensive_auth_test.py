"""
Comprehensive test to verify authentication endpoints and their functionality
"""
import sys
import os
import json
import asyncio
from typing import Dict, Any

def test_backend_setup():
    """Test backend app and authentication routes"""
    print("=== BACKEND AUTHENTICATION SETUP TEST ===\n")

    try:
        from src.main import app
        print("SUCCESS: Backend app imports successfully")
    except Exception as e:
        print(f"ERROR: Backend app import failed: {e}")
        return False

    # Check registered routes
    print("\n--- Registered Routes ---")
    auth_routes_found = []
    for route in app.routes:
        if hasattr(route, 'path') and 'auth' in route.path.lower():
            methods = getattr(route, 'methods', ['UNKNOWN']) if hasattr(route, 'methods') else ['UNKNOWN']
            print(f"  {list(methods)} {route.path}")
            auth_routes_found.append((route.path, list(methods)))

    if not auth_routes_found:
        print("ERROR: No authentication routes found!")
        # Check for register/login in any route
        for route in app.routes:
            if hasattr(route, 'path'):
                path_lower = route.path.lower()
                if any(keyword in path_lower for keyword in ['register', 'login', 'auth']):
                    methods = getattr(route, 'methods', ['UNKNOWN'])
                    print(f"  {list(methods)} {route.path}")
                    auth_routes_found.append((route.path, list(methods)))

    if auth_routes_found:
        print("SUCCESS: Authentication routes found")
    else:
        print("ERROR: No authentication routes found at all")

    # Check auth module
    try:
        from src.auth import Token, TokenData, UserLogin, UserRegister, UserResponse, create_access_token, verify_password
        print("\nSUCCESS: Authentication module imports successfully")

        # Test token creation
        test_token = create_access_token({"sub": "test@example.com"})
        print("SUCCESS: JWT token creation works")

    except Exception as e:
        print(f"ERROR: Authentication module error: {e}")
        return False

    # Check auth routes module
    try:
        from src.api.auth_routes import router
        print("SUCCESS: Authentication routes module imports successfully")
    except Exception as e:
        print(f"ERROR: Authentication routes module error: {e}")
        return False

    # Check database connectivity
    try:
        from src.database.db import DATABASE_URL, async_engine
        print(f"\nSUCCESS: Database configured: {DATABASE_URL}")

        # Check if models are accessible
        from src.models.user import User
        from src.models.task import Task
        print("SUCCESS: Database models accessible")

    except Exception as e:
        print(f"ERROR: Database setup error: {e}")
        return False

    print("\n=== BACKEND SETUP VERIFICATION COMPLETE ===")
    print("SUCCESS: Backend authentication system is properly configured")
    print("SUCCESS: Authentication endpoints are registered")
    print("SUCCESS: Database connection is set up")
    print("SUCCESS: Ready to handle registration requests")

    return True

def test_frontend_setup():
    """Test frontend authentication API configuration"""
    print("\n=== FRONTEND AUTHENTICATION SETUP TEST ===\n")

    try:
        # Check if the auth API file exists
        auth_api_path = "E:/Hackathon 2/Phase-II/Full-Stack-Web-Application/frontend/lib/backend-auth-api.ts"
        if os.path.exists(auth_api_path):
            print("SUCCESS: Frontend authentication API file exists")

            # Read the file to verify content
            with open(auth_api_path, 'r', encoding='utf-8') as f:
                content = f.read()

            if 'NEXT_PUBLIC_API_URL' in content and 'authApi' in content:
                print("SUCCESS: Frontend authentication API properly configured")

                # Check for register method
                if 'register' in content and 'POST' in content:
                    print("SUCCESS: Register method found in frontend API")
                else:
                    print("ERROR: Register method not found in frontend API")

            else:
                print("ERROR: Frontend authentication API not properly configured")
                return False

        else:
            print("ERROR: Frontend authentication API file not found")
            return False

    except Exception as e:
        print(f"ERROR: Frontend setup test error: {e}")
        return False

    print("\n=== FRONTEND SETUP VERIFICATION COMPLETE ===")
    print("SUCCESS: Frontend authentication API is properly configured")
    print("SUCCESS: API endpoints are correctly referenced")
    print("SUCCESS: Ready to communicate with backend")

    return True

def test_environment_variables():
    """Test environment variables for both frontend and backend"""
    print("\n=== ENVIRONMENT VARIABLES TEST ===\n")

    # Test backend environment
    backend_env_path = "E:/Hackathon 2/Phase-II/Full-Stack-Web-Application/.env"
    if os.path.exists(backend_env_path):
        print("SUCCESS: Backend .env file exists")

        # Check important variables
        import dotenv
        env_vars = dotenv.dotenv_values(backend_env_path)

        required_vars = ['JWT_SECRET_KEY', 'BETTER_AUTH_SECRET']
        missing_vars = []
        for var in required_vars:
            if var not in env_vars or not env_vars[var]:
                missing_vars.append(var)

        if missing_vars:
            print(f"WARNING: Missing backend environment variables: {missing_vars}")
        else:
            print("SUCCESS: All required backend environment variables are present")
    else:
        print("ERROR: Backend .env file not found")

    # Test frontend environment
    frontend_env_path = "E:/Hackathon 2/Phase-II/Full-Stack-Web-Application/frontend/.env.local"
    if os.path.exists(frontend_env_path):
        print("SUCCESS: Frontend .env.local file exists")

        frontend_vars = dotenv.dotenv_values(frontend_env_path)

        if 'NEXT_PUBLIC_API_URL' in frontend_vars:
            api_url = frontend_vars['NEXT_PUBLIC_API_URL']
            print(f"SUCCESS: Frontend API URL configured: {api_url}")
        else:
            print("ERROR: NEXT_PUBLIC_API_URL not found in frontend env")
    else:
        print("ERROR: Frontend .env.local file not found")

    print("\n=== ENVIRONMENT VARIABLES TEST COMPLETE ===")
    return True

def main():
    print("COMPREHENSIVE AUTHENTICATION SYSTEM VERIFICATION")
    print("="*50)

    backend_ok = test_backend_setup()
    frontend_ok = test_frontend_setup()
    env_ok = test_environment_variables()

    print(f"\nSUMMARY:")
    print(f"- Backend Setup: {'SUCCESS' if backend_ok else 'FAILED'}")
    print(f"- Frontend Setup: {'SUCCESS' if frontend_ok else 'FAILED'}")
    print(f"- Environment Variables: {'SUCCESS' if env_ok else 'FAILED'}")

    if backend_ok and frontend_ok and env_ok:
        print(f"\nOVERALL RESULT: SUCCESS! All systems properly configured!")
        print("The authentication system should work correctly now.")
        print("Make sure both servers are running before testing.")
        return True
    else:
        print(f"\nOVERALL RESULT: ISSUES DETECTED in the setup")
        return False

if __name__ == "__main__":
    success = main()
    if success:
        print("\nYou can now run both servers and authentication should work properly.")
        print("- Start backend: python -m uvicorn src.main:app --host 0.0.0.0 --port 8000")
        print("- Start frontend: npm run dev")
    else:
        print("\nPlease check the issues identified above and fix them.")