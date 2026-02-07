# Quickstart Guide: TASKAPP Professional Todo Application

## Overview
This guide provides step-by-step instructions to set up and run the TASKAPP professional todo application locally. The application consists of a Next.js frontend and a FastAPI backend with Neon PostgreSQL database.

## Prerequisites

Before starting, ensure you have the following installed:

- **Node.js** (v20 or higher)
- **Python** (v3.11 or higher)
- **uv** (Python package manager) - install with `pip install uv`
- **Git**
- **A Neon account** with a project created

## Step 1: Clone and Navigate

```bash
# Clone the repository
git clone <repository-url>
cd Full-Stack-Web-Application

# Or if you're already in the project directory, skip cloning
```

## Step 2: Set Up Environment Variables

Create a `.env` file in the root directory with the following variables:

```bash
# .env
BETTER_AUTH_SECRET="generate-a-secure-64-character-secret-here"
DATABASE_URL="postgresql+asyncpg://username:password@ep-projectname-123456.ap-southeast-1.aws.neon.tech/dbname?sslmode=require"

# For local development
DATABASE_URL_DEV="postgresql+asyncpg://username:password@ep-projectname-dev-abcdef.ap-southeast-1.aws.neon.tech/dbname?sslmode=require"
TEST_DATABASE_URL="postgresql://username:password@ep-projectname-test-999999.ap-southeast-1.aws.neon.tech/testdb?sslmode=require"

# Better Auth configuration
NEXT_PUBLIC_BETTER_AUTH_URL="http://localhost:3000"
BETTER_AUTH_URL="http://localhost:3000"
BETTER_AUTH_TRUST_HOST="true"
```

**Important**:
- Generate a secure random string for `BETTER_AUTH_SECRET` (at least 32 bytes/64 characters)
- Replace the database URL with your actual Neon database connection string
- For `DATABASE_URL`, use your production/staging database
- For local development, you might want to create a separate Neon branch

## Step 3: Set Up Backend (FastAPI + SQLModel)

1. Navigate to the backend directory:
```bash
cd backend
```

2. Install Python dependencies using uv:
```bash
uv sync
# Or if you want to install individually:
# uv add fastapi uvicorn sqlmodel alembic pyjwt python-dotenv pydantic[email]
```

3. Set up the database and run initial migration:
```bash
# Initialize the database (if not already done)
uv run alembic upgrade head
```

4. Verify the installation by running tests:
```bash
uv run pytest
```

5. Run the backend server:
```bash
uv run uvicorn main:app --reload --port 8000
```

The backend will be available at `http://localhost:8000`.

## Step 4: Set Up Frontend (Next.js)

1. Navigate to the frontend directory:
```bash
cd ../frontend  # From backend directory, or directly to frontend
```

2. Install JavaScript dependencies:
```bash
npm install
```

3. Run development server:
```bash
npm run dev
```

The frontend will be available at `http://localhost:3000`.

## Step 5: Running Both Services Together

For the best development experience, you'll want both services running simultaneously:

### Option A: Using Concurrently
1. In the root directory, install concurrently:
```bash
npm install -g concurrently
```

2. Run both services:
```bash
concurrently "cd backend && uv run uvicorn main:app --reload --port 8000" "cd frontend && npm run dev"
```

### Option B: Using Docker Compose (if available)
```bash
docker-compose up --build
```

### Option C: Separate Terminals
Open two terminal windows/tabs:
- Terminal 1: Run the backend (`cd backend && uv run uvicorn main:app --reload --port 8000`)
- Terminal 2: Run the frontend (`cd frontend && npm run dev`)

## Step 6: Initial Setup Verification

1. Visit `http://localhost:3000` in your browser
2. You should see the landing page with "Master Your Day with TASKAPP"
3. Try navigating to sign-up page and creating an account
4. After signing up, verify you can access the dashboard
5. Create a test task to ensure the full stack is working

## Common Issues and Solutions

### Issue: Database Connection Error
**Solution**: Verify your Neon database connection string in `.env` and ensure the database is active in the Neon console.

### Issue: Better Auth Not Working
**Solution**:
- Ensure `BETTER_AUTH_SECRET` is set correctly in both frontend and backend
- Check that the URLs are configured properly
- Verify the backend is running on port 8000

### Issue: Theme Switching Not Persistent
**Solution**: This may occur in development if next-themes isn't properly initialized. Ensure the ThemeProvider is wrapping the application correctly.

### Issue: API Requests Failing
**Solution**:
- Verify the backend is running and accessible
- Check that the API endpoints are configured correctly in the frontend
- Ensure the JWT token is being properly attached to requests

## Development Commands

### Backend Commands
```bash
# Run tests
uv run pytest

# Run with auto-reload
uv run uvicorn main:app --reload

# Generate new migration after model changes
cd backend
uv run alembic revision --autogenerate -m "describe your changes"
uv run alembic upgrade head

# Check code quality
uv run ruff check .
uv run mypy .
```

### Frontend Commands
```bash
# Development server
npm run dev

# Build for production
npm run build

# Run tests
npm run test

# Lint code
npm run lint

# Type checking
npx tsc --noEmit
```

## Next Steps

1. **Customize the UI**: Modify components in `frontend/components/` to match your branding
2. **Extend the API**: Add new endpoints in `backend/routes/`
3. **Add features**: Implement additional functionality based on your requirements
4. **Configure deployment**: Set up your production environment on Vercel (frontend) and Render/Railway (backend)

## Troubleshooting

If you encounter issues:

1. Check that all environment variables are set correctly
2. Verify that your Neon database is accessible
3. Confirm both frontend and backend are running on their respective ports
4. Look at browser console and server logs for specific error messages
5. Ensure your BETTER_AUTH_SECRET is consistent across environments

For more detailed troubleshooting, refer to the individual README files in the frontend and backend directories.