# Quickstart Guide: TaskApp Professional Styling and Authentication

## Overview
This guide provides step-by-step instructions to set up, develop, and deploy the TaskApp application with professional styling and authentication.

## Prerequisites
- Node.js 20.x or higher
- Python 3.11 or higher
- PostgreSQL 12 or higher (or Neon Serverless account)
- Git
- npm or yarn package manager

## Local Development Setup

### 1. Clone and Initialize Repository
```bash
git clone <repository-url>
cd Full-Stack-Web-Application
```

### 2. Backend Setup
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your database URL and secrets
```

### 3. Frontend Setup
```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Set up environment variables
cp .env.example .env.local
# Edit .env.local with your backend URL and other configurations
```

### 4. Database Configuration
```bash
# In the backend directory
# Run database migrations
python -m alembic upgrade head

# Or if using SQLModel directly
python -c "from src.database.database import engine, create_db_and_tables; create_db_and_tables()"
```

## Environment Variables

### Backend (.env)
```env
DATABASE_URL=postgresql://username:password@localhost:5432/taskapp
BETTER_AUTH_SECRET=your-better-auth-secret
BETTER_AUTH_URL=http://localhost:3000
```

### Frontend (.env.local)
```env
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3000
```

## Running the Application

### 1. Start Backend Server
```bash
# From backend directory
uvicorn src.main:app --reload --port 8000
```

### 2. Start Frontend Development Server
```bash
# From frontend directory
npm run dev
```

### 3. Access the Application
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- Backend Docs: http://localhost:8000/docs

## API Endpoints

### Authentication
- `POST /api/auth/signup` - User registration
- `POST /api/auth/login` - User login
- `GET /api/auth/me` - Get current user
- `POST /api/auth/logout` - User logout

### Tasks
- `GET /api/{user_id}/tasks` - Get all user tasks
- `POST /api/{user_id}/tasks` - Create new task
- `PUT /api/{user_id}/tasks/{id}` - Update task
- `DELETE /api/{user_id}/tasks/{id}` - Delete task
- `PATCH /api/{user_id}/tasks/{id}/complete` - Toggle task completion

## Building for Production

### Backend
```bash
# Install production dependencies only
pip install -r requirements.txt

# Run migrations
python -m alembic upgrade head

# Start server
gunicorn src.main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:$PORT
```

### Frontend
```bash
# Build static assets
npm run build

# Start production server
npm start
```

## Testing the Functionality

1. Visit the homepage at http://localhost:3000
2. Click "Sign Up" to create an account
3. Log in with your credentials
4. Navigate to the dashboard to manage tasks
5. Test the dark/light mode toggle
6. Create, update, and delete tasks to verify full functionality

## Validation Checklist

- [ ] Homepage loads with professional styling
- [ ] Theme toggle works without flickering/hydration errors
- [ ] User registration works with validation
- [ ] User login works with validation
- [ ] Dashboard is accessible only when authenticated
- [ ] Task CRUD operations work properly
- [ ] Date handling is correct
- [ ] All API endpoints return expected responses
- [ ] Error handling works properly
- [ ] Responsive design works on all screen sizes