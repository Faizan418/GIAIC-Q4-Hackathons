# Quickstart Guide: Full-Stack Todo App Integration

**Feature**: Full-Stack Todo App Integration
**Date**: 2026-01-12
**Status**: Complete

## Prerequisites

- Node.js 18+ and npm
- Python 3.10+
- uv (Astral's Python package manager)
- PostgreSQL (or Neon Serverless PostgreSQL for cloud deployment)
- Git

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd Full-Stack-Web-Application
```

### 2. Install uv (if not already installed)
```bash
pip install uv
# Or using the official installation script
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 3. Set Up Backend Dependencies with uv
```bash
# From the repository root
uv init
uv add fastapi uvicorn sqlmodel psycopg2-binary pyjwt python-dotenv pydantic[email]

# Or if using workspace mode, create/edit root pyproject.toml:
cat > pyproject.toml << EOF
[project]
name = "todo-monorepo"
version = "0.1.0"
requires-python = ">=3.10"

[tool.uv.workspace]
members = ["backend"]
EOF

# Create backend pyproject.toml
mkdir -p backend
cat > backend/pyproject.toml << EOF
[project]
name = "todo-backend"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = [
    "fastapi>=0.115.0",
    "uvicorn>=0.32.0",
    "sqlmodel>=0.0.22",
    "pyjwt>=2.10.1",
    "python-dotenv>=1.0.1",
    "asyncpg>=0.30.0",
    "psycopg2-binary>=2.10.3",
    "slowapi>=0.1.9",
    "httpx>=0.28.0",
    "aiosqlite>=0.20.0"
]
EOF

# Sync dependencies
uv sync
```

### 4. Configure Environment Variables
Create a root `.env` file with your secrets:
```bash
# .env (at repository root)
BETTER_AUTH_SECRET=your-very-long-random-secret-here
DATABASE_URL=postgresql+asyncpg://username:password@ep-project-name.region.aws.neon.tech/dbname?sslmode=require
NEXT_PUBLIC_API_URL=http://localhost:8000
```

Also create a `.env.example` file:
```bash
# .env.example (at repository root)
# Authentication (must be same for frontend & backend)
BETTER_AUTH_SECRET=your-very-long-random-secret-here

# Database (Neon serverless PostgreSQL)
DATABASE_URL=postgresql+asyncpg://username:password@ep-project-name.region.aws.neon.tech/dbname?sslmode=require

# Frontend API URL
NEXT_PUBLIC_API_URL=http://localhost:8000
```

### 5. Set Up Frontend Dependencies
```bash
cd frontend
npm install
cd ..
```

### 6. Create Missing Backend Folders and Files
```bash
# Create backend directories if they don't exist
mkdir -p backend/alembic/versions
mkdir -p backend/tests

# Create basic test file
cat > backend/tests/__init__.py << EOF
"""Tests for the backend API"""
EOF

cat > backend/tests/test_tasks.py << EOF
"""Basic tests for task endpoints"""
import pytest
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}
EOF

# Create alembic files if needed
touch backend/alembic/__init__.py
```

### 7. Update Frontend API Client
Update `frontend/lib/api.ts` with the real API implementation:

```typescript
// frontend/lib/api.ts
import { getSession } from "@better-auth/react";

const API_BASE = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000";

export interface Task {
  id: number;
  user_id: string;
  title: string;
  description?: string;
  completed: boolean;
  priority?: 'high' | 'medium' | 'low';
  tags: string[];
  due_date?: string; // ISO date string
  recurrence?: 'none' | 'daily' | 'weekly';
  created_at: string; // ISO date string
  updated_at: string; // ISO date string
}

export interface CreateTask {
  title: string;
  description?: string;
  priority?: 'high' | 'medium' | 'low';
  tags?: string[];
  due_date?: string; // ISO date string
  recurrence?: 'none' | 'daily' | 'weekly';
}

export interface UpdateTask extends Partial<CreateTask> {
  completed?: boolean;
}

export const api = {
  async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    const session = await getSession();
    const token = session?.user?.accessToken;

    const res = await fetch(`${API_BASE}${endpoint}`, {
      ...options,
      headers: {
        "Content-Type": "application/json",
        Authorization: token ? `Bearer ${token}` : "",
        ...options.headers,
      },
    });

    if (!res.ok) {
      const error = await res.json().catch(() => ({}));
      throw new Error(error.detail || res.statusText);
    }

    return res.json();
  },

  getTasks: (userId: string, params = "") =>
    api.request<Task[]>(`/api/${userId}/tasks${params}`),

  createTask: (userId: string, data: CreateTask) =>
    api.request<Task>(`/api/${userId}/tasks`, {
      method: "POST",
      body: JSON.stringify(data)
    }),

  getTask: (userId: string, taskId: number) =>
    api.request<{data: {task: Task}; message: string}>(`/api/${userId}/tasks/${taskId}`),

  updateTask: (userId: string, taskId: number, data: UpdateTask) =>
    api.request<{data: {task: Task}; message: string}>(`/api/${userId}/tasks/${taskId}`, {
      method: "PUT",
      body: JSON.stringify(data)
    }),

  deleteTask: (userId: string, taskId: number) =>
    api.request<{message: string}>(`/api/${userId}/tasks/${taskId}`, {
      method: "DELETE"
    }),

  toggleTaskCompletion: (userId: string, taskId: number, completed?: boolean) =>
    api.request<{data: {task: Task}; message: string}>(`/api/${userId}/tasks/${taskId}/complete`, {
      method: "PATCH",
      body: JSON.stringify({ completed })
    }),
};
```

### 8. Update Better Auth Configuration
Update your Better Auth configuration to use the same secret:

```typescript
// In your auth config file (e.g., pages/api/auth/[...nextauth].ts or similar)
import { Auth, DefaultSession } from "next-auth";
import CredentialsProvider from "next-auth/providers/credentials";
import { JWT } from "next-auth/jwt";

// Example Better Auth config (adjust based on your actual setup)
export const authOptions = {
  providers: [
    CredentialsProvider({
      name: 'Credentials',
      credentials: {
        email: { label: "Email", type: "text" },
        password: { label: "Password", type: "password" }
      },
      async authorize(credentials) {
        // Your authentication logic here
        // Return user object if valid, null if not
      }
    })
  ],
  secret: process.env.BETTER_AUTH_SECRET,
  session: {
    strategy: 'jwt',
    maxAge: 30 * 24 * 60 * 60 // 30 days
  },
  callbacks: {
    async jwt({ token, user }: { token: JWT; user?: any }) {
      if (user) {
        token.id = user.id;
        token.accessToken = user.accessToken;
      }
      return token;
    },
    async session({ session, token }: { session: DefaultSession; token: JWT }) {
      if (token) {
        session.user.id = token.id as string;
        session.accessToken = token.accessToken as string;
      }
      return session;
    }
  }
};

export default Auth(authOptions);
```

### 9. Start the Applications

#### Option A: Separate Terminals
Terminal 1 (Backend):
```bash
uv run uvicorn backend.main:app --reload --port 8000
```

Terminal 2 (Frontend):
```bash
cd frontend
npm run dev
```

#### Option B: Using docker-compose (if available)
```bash
docker-compose up --build
```

#### Option C: Using a process manager like concurrently
```bash
# Install concurrently globally if not already installed
npm install -g concurrently

# Run both servers
concurrently "cd backend && uv run uvicorn backend.main:app --reload --port 8000" "cd frontend && npm run dev"
```

## Running the Applications

### Backend Server
```bash
# From repository root
uv run uvicorn backend.main:app --reload --port 8000
```

### Frontend Server
```bash
# From repository root
cd frontend && npm run dev
```

## Testing the Integration

1. Open your browser to `http://localhost:3000`
2. Register a new account or log in
3. Create a task - it should be saved to the PostgreSQL database
4. Refresh the page - the task should still be there
5. Test the recurrence feature by creating a recurring task and marking it as complete
6. Verify that a new instance of the recurring task appears
7. Test all features: create, complete, delete, update, priority, tags, due date, search, filter, sort

## Troubleshooting

### Common Issues

1. **Environment Variables Not Loading**
   - Ensure `.env` file is in the root directory
   - Check that the backend loads the environment variables properly with `load_dotenv()`

2. **JWT Token Issues**
   - Verify that the same secret is used in both frontend and backend
   - Check that the token is properly included in API requests

3. **Database Connection Issues**
   - Verify that the DATABASE_URL is correctly configured
   - Ensure the PostgreSQL/Neon database is accessible

4. **CORS Issues**
   - Check that your backend allows requests from your frontend origin
   - Add appropriate CORS middleware if needed

5. **Mock Data Still Present**
   - Search for any remaining mock data in frontend components
   - Ensure all API calls are using the real backend API

## Production Deployment Notes

### Backend Deployment
- Use a production WSGI/ASGI server like Gunicorn or Hypercorn instead of Uvicorn's dev server
- Set `--workers` parameter based on CPU cores
- Use environment variables for configuration

### Frontend Deployment
- Build the frontend with `npm run build`
- Host on platforms like Vercel, Netlify, or Cloudflare Pages
- Ensure environment variables are properly configured for production

### Security Considerations
- Use strong, randomly generated secrets for BETTER_AUTH_SECRET
- Use HTTPS in production
- Implement proper rate limiting
- Sanitize all user inputs
- Regularly update dependencies