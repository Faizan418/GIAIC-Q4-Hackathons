# TaskApp Professional Styling and Authentication

This feature implements a professional task management application with modern styling, authentication, and dashboard functionality.

## Features

- **Modern UI**: Professional styling with dark/light theme support
- **Authentication**: Secure signup and login with JWT authentication
- **Task Management**: Create, read, update, and delete tasks
- **Responsive Design**: Works on all device sizes

## Architecture

### Frontend
- Next.js 16+ with App Router
- TypeScript for type safety
- Tailwind CSS for styling
- React Hook Form + Zod for form validation
- Better Auth for authentication

### Backend
- FastAPI for the API framework
- SQLModel for database modeling
- PostgreSQL with Neon Serverless
- JWT for authentication

## Getting Started

### Prerequisites
- Node.js 20.x
- Python 3.11
- PostgreSQL database

### Installation

1. Clone the repository
2. Install backend dependencies:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```
3. Install frontend dependencies:
   ```bash
   cd frontend
   npm install
   ```
4. Set up environment variables (see `.env.example`)
5. Run the applications:
   ```bash
   # Backend
   cd backend
   uvicorn src.main:app --reload

   # Frontend
   cd frontend
   npm run dev
   ```

## API Endpoints

### Authentication
- `POST /api/auth/signup` - Register a new user
- `POST /api/auth/login` - Authenticate a user
- `GET /api/auth/me` - Get current user
- `POST /api/auth/logout` - Logout user

### Tasks
- `GET /api/{user_id}/tasks` - Get all user tasks
- `POST /api/{user_id}/tasks` - Create a new task
- `GET /api/{user_id}/tasks/{id}` - Get a specific task
- `PUT /api/{user_id}/tasks/{id}` - Update a specific task
- `DELETE /api/{user_id}/tasks/{id}` - Delete a specific task
- `PATCH /api/{user_id}/tasks/{id}/complete` - Toggle task completion status

## Security

- JWT-based authentication
- Password hashing with bcrypt
- Input validation on both frontend and backend
- SQL injection protection through SQLModel/SQLAlchemy