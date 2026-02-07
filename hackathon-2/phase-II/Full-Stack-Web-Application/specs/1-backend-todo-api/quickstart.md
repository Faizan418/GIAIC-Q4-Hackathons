# Quickstart Guide: Backend Todo API

## Prerequisites

- Python 3.10+
- pip package manager
- Git
- Neon Serverless PostgreSQL account
- Better Auth configured for frontend

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd Full-Stack-Web-Application
```

### 2. Create Environment Variables
Copy the example environment file and fill in your values:

```bash
cp .env.example .env
```

Edit `.env` with your values:
```bash
# Authentication (must be same for frontend & backend)
BETTER_AUTH_SECRET=your-very-long-random-secret-here

# Database (Neon serverless PostgreSQL)
DATABASE_URL=postgresql+asyncpg://user:password@ep-project-name-123456.us-east-2.aws.neon.tech/dbname?sslmode=require

# Optional overrides / development settings
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=10080
```

### 3. Navigate to Backend Directory
```bash
cd backend
```

### 4. Install Dependencies
```bash
pip install -r requirements.txt
```

If requirements.txt doesn't exist yet, install the core dependencies:
```bash
pip install fastapi uvicorn sqlmodel pyjwt python-multipart python-dotenv asyncpg psycopg2-binary slowapi
```

## Running the Application

### Development
```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Production
```bash
uvicorn main:app --host 0.0.0.0 --port 8000
```

## API Endpoints

Once the application is running, you can access the API at:
- Base URL: `http://localhost:8000/api/{user_id}/`
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

### Example Requests

#### Get all tasks for a user
```bash
curl -X GET "http://localhost:8000/api/user123/tasks" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN"
```

#### Create a new task
```bash
curl -X POST "http://localhost:8000/api/user123/tasks" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "New task",
    "description": "Task description",
    "priority": "medium",
    "tags": ["work", "important"],
    "due_date": "2023-12-31T10:00:00Z",
    "recurrence": "none"
  }'
```

## Configuration

### Environment Variables
- `BETTER_AUTH_SECRET`: Secret key used by Better Auth (shared with frontend)
- `DATABASE_URL`: PostgreSQL connection string
- `JWT_ALGORITHM`: Algorithm for JWT verification (default: HS256)
- `ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time (default: 10080 minutes / 7 days)

### Rate Limiting
The API implements rate limiting at 100 requests per IP per hour. This can be adjusted in the `slowapi` configuration.

## Troubleshooting

### Common Issues

1. **Database Connection Issues**
   - Verify your Neon PostgreSQL connection string is correct
   - Ensure the database is accessible from your network
   - Check that the database has been initialized

2. **JWT Authentication Issues**
   - Verify that `BETTER_AUTH_SECRET` matches the one used by Better Auth
   - Ensure JWT tokens are properly formatted: `Bearer {token}`
   - Check that the token hasn't expired

3. **Environment Variables Not Loading**
   - Ensure `.env` file is in the project root directory
   - Verify the file is named `.env` and not `.env.example`
   - Check that python-dotenv is properly installed and configured

### API Documentation
- Visit `http://localhost:8000/docs` for interactive API documentation
- Visit `http://localhost:8000/redoc` for ReDoc documentation

## Next Steps

1. Implement the data models as defined in the data-model.md
2. Create the database tables using SQLModel
3. Implement the API endpoints following the contracts in api-contracts.md
4. Add authentication middleware using the JWT implementation
5. Implement the business logic for recurring tasks and filtering