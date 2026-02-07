# Research Document: Backend Todo API Implementation

## 1. Authentication Implementation Research

### JWT Implementation with PyJWT in FastAPI
- **Decision**: Use PyJWT for token verification with custom dependency
- **Rationale**: Lightweight, well-maintained library that integrates well with FastAPI's dependency injection system
- **Implementation approach**:
  - Create a dependency function that extracts and verifies JWT from Authorization header
  - Use BETTER_AUTH_SECRET from environment variables for verification
  - Return user information extracted from token payload

### Better Auth JWT Token Verification
- **Method**: Use `jwt.decode()` with the same secret used by Better Auth
- **Token structure**: Expect user information (id, email) in the token payload
- **Verification**: Check algorithm (likely HS256) and verify signature using shared secret

### Best Practices for Token Validation
- Always verify the token signature before trusting its contents
- Check for token expiration if present
- Validate that user_id in token matches the user_id in the request URL
- Return appropriate HTTP 401 for invalid/missing tokens

## 2. Database Connection Research

### Async PostgreSQL Connection with Neon using asyncpg
- **Decision**: Use asyncpg driver with SQLModel's async engine
- **Rationale**: asyncpg is the fastest PostgreSQL driver for Python, perfect for async FastAPI application
- **Connection string**: `postgresql+asyncpg://username:password@ep-project-name.region.aws.neon.tech/dbname?sslmode=require`

### Best Practices for Connection Pooling
- Use SQLModel's built-in async engine configuration
- Set appropriate pool size based on expected concurrent users
- Implement proper connection cleanup with FastAPI event handlers

### Optimal Configuration for Serverless Database
- Configure connection timeouts appropriate for serverless (longer idle timeout)
- Use connection pooling to minimize connection overhead
- Handle potential connection drops gracefully with retry logic

## 3. SQLModel Implementation Research

### SQLModel Best Practices for Table Definitions
- Use SQLModel's declarative base for model definitions
- Define proper relationships and foreign keys
- Use appropriate field types and constraints
- Implement proper indexing for frequently queried fields

### Relationship Patterns and Indexing Strategies
- Create foreign key relationship between Task and User (via user_id)
- Index user_id field for fast filtering
- Index completed field for status queries
- Index due_date field for date-based queries
- Index priority field for priority-based sorting

### Optimal Query Patterns for Filtering and Sorting
- Use SQLModel's select() statements with appropriate where clauses
- Build dynamic filters based on query parameters
- Use proper join strategies when needed
- Implement pagination for large result sets

## 4. Rate Limiting Implementation Research

### Rate Limiting Options for FastAPI Applications
- **Decision**: Use slowapi (Starlette-based rate limiter)
- **Rationale**: Easy integration with FastAPI, in-memory storage for simplicity, can be extended with Redis later

### In-Memory vs Redis Approaches
- Start with in-memory for development and simple deployments
- Redis approach for production with multiple instances
- Memory-based approach sufficient for initial implementation

### Suitable Packages for Rate Limiting
- slowapi: Built on Starlette, integrates well with FastAPI
- Simple configuration with decorator-based approach
- Supports various rate limiting strategies

## 5. Recurrence Logic Research

### Best Practices for Recurring Task Implementation
- Create new task instances when recurring tasks are completed
- Calculate next occurrence based on recurrence pattern (daily/weekly)
- Maintain original task properties in new instances
- Set completion status to False for new instances

### Algorithms for Calculating Next Occurrence Dates
- Daily: Add 1 day to current due date
- Weekly: Add 7 days to current due date
- Handle edge cases like month boundaries and leap years using Python's datetime library
- Store recurrence pattern in original task for reference

### Patterns for Handling Recurrence Edge Cases
- If original task has no due date, use current date as base
- Preserve title, description, priority, tags from original task
- Set new due date based on recurrence pattern
- Reset completion status to False
- Generate new ID for the recurring task instance