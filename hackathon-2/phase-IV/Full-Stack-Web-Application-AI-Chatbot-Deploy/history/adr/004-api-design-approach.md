# ADR-004: API Design and Endpoint Architecture

## Status
Accepted

## Date
2026-01-12

## Context
The backend API needs to provide a consistent, RESTful interface for the frontend to interact with task data. The API must support all required functionality including basic CRUD operations, advanced filtering and sorting, recurring tasks, and proper error handling. The design should be intuitive for frontend developers and maintain consistent patterns across all endpoints.

## Decision
We will implement a RESTful API with the following design principles:

- **Base URL**: `/api/{user_id}/` to enforce user context in path
- **Endpoints**: Standard CRUD endpoints with semantic HTTP methods
- **Response Format**: Consistent JSON structure with success/error patterns
- **Authentication**: JWT token in Authorization header for all endpoints
- **Query Parameters**: Standardized filtering, sorting, and pagination parameters
- **Error Handling**: Consistent error response format with appropriate HTTP status codes
- **Validation**: Pydantic schemas for all request/response validation

Specific endpoints:
- `GET /tasks` - List tasks with filtering/sorting/search capabilities
- `POST /tasks` - Create new task for user
- `GET /tasks/{id}` - Get specific task
- `PUT /tasks/{id}` - Update task fields
- `DELETE /tasks/{id}` - Delete task
- `PATCH /tasks/{id}/complete` - Toggle completion status

## Alternatives Considered
- **GraphQL API**: More flexible queries but higher complexity
- **RPC-style endpoints**: More direct function calls but less RESTful
- **Different URL structure**: Alternative user identification methods
- **Alternative response formats**: Different error handling patterns
- **API-first approach**: Design API with OpenAPI/Swagger first

## Consequences
### Positive
- RESTful design is familiar to most developers
- Consistent patterns make API easier to use and maintain
- Standard HTTP methods and status codes improve interoperability
- Proper separation of concerns between endpoints
- Clear mapping between HTTP verbs and operations
- Automatic OpenAPI documentation generation with FastAPI

### Negative
- REST constraints may not be optimal for all use cases
- More verbose than RPC-style APIs
- Potential over-fetching with standard REST endpoints
- More complex URL structures compared to RPC approaches

## References
- specs/1-backend-todo-api/plan.md
- specs/1-backend-todo-api/research.md
- specs/1-backend-todo-api/contracts/api-contracts.md