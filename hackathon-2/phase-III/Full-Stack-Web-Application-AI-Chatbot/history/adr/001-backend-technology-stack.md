# ADR-001: Backend Technology Stack Selection

## Status
Accepted

## Date
2026-01-12

## Context
We need to select a technology stack for the backend API of the Todo application that will integrate with the existing frontend and provide secure, scalable task management functionality. The backend must support JWT authentication from Better Auth, handle multi-user data isolation, and provide efficient CRUD operations with advanced features like filtering, sorting, and recurring tasks.

## Decision
We will use the following integrated backend technology stack:

- **Framework**: FastAPI for async API routing, validation, and OpenAPI documentation
- **ORM**: SQLModel for database models and queries (combines SQLAlchemy + Pydantic)
- **Database Driver**: asyncpg for asynchronous PostgreSQL connections
- **Validation**: Pydantic for request/response validation and type safety
- **Authentication**: PyJWT for JWT token verification
- **Rate Limiting**: slowapi for request rate limiting

## Alternatives Considered
- **Framework alternatives**: Django REST Framework, Flask + Flask-RESTful, Express.js
- **ORM alternatives**: Pure SQLAlchemy, TortoiseORM, Peewee
- **Database drivers**: psycopg2, aiopg
- **Validation alternatives**: Marshmallow, Cerberus
- **Authentication alternatives**: OAuth2 with FastAPI's built-in security, python-jose

## Consequences
### Positive
- FastAPI provides excellent async support and automatic API documentation
- SQLModel combines SQLAlchemy's power with Pydantic's type safety
- asyncpg is the fastest PostgreSQL driver for Python
- Strong type safety through Pydantic integration
- Built-in support for async operations and concurrency
- Excellent performance characteristics for the requirements

### Negative
- Additional learning curve for team members unfamiliar with FastAPI/SQLModel
- Some ecosystem immaturity compared to more established frameworks
- Potential complexity in deployment due to async nature

## References
- specs/1-backend-todo-api/plan.md
- specs/1-backend-todo-api/research.md
- specs/1-backend-todo-api/data-model.md