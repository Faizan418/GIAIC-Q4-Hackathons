# ADR-003: Data Architecture and Modeling Strategy

## Status
Accepted

## Date
2026-01-12

## Context
The backend needs to efficiently store and retrieve task data for multiple users while ensuring data isolation, supporting advanced querying capabilities (filtering, sorting, search), and maintaining referential integrity. The data architecture must support the frontend's requirements for task management features including priorities, tags, due dates, and recurring tasks.

## Decision
We will implement the following data architecture:

- **ORM**: SQLModel for database modeling (combines SQLAlchemy + Pydantic)
- **Database**: PostgreSQL via Neon Serverless for managed, scalable storage
- **Models**: Task model with user_id foreign key for data isolation
- **Indexing**: Strategic indexes on user_id, completed, due_date, and priority fields
- **Relationships**: One-to-many relationship between User (from Better Auth) and Task
- **Field Types**: JSONB for tags array, proper datetime handling with timezone awareness

## Alternatives Considered
- **Database alternatives**: SQLite, MongoDB, MySQL, PostgreSQL with traditional hosting
- **ORM alternatives**: Pure SQLAlchemy, TortoiseORM, Peewee, SQL Alchemy ORM
- **Data modeling alternatives**: Denormalized storage, document-based approach
- **Indexing strategies**: Different field combinations, composite indexes

## Consequences
### Positive
- SQLModel provides both Pydantic validation and SQLAlchemy ORM power
- PostgreSQL offers robust ACID compliance and advanced querying features
- Proper indexing enables efficient filtering and sorting operations
- JSONB for tags provides flexible querying capabilities
- Clear separation between user data through foreign key relationships
- Timezone-aware datetime handling prevents date-related issues

### Negative
- PostgreSQL learning curve for team members familiar with other databases
- SQLModel is relatively newer than pure SQLAlchemy
- More complex than document-based storage for simple use cases
- Additional overhead of maintaining proper relationships and constraints

## References
- specs/1-backend-todo-api/plan.md
- specs/1-backend-todo-api/research.md
- specs/1-backend-todo-api/data-model.md