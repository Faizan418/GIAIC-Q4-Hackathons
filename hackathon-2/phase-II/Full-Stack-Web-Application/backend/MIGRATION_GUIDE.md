# Alembic Migration Guide

This document explains how to use Alembic for database migrations in the TASKAPP backend.

## Overview

Alembic is configured to manage database schema changes for the TASKAPP application. The configuration supports Neon PostgreSQL with async drivers.

## Configuration

- **Migration files**: Located in `alembic/versions/`
- **Configuration**: `alembic.ini` and `alembic/env.py`
- **Models**: Imported from `src/models/` (User, Task)

## Commands

### Generate a new migration
```bash
cd backend
python -m alembic revision --autogenerate -m "Description of changes"
```

### Apply migrations to database
```bash
cd backend
python -m alembic upgrade head
```

### Check current migration status
```bash
cd backend
python -m alembic current
```

### Show available migrations
```bash
cd backend
python -m alembic heads
```

## Important Notes

1. The application may create tables directly when run for the first time, which can cause conflicts with Alembic. This is normal for development.

2. For production deployments, always run migrations before starting the application:
   ```bash
   python -m alembic upgrade head
   ```

3. The `env.py` file automatically converts async PostgreSQL URLs (`postgresql+asyncpg://`) to sync URLs (`postgresql+psycopg2://`) for Alembic compatibility.

4. Always backup your database before running migrations in production.

## Troubleshooting

### Duplicate table error
If you encounter "relation already exists" errors, it means the tables were created outside of Alembic (e.g., by the application itself). In this case, you can mark the current state as up-to-date:
```bash
python -m alembic stamp head
```

### Database connection errors
Ensure your `DATABASE_URL` in the `.env` file is correct and accessible. For Neon PostgreSQL, the URL should follow the format:
```
postgresql://username:password@host.region.neon.tech/dbname?sslmode=require
```