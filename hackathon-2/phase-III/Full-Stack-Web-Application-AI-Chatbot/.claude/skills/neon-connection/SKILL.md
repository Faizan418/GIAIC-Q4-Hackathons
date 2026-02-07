---
name: neon-connection
description: Configures secure, performant connection to Neon serverless PostgreSQL with connection pooling, prepared statements, branching workflow, and environment-specific settings.
---

# Neon Connection Skill

Neon (serverless Postgres) is the recommended database for modern full-stack apps in 2026.

## Connection Best Practices
- Use connection pooling (PgBouncer or built-in)
- Enable prepared statements
- Use separate branches for dev/staging
- SSL required in production
- Connection limits awareness

## Environment Configuration

```env
# .env.local (development - dev branch)
DATABASE_URL=postgresql://user:pass@ep-cool-123456.us-east-2.aws.neon.tech/neondb?sslmode=require

# .env.production
DATABASE_URL=${NEON_PROD_URL}  # injected from secret manager
SQLModel + Neon Setup
Pythonfrom sqlmodel import create_engine
import os

DATABASE_URL = os.getenv("DATABASE_URL")

engine = create_engine(
    DATABASE_URL,
    pool_pre_ping=True,
    pool_size=10,
    max_overflow=20,
    pool_timeout=30,
    connect_args={"sslmode": "require"}
)