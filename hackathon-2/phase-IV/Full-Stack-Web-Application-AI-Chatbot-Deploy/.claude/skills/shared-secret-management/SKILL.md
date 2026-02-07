---
name: shared-secret-management
description: Manages secure storage, rotation, and usage of secrets (API keys, JWT secrets, database credentials) using environment variables, secret managers, and rotation strategies.
---

# Shared Secret Management Skill

This skill ensures secrets are never hardcoded and can be rotated safely.

## Best Practices (2026)
- Never commit secrets to git
- Use environment variables in development
- Use secret manager in production (Vercel, AWS Secrets Manager, Doppler, etc.)
- Rotate secrets regularly
- Separate secrets per environment (dev/staging/prod)

## Secret Types & Rotation Strategy

| Secret Type          | Rotation Frequency | Downtime Required | Strategy                          |
|----------------------|--------------------|-------------------|-----------------------------------|
| JWT_SECRET           | Every 90 days      | No                | Dual active secrets during transition |
| DATABASE_URL         | On breach/compromise | Yes             | Generate new, update app, restart |
| API keys (external)  | Per policy         | No                | Revoke old, create new            |
| SMTP credentials     | Every 180 days     | No                | Update env + restart              |

## Rotation Flow (Mermaid)

```mermaid
sequenceDiagram
    participant Admin
    participant SecretManager
    participant App

    Admin->>SecretManager: Generate new JWT_SECRET_v2
    Admin->>App: Deploy with OLD_SECRET + NEW_SECRET support
    Note over App: Accept both secrets for 7 days
    Admin->>SecretManager: Deprecate old secret
    Admin->>App: Deploy with only NEW_SECRET
Environment Template
env# .env.local (never commit)
AUTH_SECRET=super_long_random_string_here
JWT_SECRET=another_very_long_random_string

# Production: use secret manager injection