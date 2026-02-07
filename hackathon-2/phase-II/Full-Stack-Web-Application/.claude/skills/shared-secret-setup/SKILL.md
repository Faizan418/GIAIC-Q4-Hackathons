---
name: shared-secret-setup
description: Properly generates, stores, rotates, and manages shared secrets for JWT signing, database encryption, and external API keys with secure practices.
---

# Shared Secret Setup Skill

Never hardcode secrets – always externalize and rotate.

## Secret Types & Requirements

| Secret              | Length    | Rotation Frequency | Storage Method             |
|---------------------|-----------|--------------------|----------------------------|
| AUTH_SECRET / JWT   | 64+ chars | 90 days            | Secret manager + env       |
| DATABASE_ENCRYPTION | 32 bytes  | On compromise      | Encrypted at rest          |
| EXTERNAL_API_KEY    | Per service | Per policy       | Vault/secret manager       |

## Generation
```python
import secrets
import string

def generate_secret(length: int = 64):
    alphabet = string.ascii_letters + string.digits + "!@#$%^&*"
    return ''.join(secrets.choice(alphabet) for _ in range(length))

# Use once, then store securely
print(generate_secret())
Environment Setup
env# Never commit this file with real values
AUTH_SECRET=your_very_long_random_secret_here_change_in_production
JWT_ALGORITHM=HS256
DATABASE_URL=postgresql://...
Rotation Strategy

Generate new secret
Deploy app that accepts both old + new (7-30 days overlap)
Invalidate old tokens
Remove old secret support
Update secret storage