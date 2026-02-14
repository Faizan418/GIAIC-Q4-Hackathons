# Shared Secret Best Practices

## Runtime Access
```python
import os
from dotenv import load_dotenv

load_dotenv()  # dev only

AUTH_SECRET = os.getenv("AUTH_SECRET")
if not AUTH_SECRET or len(AUTH_SECRET) < 64:
    raise ValueError("AUTH_SECRET must be at least 64 characters")
Production Secret Management

Vercel/Railway/Render built-in secrets
Doppler, AWS Secrets Manager, Hashicorp Vault
Separate secrets per environment

Validation at Startup
Python# app/core/security.py
def validate_secrets():
    required = ["AUTH_SECRET", "DATABASE_URL"]
    missing = [key for key in required if not os.getenv(key)]
    if missing:
        raise RuntimeError(f"Missing required secrets: {missing}")