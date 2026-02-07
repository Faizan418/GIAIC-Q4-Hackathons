---
name: environment-variables
description: Manages secure environment variables across development, preview, staging, and production with secret handling, validation, and documentation.
---

# Environment Variables Skill

Proper env var management prevents configuration errors and security leaks.

## Required Variables Template

```env
# Database
DATABASE_URL=postgresql://user:pass@host:5432/dbname?sslmode=require

# Auth
AUTH_SECRET=super_long_random_64_chars_here
JWT_SECRET=another_very_long_random_string

# Frontend
NEXT_PUBLIC_APP_URL=https://yourapp.com

# External Services
RESEND_API_KEY=re_1234567890
STRIPE_SECRET_KEY=sk_live_...

# Misc
NODE_ENV=production
LOG_LEVEL=info
Environment Strategy

Environment,Source,Secrets Safe?,Auto-Injected
Local,.env.local,Yes (gitignored),No
Preview,Vercel/Railway Preview,Yes,Yes
Staging,Dedicated env,Yes,Yes
Production,Secret manager,Yes,Yes

EnvironmentSourceSecrets Safe?Auto-InjectedLocal.env.localYes (gitignored)NoPreviewVercel/Railway PreviewYesYesStagingDedicated envYesYesProductionSecret managerYesYes
Security Best Practices

Never commit .env files
Use different secrets per environment
Rotate secrets periodically
Validate required vars at startup
Prefix public vars with NEXT_PUBLIC_

Validation Example (Python)
Pythonfrom pydantic import BaseSettings

class Settings(BaseSettings):
    database_url: str
    auth_secret: str
    
    class Config:
        env_file = ".env"
        case_sensitive = False

settings = Settings()