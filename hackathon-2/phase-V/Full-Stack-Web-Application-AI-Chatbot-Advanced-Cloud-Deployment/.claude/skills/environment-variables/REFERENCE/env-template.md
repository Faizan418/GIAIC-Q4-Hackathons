# Complete Environment Variables List

## Required (All Environments)
```env
DATABASE_URL=
AUTH_SECRET=
JWT_SECRET=
NEXT_PUBLIC_APP_URL=
Optional / Service-Specific
env# Email
RESEND_API_KEY=
EMAIL_FROM=hello@yourapp.com

# Analytics
POSTHOG_KEY=
POSTHOG_HOST=https://app.posthog.com

# Payments
STRIPE_SECRET_KEY=
STRIPE_WEBHOOK_SECRET=

# Feature Flags
LAUNCH_DARKLY_SDK_KEY=
.env.example (Commit this!)
envDATABASE_URL=postgresql://localhost:5432/myapp_dev
AUTH_SECRET=dev_secret_change_in_production
JWT_SECRET=dev_jwt_secret_change_in_production
NEXT_PUBLIC_APP_URL=http://localhost:3000
Runtime Validation (Next.js)
TypeScript// lib/env.ts
import { z } from 'zod';

const schema = z.object({
  DATABASE_URL: z.string().url(),
  AUTH_SECRET: z.string().min(32),
});

schema.parse(process.env);