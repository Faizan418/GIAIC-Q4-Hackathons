---
name: vercel-deployment
description: Configures optimal Vercel deployment for Next.js App Router projects with preview branches, environment variables, serverless functions, edge runtime, and performance optimizations.
---

# Vercel Deployment Skill

Vercel is the gold standard for hosting Next.js applications in 2026.

## Recommended Vercel Setup

### Project Structure for Vercel
my-monorepo/
├── apps/
│   └── web/                   # Next.js app (deploy this)
│       ├── app/
│       ├── public/
│       ├── next.config.js
│       └── package.json
├── packages/
├── vercel.json                # Optional overrides
└── .vercelignore
text### Key Vercel Features to Use
- Automatic previews for every PR
- Edge Runtime for middleware
- Serverless Functions for API routes
- Image Optimization
- Incremental Static Regeneration (ISR)
- Environment variables per environment

## vercel.json Configuration
```json
{
  "buildCommand": "turbo run build --filter=web",
  "outputDirectory": "apps/web/.next",
  "framework": "nextjs",
  "regions": ["iad1"],
  "cleanUrls": true,
  "trailingSlash": false,
  "rewrites": [
    {
      "source": "/api/(.*)",
      "destination": "/api/$1"
    }
  ]
}   

graph TD
    A[Git Push] --> B{Vercel Git Integration}
    B --> C[Preview Deployment<br/>(PR branch)]
    B --> D{Branch?}
    D -->|main| E[Production Deployment]
    D -->|staging| F[Staging Deployment]
    C --> G[Preview URL shared in PR]
    E --> H[Custom Domain + SSL]