---
name: backend-hosting
description: Recommends and configures reliable hosting for FastAPI backend with options for Render, Railway, Fly.io, AWS, focusing on scalability, logs, metrics, and zero-downtime deploys.
---

# Backend Hosting Skill

Choose the best platform for your FastAPI backend needs.

## Recommended Hosting Options (2026)

| Platform     | Best For                          | Scaling          | Free Tier | Zero-Downtime |
|--------------|-----------------------------------|------------------|-----------|---------------|
| Render       | Simple + Postgres integration      | Auto           | Yes       | Yes           |
| Railway      | Fast deploys + team collaboration | Auto           | Yes       | Yes           |
| Fly.io       | Global edge + low latency         | Manual/Auto    | Limited   | Yes           |
| AWS ECS/Fargate | Enterprise control              | Full control   | No        | Yes           |

## Render.com Example Configuration

**render.yaml**
```yaml
services:
  - type: web
    name: api
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn app.main:app --host 0.0.0.0 --port $PORT
    envVars:
      DATABASE_URL: fromDatabase
      AUTH_SECRET: fromSecret
    plan: standard
    autoDeploy: true
    regions: ["oregon"]

graph TD
    A[Git Push to main] --> B[Hosting Platform Webhook]
    B --> C[Build Container]
    C --> D[Run Tests (optional)]
    D --> E[Deploy New Instance]
    E --> F[Health Check]
    F --> G[Switch Traffic<br/>(Zero Downtime)]
    G --> H[Old Instance Terminated]
    