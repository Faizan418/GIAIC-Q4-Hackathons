---
name: header-attachment
description: Properly attaches custom headers to responses including CORS, security headers, rate limit info, request IDs, and caching directives.
---

# Header Attachment Skill

Consistent headers improve security, debugging, and client experience.

## Essential Security Headers
- Content-Security-Policy
- X-Content-Type-Options: nosniff
- X-Frame-Options: DENY
- Referrer-Policy: strict-origin-when-cross-origin
- Permissions-Policy

## Implementation Options

### Global Middleware
```python
@app.middleware("http")
async def add_security_headers(request: Request, call_next):
    response = await call_next(request)
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
    return response
Per-Response
Pythonresponse.headers["X-Request-ID"] = request.state.request_id
response.headers["Cache-Control"] = "no-store"