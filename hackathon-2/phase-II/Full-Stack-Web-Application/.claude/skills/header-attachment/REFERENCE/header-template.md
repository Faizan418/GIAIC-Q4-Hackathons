# Recommended Headers

## Security Headers
```python
HEADERS = {
    "Content-Security-Policy": "default-src 'self'; script-src 'self'",
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
    "X-XSS-Protection": "1; mode=block",
    "Referrer-Policy": "strict-origin-when-cross-origin",
    "Permissions-Policy": "geolocation=(), microphone=()"
}
Rate Limit Headers
Pythonresponse.headers["X-RateLimit-Limit"] = "100"
response.headers["X-RateLimit-Remaining"] = "42"
response.headers["X-RateLimit-Reset"] = "1700000000"