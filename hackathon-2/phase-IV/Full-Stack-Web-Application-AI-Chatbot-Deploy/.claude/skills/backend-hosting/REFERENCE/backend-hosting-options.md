# Backend Hosting Comparison

## Railway
- Excellent for monorepos
- Built-in database (Neon integration)
- Easy environment groups

## Fly.io
```toml
# fly.toml
app = "my-api"
kill_signal = "SIGINT"
kill_timeout = 5

[env]
  PORT = "8080"

[experimental]
  allowed_public_ports = []
  auto_rollback = true

[[services]]
  internal_port = 8080
  protocol = "tcp"

  [[services.ports]]
    port = 80
    handlers = ["http"]
Health Checks
Always configure:

/health endpoint returning 200
Startup probe
Liveness probe