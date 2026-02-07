# Neon Production Best Practices

## Connection Pooling
- Use Supavisor (Neon's built-in PgBouncer)
- Mode: transaction or session as needed
- Max connections: monitor usage

## Secrets Management
- Never commit DATABASE_URL
- Use Vercel/Render/Doppler secret injection
- Rotate credentials via Neon console

## Branching Strategy
- main → production
- staging → staging branch
- feature/* → temporary feature branches
- Reset dev branch weekly from main