# Vercel Best Practices 2026

## Environment Variables Setup
- Add all vars in Vercel dashboard
- Use different values for Preview, Staging, Production
- Never commit secrets

## Preview Deployments
- Enable "Preview Deployment Suffix" for clear URLs
- Use for QA and stakeholder review

## Performance Optimizations
- Enable "Speed Insights"
- Use Next.js Image component
- Leverage Edge Middleware for auth/geo redirects

## .vercelignore
node_modules
.git
.env*
*.tsbuildinfo
apps/web/.next