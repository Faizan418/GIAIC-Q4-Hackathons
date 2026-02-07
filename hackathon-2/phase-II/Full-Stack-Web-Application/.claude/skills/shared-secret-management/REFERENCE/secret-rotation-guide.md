# Secret Rotation Guide

## JWT Secret Rotation (Zero Downtime)
1. Generate new secret (JWT_SECRET_NEW)
2. Deploy app that accepts both old and new secrets
3. Wait for all refresh tokens to expire (max 30 days)
4. Remove support for old secret
5. Update secret manager with only new value

## Database Credential Rotation
1. Create new DB user/password
2. Update application config
3. Restart/redeploy application
4. Revoke old credentials