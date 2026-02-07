---
name: better-auth-configuration
description: Configures a secure, modern authentication system with best practices for registration, login, password handling, email verification, social logins, and security hardening.
---

# Better Auth Configuration Skill

This skill sets up a robust, production-ready authentication foundation following 2026 security best practices.

## Core Principles
- Passwords never stored in plain text (bcrypt/argon2)
- Rate limiting on auth endpoints
- Secure cookie settings (HttpOnly, Secure, SameSite)
- Email verification for new accounts
- Support for social providers (Google, GitHub, etc.)
- Multi-factor authentication readiness

## Recommended Configuration Structure

### 1. Environment Variables
```env
AUTH_SECRET=strong-random-32-byte-hex
JWT_SECRET=strong-random-32-byte-hex (fallback)
DATABASE_URL=...
EMAIL_FROM=noreply@yourapp.com
SMTP_HOST=...
2. User Table Requirements

id (UUID or bigint)
email (unique, lowercase)
email_verified (boolean/timestamp)
password_hash (string, nullable for social logins)
name
image/avatar
role (enum: user, admin, etc.)
created_at, updated_at

3. Security Best Practices

Enforce strong password policy
Rate limit login attempts (e.g., 5 per email per 15min)
Account lockout after failures
Secure password reset flow with short-lived tokens
CSRF protection
Content Security Policy headers

4. Flow Diagrams
Registration Flow (Mermaid)
sequenceDiagram
    participant User
    participant Frontend
    participant Backend
    participant DB
    participant Email

    User->>Frontend: Submit registration
    Frontend->>Backend: POST /api/auth/register
    Backend->>DB: Create user (hashed password)
    Backend->>Email: Send verification email
    User->>Frontend: Click verification link
    Frontend->>Backend: GET /verify?token=...
    Backend->>DB: Mark email_verified = true