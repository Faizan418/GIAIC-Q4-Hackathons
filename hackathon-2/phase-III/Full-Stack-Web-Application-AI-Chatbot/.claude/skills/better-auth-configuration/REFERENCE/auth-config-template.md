# Authentication Configuration Template

## Required Environment Variables
```env
# Primary secret for signing cookies/tokens
AUTH_SECRET=change_me_to_very_strong_random_64_chars

# Email configuration
EMAIL_FROM=hello@yourdomain.com
RESEND_KEY=re_1234567890
# or SMTP
SMTP_HOST=smtp.resend.com
SMTP_PORT=587
SMTP_USER=...
SMTP_PASS=...

# Social providers
GOOGLE_CLIENT_ID=...
GOOGLE_CLIENT_SECRET=...
GITHUB_CLIENT_ID=...
GITHUB_CLIENT_SECRET=...
Password Policy

Minimum 12 characters
Require uppercase, lowercase, number, symbol
Check against HaveIBeenPwned
No common passwords

Rate Limiting

10 registration attempts per IP per hour
5 login attempts per email per 15 minutes
3 password reset requests per email per hour