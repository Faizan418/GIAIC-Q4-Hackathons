---
name: jwt-token-management
description: Implements secure JWT token creation, signing, verification, refresh token rotation, and blacklisting with short-lived access tokens and long-lived refresh tokens.
---

# JWT Token Management Skill

This skill implements the current gold standard for stateless authentication: short-lived access tokens + refresh token rotation.

## Recommended Strategy (2026 Best Practice)
- Access token: 15 minutes expiry (JWT)
- Refresh token: 7-30 days, stored in HttpOnly cookie
- Refresh token rotation (new refresh on each use)
- Refresh token reuse detection (blacklist old token)
- Separate token families to invalidate all sessions

## Token Structure

**Access Token Payload**
```json
{
  "sub": "user-id",
  "iat": 1700000000,
  "exp": 1700000900,
  "role": "user",
  "sessionId": "unique-session-id"
}
Refresh Token

Stored as random 32-byte hex in DB
Linked to user_id, session_id, expires_at, used_at

Flow Diagram (Mermaid)
sequenceDiagram
    autonumber
    actor User
    participant Frontend
    participant Backend
    participant DB

    %% Login Flow
    User ->> Frontend: Enter credentials
    Frontend ->> Backend: POST /login
    Backend ->> DB: Validate credentials
    DB -->> Backend: User record valid
    Backend -->> Frontend: Generate access + refresh tokens
    Backend -->> Frontend: Set refresh token cookie (HttpOnly)
    Frontend -->> User: Return access token

    %% Access Token Expiry
    User ->> Frontend: Request protected resource
    Frontend ->> Backend: API request with expired access token
    Backend -->> Frontend: 401 Unauthorized

    %% Refresh Token Flow
    Frontend ->> Backend: POST /refresh
    Backend ->> Backend: Validate refresh token
    Backend ->> Backend: Rotate refresh token
    Backend -->> Frontend: New refresh cookie (HttpOnly)
    Backend -->> Frontend: New access token

    %% Retry Original Request
    Frontend ->> Backend: Retry request with new access token
    Backend ->> DB: Fetch authorized data
    DB -->> Backend: Data
    Backend -->> Frontend: 200 OK (response)

