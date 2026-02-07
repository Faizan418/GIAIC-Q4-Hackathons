---
name: session-handling
description: Manages user sessions with logout, concurrent session limits, session revocation, single sign-out, and audit logging.
---

# Session Handling Skill

This skill provides complete session lifecycle management for security and user control.

## Key Features
- Logout (invalidate current session)
- Logout all devices
- View active sessions
- Concurrent session limits (e.g., max 5 per user)
- Session audit trail

## Session Table Schema
```sql
CREATE TABLE sessions (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  last_used_at TIMESTAMPTZ DEFAULT NOW(),
  user_agent TEXT,
  ip_address TEXT,
  revoked BOOLEAN DEFAULT false
);
Session Flow (Mermaid)
sequenceDiagram
    autonumber
    actor User
    participant Device1
    participant Device2
    participant Backend

    %% Login on Device1
    User ->> Device1: Login
    Device1 ->> Backend: POST /login
    Backend -->> Device1: Session A created

    %% Login on Device2
    User ->> Device2: Login
    Device2 ->> Backend: POST /login
    Backend -->> Device2: Session B created

    %% Logout from all devices
    User ->> Device1: Logout from all devices
    Device1 ->> Backend: POST /logout-all
    Backend ->> Backend: Mark all user sessions revoked
    Backend -->> Device1: Success response

    %% Request with old token (after logout-all)
    Device2 ->> Backend: API request with old token
    Backend -->> Device2: 401 Unauthorized
Explanation of Flow
Multi-device login:

User can log in on multiple devices (Device1, Device2).

Backend creates separate sessions (A, B).

Logout from all devices:

User triggers "Logout from all devices".

Backend revokes all active sessions for that user.

Token invalidation:

Any requests with old tokens are rejected with 401 Unauthorized.

Ensures account security across devices.
