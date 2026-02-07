# Session Management Template

## Active Sessions Endpoint
```ts
GET /api/auth/sessions

Response:
[
  {
    "id": "session-id",
    "device": "Chrome on macOS",
    "ip": "123.456.789.000",
    "location": "Karachi, Pakistan",
    "createdAt": "2026-01-01T10:00:00Z",
    "lastActive": "2026-01-01T12:30:00Z",
    "current": true
  }
]
Logout All Devices
TypeScriptPOST /api/auth/logout-all

// Invalidate current session + all others for user
await db.session.updateMany({
  where: { userId: user.id },
  data: { revoked: true }
});