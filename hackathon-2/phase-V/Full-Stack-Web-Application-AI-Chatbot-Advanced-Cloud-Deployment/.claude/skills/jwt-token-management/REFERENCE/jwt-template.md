# JWT Implementation Template

## Access Token Creation
```ts
const accessToken = jwt.sign(
  {
    sub: user.id,
    role: user.role,
    sessionId: session.id
  },
  process.env.JWT_SECRET!,
  { expiresIn: '15m' }
);
Refresh Token Rotation
TypeScript// On /refresh endpoint
const oldRefresh = getRefreshFromCookie();
const stored = await db.refreshToken.findUnique({ where: { token: oldRefresh } });

if (stored.used_at) throw new Error("Token reuse detected");

// Mark old as used
await db.refreshToken.update({ where: { id: stored.id }, data: { used_at: now } });

// Create new refresh token
const newRefresh = crypto.randomBytes(32).toString('hex');
await db.refreshToken.create({ data: { userId: user.id, token: newRefresh, expiresAt: ... } });

// Set new cookie
setCookie('refresh_token', newRefresh, { httpOnly: true, secure: true, sameSite: 'strict' });