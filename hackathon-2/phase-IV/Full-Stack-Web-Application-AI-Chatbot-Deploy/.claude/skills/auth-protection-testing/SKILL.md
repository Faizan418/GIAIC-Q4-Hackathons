---
name: auth-protection-testing
description: Verifies that authentication and authorization protections are enforced across all protected routes, APIs, and server actions with comprehensive unauthorized access tests.
---

# Auth Protection Testing Skill

This skill guarantees no unauthorized access to protected resources.

## Protection Layers to Test
- Public routes accessible without login
- Protected routes redirect to login when unauthenticated
- API endpoints return 401 without valid token
- Server Actions fail without session
- Role-based access (admin vs user)
- Session expiry handling

## Test Strategy

### 1. Route Protection Matrix

| Route             | Unauth Expected | Auth User Expected | Auth Admin Expected |
|-------------------|-----------------|--------------------|---------------------|
| /                 | 200             | 200                | 200                 |
| /dashboard        | Redirect login  | 200                | 200                 |
| /admin            | Redirect login  | Redirect login     | 200                 |
| /api/user/me      | 401             | 200                | 200                 |
| /api/admin/users  | 401             | 403                | 200                 |

### 2. Playwright Protection Test Template

```ts
test('unauthenticated user cannot access dashboard', async ({ page }) => {
  // Clear storage to ensure logged out
  await page.context().clearCookies();
  
  await page.goto('/dashboard');
  
  // Should redirect to login
  await expect(page).toHaveURL('/login');
  await expect(page.locator('h1')).toHaveText('Sign In');
});

test('authenticated non-admin cannot access admin panel', async ({ page }) => {
  await loginAsRegularUser(page);
  await page.goto('/admin');
  
  // Should show 403 or redirect
  await expect(page.locator('text=Forbidden')).toBeVisible();
});