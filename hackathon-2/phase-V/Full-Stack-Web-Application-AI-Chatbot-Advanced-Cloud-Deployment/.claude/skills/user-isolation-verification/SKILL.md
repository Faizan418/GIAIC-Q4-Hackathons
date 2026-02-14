---
name: user-isolation-verification
description: Ensures strict data isolation between users in multi-tenant applications by testing that users can only access their own data across all endpoints and features.
---

# User Isolation Verification Skill

This skill prevents critical security bugs where one user can see/modify another's data.

## Isolation Test Strategy
- Create multiple test users (A, B, Admin)
- Perform actions as User A
- Verify User B cannot see/access User A's data
- Test all CRUD operations
- Test filtering, pagination, search

## Common Isolation Points
- User profile data
- Posts, comments, orders, files
- Dashboard aggregations
- Admin views (should see all, but regular users not)

## Test Pattern Template

```ts
test('user cannot see another user posts', async ({ page }) => {
  // Login as User A
  await loginAsUser(page, 'userA@example.com');
  
  // Create post as User A
  await createPost(page, "User A's secret post");
  
  // Logout and login as User B
  await logout(page);
  await loginAsUser(page, 'userB@example.com');
  
  await page.goto('/posts');
  
  // User B should NOT see User A's post
  await expect(page.locator('text=User A\'s secret post')).not.toBeVisible();
});

test('API returns only own data', async ({ request }) => {
  const tokenA = await getToken('userA');
  const response = await request.get('/api/posts', {
    headers: { Authorization: `Bearer ${tokenA}` }
  });
  
  const posts = await response.json();
  posts.forEach(post => {
    expect(post.author_id).toBe(userAId);
  });
});