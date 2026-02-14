# Playwright End-to-End Test Template

## Registration Flow Test
```ts
test('user can register and access dashboard', async ({ page }) => {
  // Visit registration page
  await page.goto('/register');
  
  // Fill form
  await page.fill('[name="name"]', 'Test User');
  await page.fill('[name="email"]', 'test@example.com');
  await page.fill('[name="password"]', 'StrongPass123!');
  
  // Submit
  await page.click('button[type="submit"]');
  
  // Assert redirected to verification page or dashboard
  await expect(page).toHaveURL(/.*dashboard/);
  
  // Assert welcome message
  await expect(page.locator('h1')).toContainText('Welcome');
});
Authenticated Fixture
TypeScripttest.use({ storageState: 'playwright/.auth/user.json' });

test('dashboard shows user data', async ({ page }) => {
  await page.goto('/dashboard');
  await expect(page.locator('text=Test User')).toBeVisible();
});