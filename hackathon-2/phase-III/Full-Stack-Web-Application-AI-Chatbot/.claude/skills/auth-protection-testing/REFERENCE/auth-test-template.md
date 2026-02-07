# Comprehensive Auth Protection Tests

## API Protection Test (pytest)
```python
def test_protected_endpoint_unauthenticated(client):
    response = client.get("/api/user/me")
    assert response.status_code == 401
    assert response.json()["detail"] == "Not authenticated"

def test_protected_endpoint_authenticated(client, authenticated_user):
    response = client.get("/api/user/me")
    assert response.status_code == 200
    assert response.json()["email"] == authenticated_user.email
Role-Based Protection
TypeScripttest('user cannot access admin API', async ({ request }) => {
  const token = await getUserToken(); // regular user
  const response = await request.get('/api/admin/users', {
    headers: { Authorization: `Bearer ${token}` }
  });
  
  expect(response.status()).toBe(403);
});
Session Expiry Test
TypeScripttest('expired session redirects to login', async ({ page }) => {
  // Mock expired token/cookie
  await page.goto('/dashboard');
  await expect(page).toHaveURL('/login?expired=true');
});