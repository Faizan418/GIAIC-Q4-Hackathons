# User Isolation Test Examples

## Database Level Check (pytest)
```python
def test_user_isolation_db(db_session, user_a, user_b):
    # Create post as user_a
    post = Post(title="Secret", author_id=user_a.id)
    db_session.add(post)
    db_session.commit()
    
    # Query as user_b
    results = db_session.query(Post).filter(Post.author_id == user_b.id).all()
    assert len(results) == 0  # No leakage
    
    # Direct ID access should be forbidden in app logic
Admin Override (Allowed)
TypeScripttest('admin can see all data', async ({ page }) => {
  await loginAsAdmin(page);
  await page.goto('/admin/users');
  
  await expect(page.locator('text=userA@example.com')).toBeVisible();
  await expect(page.locator('text=userB@example.com')).toBeVisible();
});
Search Isolation
TypeScripttest('search only returns own data', async ({ page }) => {
  await loginAsUserA(page);
  await page.fill('[name="search"]', 'secret keyword from user A post');
  await page.press('[name="search"]', 'Enter');
  
  // Should find own post
  await expect(page.locator('text=Secret post')).toBeVisible();
  
  // Switch to user B - should not find
});