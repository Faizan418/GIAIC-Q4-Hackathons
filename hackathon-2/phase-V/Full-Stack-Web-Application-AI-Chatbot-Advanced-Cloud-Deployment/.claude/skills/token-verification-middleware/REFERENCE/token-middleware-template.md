# Token Verification Examples

## Header + Cookie Fallback
```python
def extract_token(request: Request):
    # Try header first
    auth = request.headers.get("Authorization")
    if auth and auth.startswith("Bearer "):
        return auth[7:]
    
    # Fallback to cookie
    return request.cookies.get("access_token")
Refresh Token Check
Pythonasync def get_current_user_or_none(token: str = Depends(oauth2_scheme_optional)):
    if not token:
        return None
    try:
        return verify_and_get_user(token)
    except:
        return None