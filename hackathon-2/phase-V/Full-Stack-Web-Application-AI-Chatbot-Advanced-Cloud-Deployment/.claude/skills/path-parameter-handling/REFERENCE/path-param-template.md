# Path Parameter Templates

## Integer ID with Validation
```python
user_id: int = Path(..., gt=0, le=1000000, example=42)
Slug Format
Pythonfrom fastapi import Path
import re

def validate_slug(slug: str = Path(..., regex=r"^[a-z0-9-]+$")):
    return slug
Existence Dependency
Pythonasync def get_user_or_404(user_id: int, db: Session = Depends(get_db)):
    user = await crud.get_user(db, user_id)
    if not user:
        raise HTTPException(404, "User not found")
    return user

```