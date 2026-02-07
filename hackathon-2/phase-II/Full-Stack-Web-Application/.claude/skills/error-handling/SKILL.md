---
name: error-handling
description: Implements consistent global and custom error handling with standardized responses, logging, and proper HTTP status codes.
---

# Error Handling Skill

Consistent error handling makes APIs predictable and debuggable.

## Global Error Response Model
```python
class ErrorResponse(BaseModel):
    detail: str
    code: Optional[str] = None
    extra: Optional[dict] = None
Global Exception Handlers
Pythonfrom fastapi import Request
from fastapi.responses import JSONResponse

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail, "code": f"HTTP_{exc.status_code}"}
    )

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled error: {exc}")
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal Server Error", "code": "SERVER_ERROR"}
    )
Custom Business Exceptions
Pythonclass NotFoundError(Exception):
    def __init__(self, resource: str, id: any):
        self.message = f"{resource} with id {id} not found"

raise NotFoundError("User", 123)