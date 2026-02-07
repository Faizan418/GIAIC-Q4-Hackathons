# Standardized Error Examples

## Validation Error Override
```python
from fastapi.exceptions import RequestValidationError

@app.exception_handler(RequestValidationError)
async def validation_handler(request: Request, exc: RequestValidationError):
    errors = []
    for error in exc.errors():
        errors.append({"field": ".".join(error["loc"]), "message": error["msg"]})
    return JSONResponse(status_code=422, content={"detail": errors, "code": "VALIDATION_ERROR"})
Common Custom Exceptions
Pythonclass AuthenticationError(HTTPException):
    def __init__(self):
        super().__init__(status_code=401, detail="Invalid credentials")

class PermissionError(HTTPException):
    def __init__(self):
        super().__init__(status_code=403, detail="Insufficient permissions")