# REST API Specification Template

## Base URL
`/api/v1`

## Global Error Format
```json
{
  "error": "string",
  "code": "ERROR_CODE",
  "details": { "optional": "object" }
}
Authentication
All endpoints require Bearer JWT token unless marked [Public].
Endpoints
GET /users/me
Description: Get current authenticated user profile
Auth: Required
Response 200
JSON{
  "id": "uuid",
  "name": "string",
  "email": "string",
  "role": "admin|user",
  "preferences": {
    "theme": "light|dark",
    "notifications": true
  }
}
PATCH /users/me
Description: Update user preferences
Request Body
JSON{
  "preferences": {
    "theme": "light|dark",
    "notifications": true|false
  }
}
Response 200
Updated user object
text**REFERENCE/openapi-example.md**
```markdown
# OpenAPI Snippet Example (Optional Export)

```yaml
paths:
  /users/me:
    get:
      summary: Get current user
      security:
        - bearerAuth: []
      responses:
        '200':
          description: Successful
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/User'