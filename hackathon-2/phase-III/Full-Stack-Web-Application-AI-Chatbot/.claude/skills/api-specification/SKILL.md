---
name: api-specification
description: Creates clear, versioned REST/GraphQL API specifications with endpoints, request/response schemas, authentication, error handling, and examples.
---

# API Specification Skill

This skill defines clean, consistent API contracts that prevent integration issues.

## REST API Template

```markdown
## API: User Management

### GET /api/users/{id}
**Description**: Retrieve a user by ID  
**Authentication**: Required (JWT Bearer)

#### Path Parameters
| Name | Type   | Description         |
|------|--------|---------------------|
| id   | string | UUID of the user    |

#### Response 200
```json
{
  "id": "string",
  "name": "string",
  "email": "string",
  "createdAt": "datetime"
}
Error Responses

401 Unauthorized
404 Not Found → { "error": "User not found" }

POST /api/users
Description: Create a new user
Request Body
JSON{
  "name": "string (required)",
  "email": "string (required, valid email)",
  "password": "string (min 8 chars)"
}
Response 201
Location: /api/users/{new-id}
text## Best Practices
- Use consistent naming (kebab-case or camelCase)
- Version APIs (/v1/)
- Include authentication requirements
- Define error format globally
- Provide example requests/responses
- Use proper HTTP status codes
- Document pagination, filtering, sorting

Next: Define **database-schema-spec** for data changes.