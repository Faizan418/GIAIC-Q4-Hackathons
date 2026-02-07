---
name: rest-endpoint-design
description: Designs clean, consistent, RESTful endpoints following industry standards with proper HTTP methods, status codes, versioning, naming conventions, and OpenAPI documentation.
---

# REST Endpoint Design Skill

This skill creates professional, predictable REST APIs that developers love.

## REST Conventions (2026 Standard)

| Action              | Method | Endpoint Pattern              | Status Code | Idempotent |
|---------------------|--------|-------------------------------|-------------|------------|
| List resources      | GET    | /api/v1/users                 | 200         | Yes        |
| Get single          | GET    | /api/v1/users/{id}            | 200 / 404   | Yes        |
| Create              | POST   | /api/v1/users                 | 201         | No         |
| Update (full)       | PUT    | /api/v1/users/{id}            | 200         | Yes        |
| Update (partial)    | PATCH  | /api/v1/users/{id}            | 200         | Yes        |
| Delete              | DELETE | /api/v1/users/{id}            | 204         | Yes        |

## Naming & Structure
- Plural nouns: /users, /posts, /orders
- Kebab-case or snake_case consistent
- Versioned: /api/v1/
- Nested for relationships: /users/{id}/posts
- Actions as resources when needed: /posts/{id}/publish

## Standard Responses
```json
// Success
{ "data": { ... } }

// Error (consistent)
{
  "detail": "User not found",
  "code": "USER_NOT_FOUND",
  "extra": { "user_id": 123 }
}
OpenAPI Best Practices

Use tags for grouping
Detailed descriptions
Example responses
Proper response models