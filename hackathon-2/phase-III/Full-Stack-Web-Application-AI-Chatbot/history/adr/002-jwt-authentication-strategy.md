# ADR-002: JWT-Based Authentication Strategy

## Status
Accepted

## Date
2026-01-12

## Context
The backend API needs to securely authenticate requests and enforce user data isolation. The system must verify JWT tokens issued by Better Auth and ensure that users can only access their own data. The authentication mechanism should be stateless, scalable, and integrate seamlessly with the frontend's authentication flow.

## Decision
We will implement JWT-based authentication using PyJWT for token verification with the following approach:

- **Token Verification**: Use PyJWT to decode and verify JWT tokens from Authorization header
- **Shared Secret**: Use BETTER_AUTH_SECRET from environment variables to verify token signatures
- **User Extraction**: Extract user_id from token payload and validate against URL parameter
- **Middleware**: Implement FastAPI dependency for token validation on protected endpoints
- **Authorization**: Enforce user_id matching between token and request URL to prevent cross-user access

## Alternatives Considered
- **Session-based authentication**: Server-side session storage with cookies
- **OAuth2 with Bearer tokens**: Using FastAPI's built-in OAuth2 schemes
- **API Keys**: Simple API key authentication for each request
- **Custom token format**: Self-managed token system instead of JWT

## Consequences
### Positive
- Stateless authentication scales well with serverless deployments
- Seamless integration with Better Auth's existing JWT system
- Clear separation of authentication and authorization concerns
- Efficient token validation without database lookups
- Natural fit with RESTful API architecture

### Negative
- Tokens cannot be invalidated before expiration (revocation challenge)
- Larger payload size compared to session identifiers
- Need for proper secret management across services
- Potential complexity with token refresh mechanisms in the future

## References
- specs/1-backend-todo-api/plan.md
- specs/1-backend-todo-api/research.md
- specs/1-backend-todo-api/data-model.md