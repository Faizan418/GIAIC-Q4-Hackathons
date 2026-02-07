---
name: database-schema-spec
description: Designs and documents database schema changes with tables, columns, indexes, relationships, migration strategy, and backward compatibility considerations.
---

# Database Schema Specification Skill

This skill creates safe, reviewable database schema changes with clear migration paths.

## Schema Change Template

```markdown
## Database Schema Change: Add User Preferences

### Motivation
Support theme selection and notification preferences.

### New/Modified Tables

#### users table (modification)
Add new columns:
| Column           | Type    | Nullable | Default | Description                  |
|------------------|---------|----------|---------|------------------------------|
| theme            | varchar | YES      | 'light' | User preferred theme         |
| notifications    | boolean | NO       | true    | Enable email notifications   |

#### New Table: user_preferences (alternative approach)
| Column           | Type       | Constraints              | Description                  |
|------------------|------------|--------------------------|------------------------------|
| user_id          | uuid       | PK, FK → users.id        | Reference to user            |
| preference_key   | varchar    | PK                       | e.g., 'theme', 'language'    |
| preference_value | varchar    |                          | Value                        |
| updated_at       | timestamp  | DEFAULT now()            | Last update                  |

### Indexes
- Index on users.theme for frequent queries
- Composite index on user_preferences(user_id, preference_key)

### Relationships
- One-to-many: users → user_preferences

### Data Migration Strategy
1. Add columns with defaults
2. Backfill existing users (theme = 'light')
3. Deploy code that reads new fields
4. Make columns non-nullable (future)

### Backward Compatibility
- Old code continues to work (defaults applied)
- No breaking changes in existing endpoints

### Rollback Plan
- Drop added columns
- Restore from backup if needed
Best Practices

Prefer additive changes
Never remove columns without deprecation period
Use defaults and backfill
Add indexes for new query patterns
Document migration scripts location
Include rollback strategy

This completes the full development workflow!
textBhai, ab tumhare paas **total 10 powerful skills** hain jo ek complete product development cycle cover karte hain:

1–5: Monorepo foundation  
6–10: Specification & implementation planning

Sab skills interconnected hain aur ek logical flow follow karte hain.

Bas folders banao aur upload kar do — yeh tumhara AI-powered engineering team ka backbone ban jaayega! 🔥

Agar REFERENCE files bhi in naye skills ke liye chahiye ya koi adjustment, bata do!