---
name: migration-planning
description: Plans safe, reversible database migrations with zero-downtime strategies, backfill plans, rollback procedures, and coordination with application releases.
---

# Migration Planning Skill

This skill ensures database changes are deployed safely in production.

## Migration Types & Strategies

| Change Type              | Downtime | Strategy                          | Rollback                      |
|--------------------------|----------|-----------------------------------|-------------------------------|
| Add column (nullable)     | None     | Direct ALTER                      | DROP COLUMN                   |
| Add column (NOT NULL)     | None     | Add nullable → backfill → NOT NULL| Reverse steps                 |
| Rename column            | None     | Add new → dual write → backfill → drop old | Reverse                     |
| Change data type         | Possible | Add new column → backfill → swap  | Keep old column longer        |
| Add index (concurrently) | None     | CREATE INDEX CONCURRENTLY         | DROP INDEX CONCURRENTLY       |

## Standard Migration Template

```markdown
## Migration: Add user preferences column

### Goal
Support theme and notification preferences.

### Changes
```sql
-- 1. Add column (nullable with default)
ALTER TABLE users ADD COLUMN preferences JSONB DEFAULT '{}';

-- 2. Create index
CREATE INDEX CONCURRENTLY idx_users_preferences ON users USING GIN (preferences);
Backfill Plan

Run script to set existing users: {"theme": "light", "notifications": true}
Monitor for completion

Application Changes

Deploy code that reads from new column
Feature flag if needed

Rollback Plan
SQLDROP INDEX CONCURRENTLY idx_users_preferences;
ALTER TABLE users DROP COLUMN preferences;
Verification

Check row counts
Sample queries
Monitor error rates