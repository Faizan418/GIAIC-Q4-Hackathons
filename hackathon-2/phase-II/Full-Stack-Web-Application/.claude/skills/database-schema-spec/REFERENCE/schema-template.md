# Database Schema Specification Template

## Change Summary
Brief one-line description.

## Motivation
Why this change is needed (link to user story or spec).

## Schema Changes

### Modified Tables

#### users
```sql
ALTER TABLE users ADD COLUMN theme VARCHAR(20) DEFAULT 'light';
ALTER TABLE users ADD COLUMN notifications_enabled BOOLEAN DEFAULT true;
New Tables (Alternative)
user_preferences
SQLCREATE TABLE user_preferences (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  key TEXT NOT NULL,
  value TEXT,
  updated_at TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(user_id, key)
);

CREATE INDEX idx_user_preferences_user_id ON user_preferences(user_id);
Migration Strategy

Deploy additive changes (new columns/table)
Backfill data via script
Update application code
(Future) Make columns non-nullable if needed

Rollback Plan
SQLALTER TABLE users DROP COLUMN theme;
-- or
DROP TABLE user_preferences;
Data Validation

Ensure defaults are applied to existing rows
Test queries with new indexes