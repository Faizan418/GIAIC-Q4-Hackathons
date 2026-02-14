# Zero-Downtime Migration Examples

## Expand VARCHAR length
```sql
-- Safe: PostgreSQL allows increasing length without lock
ALTER TABLE users ALTER COLUMN email TYPE VARCHAR(320);
Add NOT NULL column with default
SQLALTER TABLE posts ADD COLUMN view_count INT DEFAULT 0;
ALTER TABLE posts ALTER COLUMN view_count SET NOT NULL;
UPDATE posts SET view_count = 0 WHERE view_count IS NULL; -- redundant but clear
Strong Migration Pattern

Deploy new code that writes to both old and new columns
Backfill historical data
Deploy code that reads only from new column
Drop old column