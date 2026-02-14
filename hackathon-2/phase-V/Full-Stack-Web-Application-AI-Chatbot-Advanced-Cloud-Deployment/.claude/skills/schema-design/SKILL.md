---
name: schema-design
description: Designs normalized, performant, and future-proof PostgreSQL database schemas with proper normalization, indexing strategy, constraints, partitioning considerations, and scalability planning.
---

# Schema Design Skill

This skill creates professional, production-ready PostgreSQL schemas following 2026 best practices.

## Core Design Principles
- 3NF normalization (with intentional denormalization for reads)
- Explicit primary keys (UUID or serial)
- Proper foreign key constraints
- Indexes on foreign keys and frequent WHERE/ORDER columns
- Use of generated columns where helpful
- JSONB for flexible data
- Partitioning readiness for large tables

## Recommended Naming Conventions
- tables: snake_case plural (users, order_items)
- columns: snake_case
- primary keys: id
- foreign keys: user_id, order_id
- indexes: idx_table_column(s)
- constraints: fk_table_column, uq_table_column

## Schema Design Process

### 1. Entity Identification
List core entities and relationships (one-to-one, one-to-many, many-to-many)

### 2. Normalization Check
Avoid data anomalies, but allow read-optimized views/materialized views

### 3. Indexing Strategy
```mermaid
graph TD
    A[Frequent Queries] --> B[WHERE clauses]
    B --> C[Add B-tree index]
    A --> D[JOIN columns]
    D --> E[Index foreign keys]
    A --> F[ORDER BY]
    F --> G[Index or composite]
    A --> H[Partial data]
    H --> I[JSONB fields → GIN index]