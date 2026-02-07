#!/bin/bash

# Script to run database migrations
set -e

echo "Running database migrations..."

cd backend

# Check if alembic is installed
if ! command -v alembic &> /dev/null; then
    echo "Installing alembic..."
    pip install alembic
fi

# Run the migrations
alembic upgrade head

echo "Database migrations completed successfully!"
