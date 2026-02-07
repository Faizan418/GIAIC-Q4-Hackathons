#!/bin/bash

# Script to build the Docusaurus frontend
set -e

echo "Building Docusaurus frontend..."

# Change to the docusaurus directory
cd docusaurus-textbook

# Install dependencies if node_modules doesn't exist
if [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    npm ci
fi

# Run the build command
npm run build

echo "Frontend build completed successfully!"
