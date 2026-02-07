#!/bin/bash

# Script to deploy the Docusaurus frontend to Vercel
set -e

echo "Deploying Docusaurus frontend to Vercel..."

# Check if Vercel CLI is installed
if ! command -v vercel &> /dev/null; then
    echo "Installing Vercel CLI..."
    npm install -g vercel
fi

# Change to the docusaurus directory
cd docusaurus-textbook

# Deploy to Vercel
if [ "$VERCEL_ENV" = "production" ]; then
    echo "Deploying to production..."
    vercel --prod --token=$VERCEL_TOKEN
else
    echo "Deploying to preview environment..."
    vercel --token=$VERCEL_TOKEN
fi

echo "Frontend deployment completed successfully!"
