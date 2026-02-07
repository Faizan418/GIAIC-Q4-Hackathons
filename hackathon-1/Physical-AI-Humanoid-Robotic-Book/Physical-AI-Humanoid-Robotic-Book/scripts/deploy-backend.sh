#!/bin/bash

# Script to deploy the backend to Railway or similar platform
set -e

echo "Deploying backend application..."

# Check if railway CLI is installed, otherwise use docker
if command -v railway &> /dev/null; then
    echo "Deploying using Railway CLI..."
    cd backend
    railway up
elif command -v docker &> /dev/null; then
    echo "Deploying using Docker..."
    cd backend

    # Build the Docker image
    docker build -t ai-humanoid-backend .

    # Tag for deployment (replace with your registry)
    if [ ! -z "$DOCKER_REGISTRY" ]; then
        docker tag ai-humanoid-backend $DOCKER_REGISTRY/ai-humanoid-backend:$GITHUB_SHA
        docker push $DOCKER_REGISTRY/ai-humanoid-backend:$GITHUB_SHA
    fi
else
    echo "Error: Neither Railway CLI nor Docker is available for deployment"
    exit 1
fi

echo "Backend deployment completed successfully!"
