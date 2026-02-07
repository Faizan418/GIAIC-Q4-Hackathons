# CI/CD & Deployment Workflow Documentation

## Overview

This document describes the CI/CD and deployment workflow for the Physical AI & Humanoid Robotics textbook project. The workflow automates linting, testing, building, and deployment of both frontend (Docusaurus) and backend (FastAPI) components with proper database and vector database integration.

## Architecture

The CI/CD pipeline consists of the following components:

- **GitHub Actions**: Orchestrates the entire deployment process
- **Vercel**: Hosts the frontend Docusaurus application
- **Railway**: Hosts the backend FastAPI application
- **Neon Postgres**: Stores metadata and application data
- **Qdrant Cloud**: Stores vector embeddings for RAG functionality

## Workflow Components

### 1. Code Quality Checks

The pipeline includes automated linting and testing:

- **Linting**: Uses flake8 and mypy for Python code quality
- **Backend Testing**: Uses pytest with database and vector store services
- **Frontend Testing**: Uses type checking and build validation for Docusaurus

### 2. Deployment Process

The deployment process follows this sequence:

1. **Linting & Testing**: All code is checked for quality
2. **Build Backend**: Docker image is built and pushed to registry
3. **Run Migrations**: Database schema is updated
4. **Deploy Frontend**: Docusaurus site is deployed to Vercel
5. **Deploy Backend**: Application is deployed to Railway
6. **Populate Vector DB**: Textbook content is embedded and stored in Qdrant

### 3. Configuration Files

- `.github/workflows/lint.yml` - Code quality checks
- `.github/workflows/test-backend.yml` - Backend testing
- `.github/workflows/test-frontend.yml` - Frontend testing
- `.github/workflows/deploy-full.yml` - Complete deployment pipeline
- `scripts/build-frontend.sh` - Frontend build script
- `scripts/deploy-frontend.sh` - Frontend deployment script
- `scripts/deploy-backend.sh` - Backend deployment script
- `scripts/run-migrations.sh` - Database migration script
- `scripts/populate-embeddings.py` - Vector database population script

## Required Secrets

The deployment requires the following GitHub repository secrets:

- `COHERE_API_KEY` - API key for Cohere embedding service
- `QDRANT_URL` - URL for Qdrant vector database
- `QDRANT_API_KEY` - API key for Qdrant vector database
- `DATABASE_URL` - Connection string for PostgreSQL database
- `VERCEL_TOKEN` - Token for Vercel deployment
- `RAILWAY_TOKEN` - Token for Railway deployment

## Local Development Setup

To run the pipeline locally, ensure you have:

- Python 3.12
- Node.js 18+
- Docker
- Vercel CLI (for frontend deployment)
- Railway CLI (for backend deployment)

## Running Tests Locally

### Backend Tests
```bash
cd backend
pip install -r requirements.txt
python -m pytest tests/ -v
```

### Frontend Tests
```bash
cd docusaurus-textbook
npm install
npm run typecheck
npm run build
```

## Deployment Triggers

The pipeline runs automatically on:

- Push to the `main` branch
- Pull requests to the `main` branch (for testing only)

## Rollback Strategy

In case of deployment failure:

1. The pipeline will stop at the failing step
2. Previous versions remain available
3. Manual rollback can be performed by redeploying a previous version
4. Database migrations can be rolled back using Alembic commands

## Monitoring and Observability

The deployment includes:

- Health check endpoints for backend monitoring
- Test coverage reporting
- Error logging and monitoring
- Performance metrics collection

## Security Considerations

- All secrets are stored in GitHub repository secrets
- No sensitive data is committed to the repository
- Docker images are built with minimal base images
- Regular security scanning is performed on dependencies

## Troubleshooting

### Common Issues

1. **Deployment fails due to timeout**: Increase timeout values in the deployment scripts
2. **Database migration fails**: Check database connection strings and permissions
3. **Vector database population fails**: Verify Cohere API key and Qdrant connection
4. **Frontend build fails**: Check Node.js version and dependency compatibility

### Debugging Steps

1. Check GitHub Actions logs for detailed error messages
2. Verify all required secrets are properly configured
3. Test locally before pushing changes
4. Review the deployment configuration files

## Maintenance

Regular maintenance tasks include:

- Updating dependencies
- Monitoring deployment success rates
- Reviewing and optimizing deployment times
- Updating security scanning tools
- Maintaining test coverage

## Future Enhancements

- Implement approval processes for production deployments
- Add performance testing to the pipeline
- Enhance monitoring and alerting capabilities
- Implement blue-green deployment strategy
- Add comprehensive end-to-end testing
