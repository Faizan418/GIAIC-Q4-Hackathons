# Quickstart: CI/CD & Deployment Workflow

## Prerequisites

- GitHub account with repository access
- Vercel account for frontend deployment
- Cloud platform account (e.g., Railway) for backend deployment
- Neon Postgres account for database
- Qdrant Cloud account for vector database
- Cohere API key for embeddings

## Setup

1. **Configure GitHub Secrets**:
   Add the following secrets to your GitHub repository:
   ```
   VERCEL_TOKEN - Vercel deployment token
   VERCEL_ORG_ID - Vercel organization ID
   VERCEL_PROJECT_ID - Vercel project ID
   BACKEND_DATABASE_URL - Neon Postgres connection string
   VECTOR_DATABASE_URL - Qdrant Cloud URL
   COHERE_API_KEY - Cohere API key
   ```

2. **Create GitHub Actions workflow**:
   ```bash
   mkdir -p .github/workflows
   touch .github/workflows/deploy.yml
   ```

3. **Set up deployment scripts**:
   ```bash
   mkdir -p scripts
   touch scripts/deploy-backend.sh
   touch scripts/deploy-frontend.sh
   touch scripts/populate-embeddings.py
   ```

## Basic Usage

1. **Trigger deployment**:
   Push code changes to the repository or create a pull request to trigger the CI/CD pipeline automatically.

2. **Monitor deployment**:
   Check the Actions tab in GitHub to monitor the progress of your deployment pipeline.

3. **Verify deployment**:
   - Frontend: Visit your Vercel deployment URL
   - Backend: Check the API endpoint for the deployed backend
   - Database: Verify migrations were applied successfully
   - Vector DB: Confirm embeddings were populated

## Development Workflow

1. **Local development**:
   ```bash
   # Run backend locally
   cd backend
   pip install -r requirements.txt
   uvicorn main:app --reload

   # Run frontend locally
   cd frontend/docusaurus-textbook
   npm install
   npm run start
   ```

2. **Code changes**:
   - Make changes to code
   - Run local tests: `pytest` (backend) and `npm test` (frontend)
   - Commit and push changes to trigger CI/CD

3. **Deployment verification**:
   - Check GitHub Actions for successful pipeline execution
   - Verify all components are working together
   - Test the RAG functionality end-to-end

## Testing

1. **Local testing**:
   ```bash
   # Backend tests
   cd backend
   pytest tests/

   # Frontend tests
   cd frontend/docusaurus-textbook
   npm run test
   ```

2. **CI testing**:
   Tests run automatically in the GitHub Actions pipeline
   - Code linting with flake8 and mypy
   - Unit and integration tests with pytest
   - Frontend tests with Jest

## Building for Production

1. **Frontend build**:
   ```bash
   cd frontend/docusaurus-textbook
   npm run build
   ```

2. **Backend container** (if using containerization):
   ```bash
   cd backend
   docker build -t rag-backend .
   ```

## Troubleshooting

1. **Failed deployments**:
   - Check GitHub Actions logs for error details
   - Verify all environment variables are properly set
   - Ensure database migrations are compatible

2. **Embedding population issues**:
   - Check Cohere API key validity
   - Verify Qdrant Cloud connection
   - Ensure content is properly formatted for processing

3. **Rollback procedure**:
   - Use the rollback endpoint in the deployment API
   - Or manually deploy a previous git tag
