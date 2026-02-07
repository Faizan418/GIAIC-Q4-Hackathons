# Data Model: CI/CD & Deployment Workflow

## Entity: DeploymentConfiguration

**Description**: Configuration parameters required for deploying the application components

**Fields**:
- `id`: string (unique identifier for the configuration)
- `environment`: 'development' | 'staging' | 'production' (target environment)
- `backendUrl`: string (URL for the backend API)
- `frontendUrl`: string (URL for the frontend application)
- `databaseUrl`: string (connection string for Neon Postgres)
- `vectorDatabaseUrl`: string (connection string for Qdrant)
- `cohereApiKey`: string (API key for Cohere embeddings)
- `vercelToken`: string (Vercel deployment token)
- `createdAt`: Date (when the configuration was created)
- `updatedAt`: Date (when the configuration was last updated)

**Validation Rules**:
- All URL fields must be valid URLs
- API keys must be properly formatted
- Environment must be one of the allowed values

## Entity: DeploymentJob

**Description**: Represents a single deployment job execution

**Fields**:
- `id`: string (unique identifier for the job)
- `jobType`: 'lint' | 'test' | 'build-frontend' | 'build-backend' | 'deploy-frontend' | 'deploy-backend' | 'migrate-db' | 'populate-embeddings' (type of job)
- `status`: 'pending' | 'running' | 'success' | 'failure' | 'cancelled' (current status)
- `startTime`: Date (when the job started)
- `endTime?`: Date (when the job completed, optional until completion)
- `logs`: string (output logs from the job)
- `commitHash`: string (git commit hash being deployed)
- `branch`: string (git branch being deployed from)
- `triggeredBy`: string (user or system that triggered the job)

**Validation Rules**:
- Status must be one of the allowed values
- startTime must be before endTime (when present)
- commitHash must be a valid git hash format

## Entity: DeploymentArtifact

**Description**: Build artifacts generated during the deployment process

**Fields**:
- `id`: string (unique identifier for the artifact)
- `artifactType`: 'frontend-build' | 'backend-container' | 'embeddings-data' (type of artifact)
- `storagePath`: string (path where the artifact is stored)
- `size`: number (size of the artifact in bytes)
- `checksum`: string (SHA256 checksum for integrity verification)
- `createdAt`: Date (when the artifact was created)
- `expiresAt`: Date (when the artifact expires and should be cleaned up)
- `jobId`: string (ID of the job that created this artifact)

**Validation Rules**:
- checksum must be a valid SHA256 hash
- expiresAt must be in the future
- artifactType must be one of the allowed values

## Entity: DeploymentEvent

**Description**: Log of deployment-related events for monitoring and auditing

**Fields**:
- `id`: string (unique identifier for the event)
- `eventType`: 'pipeline-started' | 'job-started' | 'job-completed' | 'job-failed' | 'deployment-success' | 'deployment-failure' | 'rollback-initiated' | 'rollback-completed' (type of event)
- `timestamp`: Date (when the event occurred)
- `message`: string (descriptive message about the event)
- `metadata`: object (additional context-specific data)
- `jobId?`: string (ID of the associated job, if applicable)
- `deploymentId?`: string (ID of the associated deployment, if applicable)

**Validation Rules**:
- eventType must be one of the allowed values
- timestamp must be in the past or present

## State Transitions

### DeploymentJob State Transitions
```
pending → running → success
              ↘ failure
              ↘ cancelled
```

### DeploymentConfiguration State Transitions
```
created → active → deprecated
```

## Relationships

- DeploymentConfiguration contains multiple DeploymentJob entities (one configuration can trigger multiple jobs)
- DeploymentJob generates multiple DeploymentArtifact entities (one job can create multiple artifacts)
- DeploymentEvent references both DeploymentJob and DeploymentConfiguration entities for context
- DeploymentJob may be associated with zero or one DeploymentEvent for each major state change
