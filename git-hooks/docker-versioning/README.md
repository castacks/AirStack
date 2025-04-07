# Docker Versioning Git Hook

This directory contains a git hook that automatically updates the Docker image tag with the current git commit hash.

## Hook: update-docker-image-tag.pre-commit

This pre-commit hook automatically updates the `DOCKER_IMAGE_TAG` in the `.env` file with the current git commit hash whenever Docker-related files (Dockerfile or docker-compose.yaml) are modified.

### Features

- Automatically updates `DOCKER_IMAGE_TAG` with the git commit hash
- Adds a comment above the variable indicating it's auto-generated
- Only triggers when Docker-related files are modified
- Automatically stages the modified .env file for commit

### Installation

To install the hook:

1. Copy the hook to your local .git/hooks directory:
   ```bash
   cp update-docker-image-tag.pre-commit ../../.git/hooks/pre-commit
   ```

2. Make sure the hook file is executable:
   ```bash
   chmod +x ../../.git/hooks/pre-commit
   ```

### Benefits

- Eliminates version conflicts between parallel branches
- Ensures Docker images are tagged with the exact commit they were built from
- Simplifies tracking which version of the code is running in Docker containers
- Provides a consistent and automated versioning system for Docker images