# Git Hooks

This directory contains git hooks used in the AirStack repository.

## Available Hooks

### Docker Versioning Hook

The `update-docker-image-tag.pre-commit` hook automatically updates the `DOCKER_IMAGE_TAG` in the `.env` file with the current git commit hash whenever Docker-related files (Dockerfile or docker-compose.yaml) are modified. It also adds a comment above the variable indicating that the value is auto-generated from the git commit hash.

This ensures that Docker images are always tagged with the exact commit they were built from, eliminating version conflicts between parallel branches.

### Installation

To install the hooks:

1. Copy the hook to your local .git/hooks directory:
   ```bash
   cp git-hooks/docker-versioning/update-docker-image-tag.pre-commit .git/hooks/pre-commit
   ```

2. Make sure the hook file is executable:
   ```bash
   chmod +x .git/hooks/pre-commit
   ```

## How the Docker Versioning Hook Works

1. When you commit changes, the hook checks if any Dockerfile or docker-compose.yaml files are being committed
2. If Docker-related files are detected, it updates the DOCKER_IMAGE_TAG in the .env file with the current git commit hash and adds a comment above the variable
3. The modified .env file is automatically added to the commit

This approach eliminates version conflicts between parallel branches by ensuring Docker images are tagged with the exact commit they were built from.