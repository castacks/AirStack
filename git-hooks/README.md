# Git Hooks

This directory contains git hooks used in the AirStack repository.

## Available Hooks

### pre-commit

The pre-commit hook automatically updates the `PROJECT_VERSION` in the `.env` file with the current git commit hash whenever Docker-related files (Dockerfile or docker-compose.yaml) are modified.

### Installation

To install the hooks:

1. Copy the hooks to your local .git/hooks directory:
   ```bash
   cp git-hooks/pre-commit .git/hooks/
   ```

2. Make sure the hook files are executable:
   ```bash
   chmod +x .git/hooks/pre-commit
   ```

## How the pre-commit Hook Works

1. When you commit changes, the hook checks if any Dockerfile or docker-compose.yaml files are being committed
2. If Docker-related files are detected, it updates the PROJECT_VERSION in the .env file with the current git commit hash
3. The modified .env file is automatically added to the commit

This approach eliminates version conflicts between parallel branches by ensuring Docker images are tagged with the exact commit they were built from.