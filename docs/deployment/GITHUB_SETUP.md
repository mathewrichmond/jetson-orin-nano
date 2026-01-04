# GitHub CI/CD Setup

## Overview

The Isaac robot system uses GitHub Actions for continuous integration, testing, and deployment.

## Workflows

### CI Workflow (`.github/workflows/ci.yml`)

Runs on every push and pull request:
- **Lint**: Format check (Black), import sorting (isort)
- **Test**: Run pytest tests with coverage
- **Build**: Build package artifact
- **Docker**: Test Docker build

### Release Workflow (`.github/workflows/release.yml`)

Runs when a release is created:
- **Build**: Creates release package
- **Checksums**: Generates SHA256 checksums
- **Upload**: Uploads to GitHub releases
- **Deploy**: Optionally deploys to target (if secrets configured)

### Deploy Dev Workflow (`.github/workflows/deploy-dev.yml`)

Manual workflow for deploying dev changes:
- **Deploy**: Syncs code to dev sandbox on target
- **Restart**: Restarts services

## GitHub Secrets Configuration

To enable deployment, configure these secrets in GitHub repository settings:

### Required Secrets

1. **`DEPLOY_HOST`**
   - Target hostname or IP address
   - Example: `isaac.local` or `192.168.1.100`

2. **`DEPLOY_USER`**
   - SSH username for target
   - Example: `nano`

3. **`DEPLOY_SSH_KEY`**
   - SSH private key for authentication
   - Generate: `ssh-keygen -t ed25519 -C "github-actions"`
   - Add public key to target: `~/.ssh/authorized_keys`

### Setting Up Secrets

1. Go to repository → Settings → Secrets and variables → Actions
2. Click "New repository secret"
3. Add each secret:
   - Name: `DEPLOY_HOST`
   - Value: Your target hostname/IP
4. Repeat for `DEPLOY_USER` and `DEPLOY_SSH_KEY`

### SSH Key Setup

**On development machine**:
```bash
ssh-keygen -t ed25519 -C "github-actions" -f ~/.ssh/github_actions
```

**Copy public key to target**:
```bash
ssh-copy-id -i ~/.ssh/github_actions.pub nano@isaac.local
```

**Add private key to GitHub secrets**:
```bash
cat ~/.ssh/github_actions
# Copy output and paste into DEPLOY_SSH_KEY secret
```

## Testing Workflows

### Test CI Locally

```bash
# Install act (GitHub Actions runner)
# https://github.com/nektos/act

# Run CI workflow
act push
```

### Test Release Workflow

```bash
# Create a test release
git tag v1.0.0-test
git push origin v1.0.0-test

# Or use workflow_dispatch in GitHub UI
```

## Workflow Triggers

### Automatic Triggers

- **CI**: On push to `main` or `develop`, on PR
- **Release**: On release creation

### Manual Triggers

- **Deploy Dev**: Use "Run workflow" button in GitHub Actions

## Troubleshooting

### Workflow Fails

1. Check workflow logs in GitHub Actions
2. Verify secrets are set correctly
3. Test SSH connection manually:
   ```bash
   ssh -i ~/.ssh/github_actions nano@isaac.local
   ```

### Deployment Fails

1. Check target host is reachable
2. Verify SSH key is authorized
3. Check target directory permissions
4. Review service logs on target

### Package Build Fails

1. Check for missing files
2. Verify package script permissions
3. Check disk space on runner

## Best Practices

1. **Test locally first**: Run scripts locally before pushing
2. **Use branches**: Test workflows on feature branches
3. **Monitor workflows**: Check GitHub Actions regularly
4. **Secure secrets**: Never commit secrets to repository
5. **Version packages**: Use semantic versioning for releases
