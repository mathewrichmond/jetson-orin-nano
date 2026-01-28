#!/bin/bash
# Quick setup script for pre-commit hooks with Docker

set -e

echo "=== Pre-Commit Hook Setup ==="
echo ""

# Step 1: Docker permissions
echo "Step 1: Checking Docker permissions..."
if groups | grep -q docker; then
    echo "✅ User is in docker group"
else
    echo "⚠️  Adding user to docker group..."
    sudo usermod -aG docker $USER
    echo "✅ Added to docker group"
    echo "   Note: You may need to logout/login or run: newgrp docker"
fi

# Step 2: Install pre-commit
echo ""
echo "Step 2: Installing pre-commit..."
if command -v pre-commit &> /dev/null; then
    echo "✅ pre-commit already installed"
else
    pip install --user pre-commit
    echo "✅ pre-commit installed"
fi

# Step 3: Install hooks
echo ""
echo "Step 3: Installing pre-commit hooks..."
pre-commit install
echo "✅ Hooks installed"

# Step 4: Build Docker image
echo ""
echo "Step 4: Building Docker CI image (this may take 2-5 minutes)..."
make docker-build
echo "✅ Docker image built"

# Step 5: Test
echo ""
echo "Step 5: Testing hooks..."
echo "Running lint tests..."
make docker-lint
echo "✅ Lint tests passed"

echo ""
echo "==================================="
echo "✅ Setup Complete!"
echo ""
echo "Now you can:"
echo "  1. Commit normally: git commit -m 'message'"
echo "  2. Hooks will run automatically before commit"
echo "  3. Manual testing: make docker-test"
echo ""
echo "See SETUP_PRE_COMMIT.md for details"
echo "==================================="
