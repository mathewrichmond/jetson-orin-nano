# Makefile for Isaac Robot System
# Provides convenient shortcuts for common tasks

.PHONY: help docker-build docker-lint docker-unit docker-test pre-commit-install pre-commit-test

# Docker CI image
DOCKER_IMAGE := isaac-robot-ci:local

.PHONY: help setup setup-docker build-docker run-docker clean reset test system-check

help:
	@echo "Isaac Robot System - Makefile Commands"
	@echo ""
	@echo "Quick Start (CI Testing):"
	@echo "  make docker-test        - Run lint + unit tests in Docker (CI consistency)"
	@echo "  make docker-lint        - Run only lint in Docker"
	@echo "  make docker-unit        - Run only unit tests in Docker"
	@echo "  make pre-commit-install - Install pre-commit hooks (auto-run docker tests)"
	@echo ""
	@echo "Setup:"
	@echo "  make setup              - Run setup script"
	@echo "  make setup-docker       - Build and setup Docker environment"
	@echo ""
	@echo "Docker:"
	@echo "  make docker-build       - Build lightweight CI Docker image"
	@echo "  make build-docker       - Build full Docker image"
	@echo "  make run-docker         - Run Docker container"
	@echo "  make shell-docker       - Run Docker container with shell"
	@echo ""
	@echo "Development:"
	@echo "  make env                - Activate development environment"
	@echo "  make build-ros          - Build ROS 2 workspace"
	@echo "  make test               - Run tests"
	@echo ""
	@echo "Maintenance:"
	@echo "  make system-check       - Run full system health check"
	@echo "  make clean              - Clean build artifacts"
	@echo "  make reset              - Reset setup state"
	@echo "  make update-packages    - Update system packages"

# === Docker CI Commands (Fast local testing) ===

docker-build: ## Build lightweight CI Docker image
	@echo "Building CI Docker image..."
	docker build -f Dockerfile.ci -t $(DOCKER_IMAGE) .
	@echo "✅ Docker image built: $(DOCKER_IMAGE)"

docker-lint: ## Run lint tests in Docker
	@./scripts/testing/docker_test.sh lint

docker-unit: ## Run unit tests in Docker
	@./scripts/testing/docker_test.sh unit

docker-test: ## Run lint + unit tests in Docker (pre-commit simulation)
	@./scripts/testing/docker_test.sh all

pre-commit-install: ## Install pre-commit hooks (Docker-based testing)
	@echo "=== Pre-Commit Hook Setup ==="
	@echo ""
	@echo "Step 1: Installing pre-commit..."
	@pip install --user pre-commit || pip3 install --user pre-commit
	@echo ""
	@echo "Step 2: Installing hooks..."
	@pre-commit install
	@echo ""
	@echo "Step 3: Building Docker image (first time only)..."
	@$(MAKE) docker-build
	@echo ""
	@echo "✅ Pre-commit hooks installed!"
	@echo ""
	@echo "Usage:"
	@echo "  git commit -m 'message'  # Hooks run automatically (~30s)"
	@echo "  make docker-test         # Manual testing"
	@echo ""
	@echo "See docs/PRE_COMMIT_DOCKER.md for details"

pre-commit-test: ## Test pre-commit hooks manually
	@echo "Running pre-commit hooks on all files..."
	@pre-commit run --all-files

# === Setup ===

setup:
	@echo "Running setup..."
	./setup.sh

setup-docker: build-docker run-docker
	@echo "Docker environment ready"

build-docker:
	@echo "Building Docker image..."
	docker-compose build

run-docker:
	@echo "Running Docker container..."
	docker-compose run --rm isaac-dev

shell-docker:
	@echo "Starting Docker shell..."
	docker-compose run --rm isaac-dev /bin/bash

env:
	@echo "Activating environment..."
	@bash -c "source scripts/utils/env_setup.sh; exec bash"

build-ros:
	@echo "Building ROS 2 workspace..."
	@bash -c "source scripts/utils/env_setup.sh; cd ~/ros2_ws && colcon build --symlink-install"

test: test-all ## Run all tests (lint, unit, integration)

test-lint: ## Run lint tests
	./scripts/testing/run_tests.sh lint

test-unit: ## Run unit tests
	./scripts/testing/run_tests.sh unit

test-integration: ## Run integration tests
	./scripts/testing/run_tests.sh integration

test-bench: ## Run bench tests (requires hardware)
	./scripts/testing/run_tests.sh bench

test-all: ## Run all tests (lint, unit, integration, no bench)
	./scripts/testing/run_tests.sh all

test-ros2: ## Run ROS 2 colcon tests
	@bash -c "source scripts/utils/env_setup.sh; cd ~/ros2_ws && colcon test && colcon test-result --verbose"

clean:
	@echo "Cleaning build artifacts..."
	rm -rf build/ install/ log/ .venv/ __pycache__/ *.pyc
	@echo "Clean complete"

reset:
	@echo "Resetting setup state..."
	rm -f .setup_state .setup.log
	@echo "Setup state reset. Run 'make setup' to reinstall."

system-check: ## Run full system health check (hostname, disk, memory, temp, network, SSD)
	@./scripts/monitoring/system_health_check.sh

update-packages:
	@echo "Updating packages..."
	sudo apt-get update
	python3 scripts/utils/package_manager.py install-system --groups dev_full
	python3 scripts/utils/package_manager.py install-python --groups dev_all
