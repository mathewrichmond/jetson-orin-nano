# Makefile for Isaac Robot System
# Provides convenient shortcuts for common tasks

.PHONY: help setup setup-docker build-docker run-docker clean reset test

help:
	@echo "Isaac Robot System - Makefile Commands"
	@echo ""
	@echo "Setup:"
	@echo "  make setup              - Run setup script"
	@echo "  make setup-docker       - Build and setup Docker environment"
	@echo ""
	@echo "Docker:"
	@echo "  make build-docker       - Build Docker image"
	@echo "  make run-docker         - Run Docker container"
	@echo "  make shell-docker       - Run Docker container with shell"
	@echo ""
	@echo "Development:"
	@echo "  make env                - Activate development environment"
	@echo "  make build-ros          - Build ROS 2 workspace"
	@echo "  make test               - Run tests"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean              - Clean build artifacts"
	@echo "  make reset              - Reset setup state"
	@echo "  make update-packages    - Update system packages"

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

update-packages:
	@echo "Updating packages..."
	sudo apt-get update
	python3 scripts/utils/package_manager.py install-system --groups dev_full
	python3 scripts/utils/package_manager.py install-python --groups dev_all
