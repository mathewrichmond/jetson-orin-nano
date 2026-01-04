# Dockerfile for Isaac Robot System
# Supports development and testing in containerized environment

FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set working directory
WORKDIR /workspace

# Install base system packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user (matching Jetson setup)
RUN useradd -m -s /bin/bash nano && \
    echo "nano ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Copy project files
COPY --chown=nano:nano . /workspace

# Switch to non-root user
USER nano

# Set up environment
ENV PATH="/home/nano/.local/bin:$PATH"
ENV PYTHONPATH="/workspace:$PYTHONPATH"

# Run setup script
RUN /workspace/setup.sh || true

# Default command
CMD ["/bin/bash"]

