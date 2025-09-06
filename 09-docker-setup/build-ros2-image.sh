#!/bin/bash

# Build ROS2 Development Image for Isaac Sim Integration
set -e

echo "=========================================="
echo "Building ROS2 Development Image"
echo "=========================================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi

# Build the image
echo "🔨 Building ROS2 Humble development image..."
docker-compose build --no-cache

echo "✅ ROS2 development image built successfully!"
echo ""
echo "Next steps:"
echo "1. Run: ./start-ros2-container.sh"
echo "2. In another terminal, run: ./start-isaac-sim.sh"
echo ""
echo "To access the container:"
echo "docker exec -it isaac-sim-ros2-dev bash"
