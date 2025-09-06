#!/bin/bash

# Start ROS2 Development Container
set -e

echo "=========================================="
echo "Starting ROS2 Development Container"
echo "=========================================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi

# Stop any existing containers
echo "🛑 Stopping existing containers..."
docker-compose down

# Start the containers
echo "🚀 Starting ROS2 development container..."
docker-compose up -d

# Wait for container to be ready
echo "⏳ Waiting for container to be ready..."
sleep 5

# Check if container is running
if docker ps | grep -q "isaac-sim-ros2-dev"; then
    echo "✅ ROS2 development container is running!"
    echo ""
    echo "Container Information:"
    echo "- Name: isaac-sim-ros2-dev"
    echo "- Network: host mode"
    echo "- ROS Domain ID: 0"
    echo ""
    echo "To access the container:"
    echo "docker exec -it isaac-sim-ros2-dev bash"
    echo ""
    echo "To view logs:"
    echo "docker logs isaac-sim-ros2-dev"
    echo ""
    echo "To stop the container:"
    echo "docker-compose down"
else
    echo "❌ Failed to start ROS2 container. Check logs:"
    docker-compose logs
    exit 1
fi
