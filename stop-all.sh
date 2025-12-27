#!/bin/bash

# Stop all services for Fubuni Physical Humanoid Robotics Project

echo "🛑 Stopping all services..."
echo "=================================================="

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

# Function to kill process on a port
kill_port() {
    local port=$1
    local service=$2
    local pid=$(lsof -ti:$port 2>/dev/null)
    if [ -n "$pid" ]; then
        echo -e "${RED}Stopping $service on port $port (PID: $pid)${NC}"
        kill -9 $pid 2>/dev/null || true
        sleep 1
        echo -e "${GREEN}✓ $service stopped${NC}"
    else
        echo "✓ $service is not running on port $port"
    fi
}

# Stop all services
kill_port 3000 "Frontend"
kill_port 4000 "Auth Service"
kill_port 8000 "Backend API"

echo ""
echo "=================================================="
echo -e "${GREEN}✓ All services stopped${NC}"
